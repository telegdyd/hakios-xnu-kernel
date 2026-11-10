/*
 * sweetd: a tiny remote command runner for the phone's iOS userland. From ios-jasmine (ios/sweetd.c, MIT,
 * the same author's Mi A2 port), by way of ios-vayu: launchd starts it as root (com.hakios.sweetd.plist),
 * it listens on TCP port 5143 of the USB Ethernet link (en2, boot-arg usb_net=1: CDC-NCM, the phone is
 * 192.168.7.2) and runs one command per connection:
 *
 *   request:  "RUN <token> <timeout-seconds> <argc>
" then argc lines, one
 *             argument each (argv[0] is an absolute path)
 *   reply:    the command's stdout and stderr, merged, as they come, then
 *             "
" SWEETD_END " <status>
" where status is the exit code,
 *             -SIGNAL when killed, or -1000 when it could not be started
 *
 * argv[0] without a slash is one of the built-in commands below instead (iOS has no ls or cat, and
 * the image has no package prefix yet): ls, cat, sysctl, put (a file from the client: "put PATH MODE
 * SIZE", then SIZE bytes after the argument lines), rm, mkdir, kill, and two that end the run:
 * "loader" (back to the loader, which waits for fastboot commands) and "reboot" (back to the loader, which
 * resets the phone: Android). Both go through kern.hakios.loader_request and take about a second.
 *
 * It also answers DHCP on the link (a child process): the PC is given 192.168.7.1/24, no router, no DNS, so
 * that nobody has to set the address by hand. Windows, Linux and macOS ask on their own once the adapter
 * appears. The address is fixed; only the one client on the cable is served. The answer goes out as a whole
 * Ethernet frame through BPF on en2: the kernel will not route 255.255.255.255 without a default route
 * ("Network is unreachable", even with IP_BOUND_IF), and the client has no address to send to yet.
 *
 * The client is devices/negroni/tools/sweet-sh.py. Only peers on the USB Ethernet link (192.168.7.0/24)
 * are served, and only with the token: TOKEN_IMAGE (build.sh copies userland/sweetd.token, made on the PC
 * and never in git, into the image), else TOKEN_PATH. Without either nothing runs. Clients are served
 * one at a time.
 */
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sysctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define PORT            5143
#define MAX_ARGS        64
#define MAX_LINE        4096
#define SWEETD_END      "__SWEETD_END__"
#define TOKEN_IMAGE     "/redwood/etc/sweetd.token"
#define TOKEN_PATH      "/private/var/root/sweetd.token"

extern char **environ;

static int
read_line(int fd, char *buf, size_t max)
{
	size_t n = 0;
	while (n < max - 1) {
		char c;
		ssize_t r = read(fd, &c, 1);
		if (r <= 0) {
			return -1;
		}
		if (c == '\n') {
			break;
		}
		buf[n++] = c;
	}
	buf[n] = 0;
	return (int)n;
}

static void
send_all(int fd, const char *buf, size_t len)
{
	while (len > 0) {
		ssize_t w = write(fd, buf, len);
		if (w <= 0) {
			return;
		}
		buf += w;
		len -= (size_t)w;
	}
}

/* The token from TOKEN_PATH (first line, at most 64 characters), or "" when there is none. */
static void
load_token(char *out, size_t max)
{
	out[0] = 0;
	FILE *f = fopen(TOKEN_IMAGE, "r");
	if (f == NULL) {
		f = fopen(TOKEN_PATH, "r");
	}
	if (f == NULL) {
		return;
	}
	if (fgets(out, (int)max, f) == NULL) {
		out[0] = 0;
	}
	fclose(f);
	out[strcspn(out, "\r\n")] = 0;
}

/* Constant-time comparison, so the token cannot be found byte by byte. */
static bool
token_ok(const char *got, const char *want)
{
	size_t n = strlen(want);
	if (n < 16 || strlen(got) != n) {
		return false;
	}
	unsigned char d = 0;
	for (size_t i = 0; i < n; i++) {
		d |= (unsigned char)(got[i] ^ want[i]);
	}
	return d == 0;
}

static int
bi_ls(int cs, const char *path)
{
	struct stat st;
	DIR *dir;
	struct dirent *e;
	char full[1024], link[1024];

	if (lstat(path, &st) != 0) {
		dprintf(cs, "ls: %s: %s\n", path, strerror(errno));
		return 1;
	}
	if (!S_ISDIR(st.st_mode)) {
		dprintf(cs, "%06o %5u %5u %10lld %s\n", st.st_mode, st.st_uid, st.st_gid, (long long)st.st_size, path);
		return 0;
	}
	dir = opendir(path);
	if (dir == NULL) {
		dprintf(cs, "ls: %s: %s\n", path, strerror(errno));
		return 1;
	}
	while ((e = readdir(dir)) != NULL) {
		ssize_t n;

		snprintf(full, sizeof(full), "%s/%s", path, e->d_name);
		if (lstat(full, &st) != 0) {
			dprintf(cs, "?????? %s: %s\n", e->d_name, strerror(errno));
			continue;
		}
		dprintf(cs, "%06o %5u %5u %10lld %s", st.st_mode, st.st_uid, st.st_gid, (long long)st.st_size, e->d_name);
		if (S_ISLNK(st.st_mode) && (n = readlink(full, link, sizeof(link) - 1)) >= 0) {
			link[n] = 0;
			dprintf(cs, " -> %s", link);
		}
		dprintf(cs, "\n");
	}
	closedir(dir);
	return 0;
}

static int
bi_cat(int cs, const char *path)
{
	char buf[16384];
	ssize_t n;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		dprintf(cs, "cat: %s: %s\n", path, strerror(errno));
		return 1;
	}
	while ((n = read(fd, buf, sizeof(buf))) > 0) {
		send_all(cs, buf, (size_t)n);
	}
	close(fd);
	return n < 0 ? 1 : 0;
}

static int
bi_sysctl(int cs, const char *name)
{
	unsigned char buf[4096];
	size_t len = sizeof(buf) - 1;
	bool text = true;

	if (sysctlbyname(name, buf, &len, NULL, 0) != 0) {
		dprintf(cs, "sysctl: %s: %s\n", name, strerror(errno));
		return 1;
	}
	for (size_t i = 0; i < len; i++) {
		if ((buf[i] < 0x20 && buf[i] != '\n' && buf[i] != '\t' && !(buf[i] == 0 && i == len - 1)) || buf[i] > 0x7e) {
			text = false;
		}
	}
	if (len == 4 && !text) {
		dprintf(cs, "%s: %d\n", name, *(int *)buf);
	} else if (len == 8 && !text) {
		dprintf(cs, "%s: %lld\n", name, *(long long *)buf);
	} else if (text) {
		buf[len] = 0;
		dprintf(cs, "%s: %s\n", name, (char *)buf);
	} else {
		dprintf(cs, "%s: %zu bytes:", name, len);
		for (size_t i = 0; i < len && i < 256; i++) {
			dprintf(cs, " %02x", buf[i]);
		}
		dprintf(cs, "\n");
	}
	return 0;
}

/* "put PATH MODE SIZE": SIZE bytes from the client after the argument lines, into PATH (made, or emptied) */
static int
bi_put(int cs, const char *path, const char *mode, const char *size)
{
	long long left = strtoll(size, NULL, 10), got = 0;
	char buf[16384];
	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, (mode_t)strtol(mode, NULL, 8));
	bool ok = fd >= 0;

	if (!ok) {
		dprintf(cs, "put: %s: %s\n", path, strerror(errno));
	}
	while (left > 0) {
		ssize_t n = read(cs, buf, left < (long long)sizeof(buf) ? (size_t)left : sizeof(buf));

		if (n <= 0) {
			dprintf(cs, "put: the client stopped after %lld bytes\n", got);
			ok = false;
			break;
		}
		if (fd >= 0 && write(fd, buf, (size_t)n) != n) {
			dprintf(cs, "put: %s: %s\n", path, strerror(errno));
			close(fd);
			fd = -1;
			ok = false;
		}
		left -= n;
		got += n;
	}
	if (fd >= 0) {
		fchmod(fd, (mode_t)strtol(mode, NULL, 8));
		if (fsync(fd) != 0 || close(fd) != 0) {
			ok = false;
		}
	}
	if (ok) {
		dprintf(cs, "put: %s: %lld bytes\n", path, got);
	}
	return ok ? 0 : 1;
}

/* A built-in command (argv[0] has no slash): its status, or -2 when there is no such built-in. */
static int
builtin(int cs, int argc, char **argv)
{
	int status = 0;

	if (strcmp(argv[0], "ls") == 0) {
		if (argc == 1) {
			return bi_ls(cs, "/");
		}
		for (int i = 1; i < argc; i++) {
			if (argc > 2) {
				dprintf(cs, "%s:\n", argv[i]);
			}
			status |= bi_ls(cs, argv[i]);
		}
		return status;
	}
	if (strcmp(argv[0], "cat") == 0) {
		for (int i = 1; i < argc; i++) {
			status |= bi_cat(cs, argv[i]);
		}
		return status;
	}
	if (strcmp(argv[0], "sysctl") == 0) {
		for (int i = 1; i < argc; i++) {
			status |= bi_sysctl(cs, argv[i]);
		}
		return status;
	}
	if (strcmp(argv[0], "put") == 0) {
		if (argc != 4) {
			dprintf(cs, "put: put PATH MODE SIZE\n");
			return 2;
		}
		return bi_put(cs, argv[1], argv[2], argv[3]);
	}
	if (strcmp(argv[0], "rm") == 0 || strcmp(argv[0], "mkdir") == 0) {
		bool rm = argv[0][0] == 'r';

		for (int i = 1; i < argc; i++) {
			if ((rm ? (unlink(argv[i]) != 0 && rmdir(argv[i]) != 0) : mkdir(argv[i], 0755) != 0)) {
				dprintf(cs, "%s: %s: %s\n", argv[0], argv[i], strerror(errno));
				status = 1;
			}
		}
		return status;
	}
	if (strcmp(argv[0], "loader") == 0 || strcmp(argv[0], "reboot") == 0) {
		int mode = argv[0][0] == 'r' ? 2 : 1;

		sync();
		if (sysctlbyname("kern.hakios.loader_request", NULL, NULL, &mode, sizeof(mode)) != 0) {
			dprintf(cs, "%s: kern.hakios.loader_request: %s\n", argv[0], strerror(errno));
			return 1;
		}
		dprintf(cs, "%s: asked; the phone goes %s within a few seconds\n", argv[0],
		    mode == 2 ? "back to Android" : "back to the loader");
		return 0;
	}
	if (strcmp(argv[0], "kill") == 0) {
		int sig = SIGTERM, i = 1;

		if (argc > 2 && argv[1][0] == '-') {
			sig = atoi(argv[1] + 1);
			i = 2;
		}
		for (; i < argc; i++) {
			if (kill(atoi(argv[i]), sig) != 0) {
				dprintf(cs, "kill: %s: %s\n", argv[i], strerror(errno));
				status = 1;
			}
		}
		return status;
	}
	return -2;
}

static void
serve(int cs)
{
	char line[MAX_LINE], token[80], want[80];
	char *argv[MAX_ARGS + 1];
	int timeout = 0, argc = 0;

	load_token(want, sizeof(want));
	if (read_line(cs, line, sizeof(line)) < 0 || sscanf(line, "RUN %79s %d %d", token, &timeout, &argc) != 3 ||
	    argc < 1 || argc > MAX_ARGS) {
		dprintf(cs, "bad request\n%s -1000\n", SWEETD_END);
		return;
	}
	if (!token_ok(token, want)) {
		dprintf(cs, "sweetd: wrong or missing token\n%s -1000\n", SWEETD_END);
		return;
	}
	for (int i = 0; i < argc; i++) {
		if (read_line(cs, line, sizeof(line)) < 0) {
			return;
		}
		argv[i] = strdup(line);
	}
	argv[argc] = NULL;

	if (strchr(argv[0], '/') == NULL) {
		int status = builtin(cs, argc, argv);

		if (status == -2) {
			dprintf(cs, "sweetd: no built-in %s (ls cat sysctl put rm mkdir kill loader reboot); a program wants its path\n", argv[0]);
			status = -1000;
		}
		dprintf(cs, "\n%s %d\n", SWEETD_END, status);
		for (int i = 0; i < argc; i++) {
			free(argv[i]);
		}
		return;
	}

	int out[2];
	pid_t pid = -1;
	int status = -1000;
	if (pipe(out) == 0) {
		posix_spawn_file_actions_t fa;
		posix_spawn_file_actions_init(&fa);
		posix_spawn_file_actions_addopen(&fa, 0, "/dev/null", O_RDONLY, 0);
		posix_spawn_file_actions_adddup2(&fa, out[1], 1);
		posix_spawn_file_actions_adddup2(&fa, out[1], 2);
		posix_spawn_file_actions_addclose(&fa, out[0]);
		int err = posix_spawn(&pid, argv[0], &fa, NULL, argv, environ);
		posix_spawn_file_actions_destroy(&fa);
		close(out[1]);
		if (err != 0) {
			dprintf(cs, "sweetd: cannot start %s: %s\n", argv[0], strerror(err));
			pid = -1;
		}
		time_t deadline = time(NULL) + (timeout > 0 ? timeout : 3600);
		bool killed = false;
		for (;;) {
			struct pollfd p = { .fd = out[0], .events = POLLIN };
			int r = poll(&p, 1, 500);
			if (r > 0) {
				char buf[4096];
				ssize_t n = read(out[0], buf, sizeof(buf));
				if (n <= 0) {
					break;
				}
				send_all(cs, buf, (size_t)n);
			}
			if (pid > 0 && !killed && time(NULL) > deadline) {
				kill(pid, SIGKILL);
				killed = true;
				dprintf(cs, "\nsweetd: timeout after %d s, killed\n", timeout);
			}
			if (pid <= 0 && r == 0) {
				break;
			}
		}
		close(out[0]);
		if (pid > 0) {
			int st = 0;
			if (waitpid(pid, &st, 0) == pid) {
				status = WIFEXITED(st) ? WEXITSTATUS(st) : WIFSIGNALED(st) ? -WTERMSIG(st) : -1000;
			}
		}
	}
	dprintf(cs, "\n%s %d\n", SWEETD_END, status);
	for (int i = 0; i < argc; i++) {
		free(argv[i]);
	}
}

/* ---- DHCP: the PC's address on the cable ------------------------------------------------------------ */

#define PC_ADDR         0xc0a80701u     /* 192.168.7.1 */
#define PHONE_ADDR      0xc0a80702u     /* 192.168.7.2 */
#define DHCP_LEASE      86400

struct dhcp_msg {
	uint8_t op, htype, hlen, hops;
	uint32_t xid;
	uint16_t secs, flags;
	uint32_t ciaddr, yiaddr, siaddr, giaddr;
	uint8_t chaddr[16], sname[64], file[128];
	uint32_t cookie;
	uint8_t options[308];
} __attribute__((packed));

static size_t
dhcp_opt(uint8_t *p, uint8_t code, const void *data, uint8_t len)
{
	p[0] = code;
	p[1] = len;
	memcpy(p + 2, data, len);
	return 2u + len;
}

/* net/bpf.h's (the iOS SDK has no net/bpf.h) */
#define DHCP_BIOCSETIF          _IOW('B', 108, struct ifreq)
#define DHCP_BIOCSHDRCMPLT      _IOW('B', 117, u_int)

/* A BPF device bound to en2, writing whole frames (the source address ours), and en2's own address: -1 without. */
static int
dhcp_bpf(uint8_t mac[6])
{
	struct ifaddrs *list, *i;
	struct ifreq ifr;
	bool found = false;
	u_int one = 1;
	char dev[16];
	int fd = -1;

	if (getifaddrs(&list) != 0) {
		return -1;
	}
	for (i = list; i != NULL; i = i->ifa_next) {
		if (i->ifa_addr != NULL && i->ifa_addr->sa_family == AF_LINK && strcmp(i->ifa_name, "en2") == 0 &&
		    ((struct sockaddr_dl *)i->ifa_addr)->sdl_alen == 6) {
			memcpy(mac, LLADDR((struct sockaddr_dl *)i->ifa_addr), 6);
			found = true;
		}
	}
	freeifaddrs(list);
	for (int n = 0; found && n < 32 && fd < 0; n++) {
		snprintf(dev, sizeof(dev), "/dev/bpf%d", n);
		fd = open(dev, O_WRONLY);
	}
	if (fd < 0) {
		return -1;
	}
	memset(&ifr, 0, sizeof(ifr));
	strlcpy(ifr.ifr_name, "en2", sizeof(ifr.ifr_name));
	if (ioctl(fd, DHCP_BIOCSETIF, &ifr) != 0 || ioctl(fd, DHCP_BIOCSHDRCMPLT, &one) != 0) {
		close(fd);
		return -1;
	}
	return fd;
}

/* The answer as a frame: to ff:ff:ff:ff:ff:ff, IP 192.168.7.2 -> 255.255.255.255, UDP 67 -> 68 (no UDP checksum). */
static ssize_t
dhcp_bpf_send(int fd, const uint8_t mac[6], const void *payload, size_t len)
{
	uint8_t f[14 + 20 + 8 + sizeof(struct dhcp_msg)], *ip = f + 14, *udp = ip + 20;
	uint32_t sum = 0, src = htonl(PHONE_ADDR);
	uint16_t total = (uint16_t)(20 + 8 + len), ulen = (uint16_t)(8 + len);

	memset(f, 0, sizeof(f));
	memset(f, 0xff, 6);
	memcpy(f + 6, mac, 6);
	f[12] = 0x08;
	ip[0] = 0x45;
	ip[2] = (uint8_t)(total >> 8);
	ip[3] = (uint8_t)total;
	ip[8] = 64;
	ip[9] = IPPROTO_UDP;
	memcpy(ip + 12, &src, 4);
	memset(ip + 16, 0xff, 4);
	for (int k = 0; k < 20; k += 2) {
		sum += (uint32_t)(ip[k] << 8 | ip[k + 1]);
	}
	while (sum >> 16) {
		sum = (sum & 0xffff) + (sum >> 16);
	}
	ip[10] = (uint8_t)(~sum >> 8);
	ip[11] = (uint8_t)~sum;
	udp[1] = 67;
	udp[3] = 68;
	udp[4] = (uint8_t)(ulen >> 8);
	udp[5] = (uint8_t)ulen;
	memcpy(udp + 8, payload, len);
	return write(fd, f, 14 + 20 + 8 + len);
}

/* Answers DISCOVER with OFFER and REQUEST with ACK (always the same address), broadcast, as a client without an address expects. */
static void
dhcp_serve(void)
{
	int s = socket(AF_INET, SOCK_DGRAM, 0);
	int on = 1;
	struct sockaddr_in a = { .sin_len = sizeof(a), .sin_family = AF_INET, .sin_port = htons(67), .sin_addr.s_addr = htonl(INADDR_ANY) };

	if (s < 0) {
		return;
	}
	setsockopt(s, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
	setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	while (bind(s, (struct sockaddr *)&a, sizeof(a)) != 0) {
		sleep(2);
	}
	for (;;) {
		struct dhcp_msg m, r;
		struct sockaddr_in from;
		socklen_t fl = sizeof(from);
		ssize_t n = recvfrom(s, &m, sizeof(m), 0, (struct sockaddr *)&from, &fl);
		uint8_t type = 0;

		if (n < (ssize_t)offsetof(struct dhcp_msg, options) || m.op != 1 || m.hlen != 6 || m.cookie != htonl(0x63825363)) {
			continue;
		}
		for (size_t i = 0; i + 1 < sizeof(m.options) && m.options[i] != 255;) {
			if (m.options[i] == 0) {
				i++;
				continue;
			}
			if (i + 2 + m.options[i + 1] > sizeof(m.options)) {
				break;
			}
			if (m.options[i] == 53 && m.options[i + 1] == 1) {
				type = m.options[i + 2];
			}
			i += 2u + m.options[i + 1];
		}
		if (type != 1 && type != 3) {
			continue;
		}
		/* a REQUEST for somebody else's server, or for another address, is not ours (a DISCOVER is always) */
		memset(&r, 0, sizeof(r));
		r.op = 2;
		r.htype = m.htype;
		r.hlen = 6;
		r.xid = m.xid;
		r.flags = m.flags;
		r.yiaddr = htonl(PC_ADDR);
		r.siaddr = htonl(PHONE_ADDR);
		memcpy(r.chaddr, m.chaddr, 16);
		r.cookie = htonl(0x63825363);

		uint8_t *o = r.options;
		uint8_t t = type == 1 ? 2 : 5, mask[4] = { 255, 255, 255, 0 };
		uint32_t id = htonl(PHONE_ADDR), lease = htonl(DHCP_LEASE), half = htonl(DHCP_LEASE / 2), seven = htonl(DHCP_LEASE * 7 / 8);

		o += dhcp_opt(o, 53, &t, 1);
		o += dhcp_opt(o, 54, &id, 4);
		o += dhcp_opt(o, 51, &lease, 4);
		o += dhcp_opt(o, 58, &half, 4);
		o += dhcp_opt(o, 59, &seven, 4);
		o += dhcp_opt(o, 1, mask, 4);
		*o++ = 255;

		struct sockaddr_in to = { .sin_len = sizeof(to), .sin_family = AF_INET, .sin_port = htons(68),
			                  .sin_addr.s_addr = htonl(INADDR_BROADCAST) };
		size_t len = (size_t)(o - (uint8_t *)&r);
		if (len < 300) {
			len = 300;
		}
		uint8_t mac[6];
		int bpf = dhcp_bpf(mac);
		ssize_t sent;
		const char *how = "BPF on en2";

		if (bpf >= 0) {
			sent = dhcp_bpf_send(bpf, mac, &r, len);
			close(bpf);
		} else {
			/* (without BPF: works only once something routes 255.255.255.255) */
			unsigned idx = if_nametoindex("en2");

			if (idx != 0) {
				setsockopt(s, IPPROTO_IP, IP_BOUND_IF, &idx, sizeof(idx));
			}
			sent = sendto(s, &r, len, 0, (struct sockaddr *)&to, sizeof(to));
			how = "UDP (no BPF)";
		}
		if (sent < 0) {
			fprintf(stderr, "sweetd: dhcp %s -> 192.168.7.1 NOT sent (%s): %s\n", type == 1 ? "offer" : "ack", how,
			    strerror(errno));
		} else {
			fprintf(stderr, "sweetd: dhcp %s -> 192.168.7.1 (%s)\n", type == 1 ? "offer" : "ack", how);
		}
	}
}

int
main(void)
{
	signal(SIGPIPE, SIG_IGN);
	if (fork() == 0) {
		dhcp_serve();
		_exit(0);
	}
	int ls = socket(AF_INET, SOCK_STREAM, 0);
	int on = 1;
	setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	struct sockaddr_in a = { .sin_len = sizeof(a), .sin_family = AF_INET, .sin_port = htons(PORT),
		                 .sin_addr.s_addr = htonl(INADDR_ANY) };
	while (bind(ls, (struct sockaddr *)&a, sizeof(a)) != 0 || listen(ls, 4) != 0) {
		sleep(2);       /* (the network may not be up yet) */
	}
	fprintf(stderr, "sweetd: listening on port %d\n", PORT);
	for (;;) {
		struct sockaddr_in peer;
		socklen_t plen = sizeof(peer);
		int cs = accept(ls, (struct sockaddr *)&peer, &plen);
		if (cs < 0) {
			continue;
		}
		/* the USB Ethernet link only (192.168.7.0/24) */
		uint32_t pa = ntohl(peer.sin_addr.s_addr);
		if (peer.sin_family == AF_INET && (pa >> 8) == 0xc0a807) {
			serve(cs);
		}
		close(cs);
	}
}
