/*
 * arp_test: what the network sees of an IPv4 address on an interface (diagnosis of en0 not being found by ARP).
 *
 *   arp_test alias IF ADDR       ADDR added to IF (a /32 alias, so no route changes)
 *   arp_test unalias IF ADDR     ... and taken away
 *   arp_test announce IF ADDR    one gratuitous ARP for ADDR from IF's address, through BPF (as IPConfiguration's
 *                                arp_session does after DHCP): sender = target = ADDR, to everybody
 *   arp_test stats               the kernel's ARP counts (struct arpstat, net.link.ether.inet.stats)
 *   arp_test watch IF SECONDS    every ARP frame on IF, both ways (BPF), and ICMP: who asks, who answers, to whom
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>

/* (net/bpf.h is not in the iOS SDK: xnu's bsd/net/bpf.h) */
#define BIOCSETIF       _IOW('B', 108, struct ifreq)
#define BIOCSHDRCMPLT   _IOW('B', 117, unsigned int)
#define BIOCGBLEN       _IOR('B', 102, unsigned int)
#define BIOCIMMEDIATE   _IOW('B', 112, unsigned int)
#define BIOCSSEESENT    _IOW('B', 119, unsigned int)
struct bpf_hdr32 { int32_t sec, usec; uint32_t caplen, datalen; uint16_t hdrlen; };
#ifndef SIOCAIFADDR
#define SIOCAIFADDR _IOW('i', 26, struct ifaliasreq)
#endif

static int
address(const char *text, struct sockaddr_in *sin)
{
	memset(sin, 0, sizeof(*sin));
	sin->sin_len = sizeof(*sin);
	sin->sin_family = AF_INET;
	return inet_pton(AF_INET, text, &sin->sin_addr) == 1 ? 0 : -1;
}

static int
alias(const char *ifname, const char *text, int add)
{
	struct ifaliasreq request;
	struct ifreq remove;
	int s = socket(AF_INET, SOCK_DGRAM, 0), r;

	if (s < 0) { perror("arp_test: socket"); return 1; }
	if (add) {
		memset(&request, 0, sizeof(request));
		strlcpy(request.ifra_name, ifname, sizeof(request.ifra_name));
		if (address(text, (struct sockaddr_in *)&request.ifra_addr)) { fprintf(stderr, "arp_test: bad address\n"); return 1; }
		struct sockaddr_in *mask = (struct sockaddr_in *)&request.ifra_mask;
		mask->sin_len = sizeof(*mask);
		mask->sin_family = AF_INET;
		mask->sin_addr.s_addr = 0xffffffffu;
		r = ioctl(s, SIOCAIFADDR, &request);
	} else {
		memset(&remove, 0, sizeof(remove));
		strlcpy(remove.ifr_name, ifname, sizeof(remove.ifr_name));
		if (address(text, (struct sockaddr_in *)&remove.ifr_addr)) { fprintf(stderr, "arp_test: bad address\n"); return 1; }
		r = ioctl(s, SIOCDIFADDR, &remove);
	}
	printf("arp_test: %s %s %s: %s\n", add ? "alias" : "unalias", ifname, text, r == 0 ? "done" : strerror(errno));
	close(s);
	return r != 0;
}

static int
lladdr(const char *ifname, uint8_t mac[6])
{
	struct ifaddrs *list, *a;
	int found = -1;

	if (getifaddrs(&list) != 0) return -1;
	for (a = list; a != NULL; a = a->ifa_next) {
		if (a->ifa_addr != NULL && a->ifa_addr->sa_family == AF_LINK && strcmp(a->ifa_name, ifname) == 0) {
			struct sockaddr_dl *dl = (struct sockaddr_dl *)a->ifa_addr;
			if (dl->sdl_alen == 6) { memcpy(mac, LLADDR(dl), 6); found = 0; }
		}
	}
	freeifaddrs(list);
	return found;
}

static int
announce(const char *ifname, const char *text)
{
	uint8_t frame[42], mac[6];
	struct sockaddr_in ip;
	struct ifreq request;
	char path[16];
	int fd = -1, one = 1;

	if (address(text, &ip) || lladdr(ifname, mac)) { fprintf(stderr, "arp_test: no address, or no link address on %s\n", ifname); return 1; }
	for (int i = 0; i < 64 && fd < 0; i++) {
		snprintf(path, sizeof(path), "/dev/bpf%d", i);
		fd = open(path, O_RDWR);
	}
	if (fd < 0) { perror("arp_test: /dev/bpf"); return 1; }
	memset(&request, 0, sizeof(request));
	strlcpy(request.ifr_name, ifname, sizeof(request.ifr_name));
	if (ioctl(fd, BIOCSETIF, &request) != 0 || ioctl(fd, BIOCSHDRCMPLT, &one) != 0) { perror("arp_test: bpf"); close(fd); return 1; }
	memset(frame, 0xff, 6);			/* to everybody */
	memcpy(frame + 6, mac, 6);
	frame[12] = 0x08; frame[13] = 0x06;	/* ARP */
	frame[14] = 0; frame[15] = 1;		/* Ethernet */
	frame[16] = 0x08; frame[17] = 0;	/* IPv4 */
	frame[18] = 6; frame[19] = 4;
	frame[20] = 0; frame[21] = 1;		/* request (a gratuitous one: sender = target) */
	memcpy(frame + 22, mac, 6);
	memcpy(frame + 28, &ip.sin_addr, 4);
	memset(frame + 32, 0, 6);
	memcpy(frame + 38, &ip.sin_addr, 4);
	ssize_t n = write(fd, frame, sizeof(frame));
	printf("arp_test: announce %s on %s (%02x:%02x:%02x:%02x:%02x:%02x): %s\n", text, ifname, mac[0], mac[1], mac[2], mac[3],
	    mac[4], mac[5], n == (ssize_t)sizeof(frame) ? "sent" : strerror(errno));
	close(fd);
	return n != (ssize_t)sizeof(frame);
}

static int
stats(void)
{
	uint32_t v[16] = { 0 };
	size_t size = sizeof(v);

	if (sysctlbyname("net.link.ether.inet.stats", v, &size, NULL, 0) != 0) { perror("arp_test: stats"); return 1; }
	printf("arp_test: sent %u requests %u replies %u announces; received %u requests %u replies (%u ARP in all); "
	    "dupips %u\n", v[0], v[1], v[2], v[3], v[4], v[5], v[12]);
	return 0;
}

static void
mac_text(char *out, const uint8_t *m)
{
	snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static int
watch(const char *ifname, int seconds)
{
	struct ifreq request;
	unsigned length = 0, one = 1;
	char path[16];
	int fd = -1;
	uint8_t mine[6];

	if (lladdr(ifname, mine)) { fprintf(stderr, "arp_test: no link address on %s\n", ifname); return 1; }
	for (int i = 0; i < 64 && fd < 0; i++) {
		snprintf(path, sizeof(path), "/dev/bpf%d", i);
		fd = open(path, O_RDWR);
	}
	if (fd < 0) { perror("arp_test: /dev/bpf"); return 1; }
	memset(&request, 0, sizeof(request));
	strlcpy(request.ifr_name, ifname, sizeof(request.ifr_name));
	if (ioctl(fd, BIOCSETIF, &request) != 0 || ioctl(fd, BIOCIMMEDIATE, &one) != 0 || ioctl(fd, BIOCSSEESENT, &one) != 0 ||
	    ioctl(fd, BIOCGBLEN, &length) != 0) { perror("arp_test: bpf"); close(fd); return 1; }
	uint8_t *buffer = malloc(length);
	struct timeval start, now;
	gettimeofday(&start, NULL);
	printf("arp_test: watching %s (ours %02x:%02x:%02x:%02x:%02x:%02x) for %d s\n", ifname, mine[0], mine[1], mine[2], mine[3],
	    mine[4], mine[5], seconds);
	fflush(stdout);
	for (;;) {
		gettimeofday(&now, NULL);
		long left_ms = seconds * 1000L - ((now.tv_sec - start.tv_sec) * 1000L + (now.tv_usec - start.tv_usec) / 1000);
		if (left_ms <= 0) break;
		fd_set set;
		FD_ZERO(&set);
		FD_SET(fd, &set);
		struct timeval wait = { left_ms / 1000, (int)(left_ms % 1000) * 1000 };
		if (select(fd + 1, &set, NULL, NULL, &wait) <= 0) continue;
		ssize_t n = read(fd, buffer, length);
		for (ssize_t at = 0; n > 0 && at + (ssize_t)sizeof(struct bpf_hdr32) <= n;) {
			struct bpf_hdr32 *h = (struct bpf_hdr32 *)(buffer + at);
			const uint8_t *f = buffer + at + h->hdrlen;
			unsigned c = h->caplen;
			char src[18], dst[18], smac[18], tmac[18];
			double t = (h->sec - start.tv_sec) + (h->usec - start.tv_usec) / 1e6;
			if (c >= 14) {
				const char *way = memcmp(f + 6, mine, 6) == 0 ? "OUT" : "in ";
				unsigned type = (unsigned)f[12] << 8 | f[13];
				mac_text(dst, f);
				mac_text(src, f + 6);
				if (type == 0x0806 && c >= 42) {
					mac_text(smac, f + 22);
					mac_text(tmac, f + 32);
					printf("%7.3f %s %s > %s ARP %s: sender %u.%u.%u.%u %s, target %u.%u.%u.%u %s\n", t, way, src, dst,
					    f[21] == 1 ? "request" : f[21] == 2 ? "REPLY" : "op?", f[28], f[29], f[30], f[31], smac, f[38], f[39],
					    f[40], f[41], tmac);
				} else if (type == 0x0800 && c >= 34 && f[23] == 1) {
					unsigned ihl = (f[14] & 15) * 4u;
					printf("%7.3f %s %s > %s ICMP type %u %u.%u.%u.%u > %u.%u.%u.%u\n", t, way, src, dst,
					    c > 14 + ihl ? f[14 + ihl] : 255, f[26], f[27], f[28], f[29], f[30], f[31], f[32], f[33]);
				} else if (f[0] & 1) {	/* any other frame to a group: who sent what */
					printf("%7.3f %s %s > %s type %#06x", t, way, src, dst, type);
					if (type == 0x0800 && c >= 34)
						printf(" IPv4 proto %u %u.%u.%u.%u > %u.%u.%u.%u", f[23], f[26], f[27], f[28], f[29], f[30], f[31],
						    f[32], f[33]);
					printf("\n");
				}
			}
			fflush(stdout);
			at += (h->hdrlen + h->caplen + 3) & ~3u;
		}
	}
	close(fd);
	return 0;
}

int
main(int argc, char **argv)
{
	if (argc == 4 && strcmp(argv[1], "watch") == 0) return watch(argv[2], atoi(argv[3]));
	if (argc == 2 && strcmp(argv[1], "stats") == 0) return stats();
	if (argc == 4 && strcmp(argv[1], "alias") == 0) return alias(argv[2], argv[3], 1);
	if (argc == 4 && strcmp(argv[1], "unalias") == 0) return alias(argv[2], argv[3], 0);
	if (argc == 4 && strcmp(argv[1], "announce") == 0) return announce(argv[2], argv[3]);
	fprintf(stderr, "usage: arp_test stats | alias IF ADDR | unalias IF ADDR | announce IF ADDR | watch IF SECONDS\n");
	return 2;
}
