/*
 * /sbin/launchd of the small image, when the kernel starts from it (rd=md0 is the small image, no
 * system image in memory): the first program the kernel runs, for a root read from the phone's
 * storage as it is needed.
 *
 * The Darwin system volume is on the phone's storage, read block by block as it is needed (the
 * kernel's /dev/ufs_NAME, bsd/dev/board/ufs.c): only what is read comes off the storage, so the
 * system can be bigger than what fits in the kernel's memory window. Its place is the port's own
 * partition, darwin (/dev/ufs_darwin: the system image from block 0, tools/darwin-write.py); the
 * Android DSU image in userdata (/dev/ufs_dsu, the loader's partition "dsu") is where it was before
 * that, used only while darwin does not hold the Darwin this image was made for (still 16.5's root,
 * say). This program mounts the volume read only on /flash, makes it the root -- this image goes to
 * /hakios, where the system volume has the directory stage0 mounts it on otherwise (init/stage0.c)
 * -- and runs init from there, as stage0 does: from then on all is as in a boot from the system
 * image in memory.
 */
#include "mini.h"

#define VERSION_PLIST "/flash/System/Library/CoreServices/SystemVersion.plist"
#define WANT_VERSION  "27."    /* the Darwin the small image's libraries and jobs are built for */

/* The mounted volume's ProductVersion into ver ("" if there is none). */
static void
product_version(char *ver, size_t size)
{
	static const char key[] = "<key>ProductVersion</key>";
	static char buf[4096];
	int fd = open(VERSION_PLIST, O_RDONLY, 0);
	ssize_t n = fd < 0 ? -1 : read(fd, buf, sizeof(buf) - 1);
	char *p;
	size_t i = 0;

	ver[0] = '\0';
	if (fd >= 0) {
		close(fd);
	}
	if (n <= 0) {
		return;
	}
	buf[n] = '\0';
	for (p = buf; *p != '\0' && strncmp(p, key, sizeof(key) - 1) != 0; p++) {
	}
	if (*p == '\0') {
		return;
	}
	for (p += sizeof(key) - 1; *p == ' ' || *p == '\t' || *p == '\r' || *p == '\n'; p++) {
	}
	if (strncmp(p, "<string>", 8) != 0) {
		return;
	}
	for (p += 8; *p != '\0' && *p != '<' && i + 1 < size; p++) {
		ver[i++] = *p;
	}
	ver[i] = '\0';
}

/* dev on /flash, a few tries in case its node is not there yet; 0 when it is mounted. */
static int
mount_system(const char *dev, int tries)
{
	while (tries-- > 0) {
		if (mount("apfs", "/flash", MNT_RDONLY, (void *)dev) == 0) {
			return 0;
		}
		usleep(100000);
	}
	return -1;
}

int
main(int argc, char **argv, char **envp)
{
	char *init_argv[] = { "/hakios/sbin/init", NULL };
	static const char switch_to[] = "/flash hakios";
	int fd = open("/dev/console", O_RDWR, 0);
	const char *dev = "/dev/ufs_darwin";
	char ver[32];

	(void)argc, (void)argv;
	dup2(fd, 0);
	dup2(fd, 1);
	dup2(fd, 2);
	if (fd > 2) {
		close(fd);
	}
	printf("\n=== stage0dsu: the system volume from the phone's storage ===\n");
	/* (the nodes are made before the first process runs; a few tries in case they are not there yet) */
	if (mount_system(dev, 50) != 0) {
		printf("stage0dsu: cannot mount %s on /flash: %s\n", dev, strerror(errno));
		dev = NULL;
	} else {
		product_version(ver, sizeof(ver));
		if (strncmp(ver, WANT_VERSION, sizeof(WANT_VERSION) - 1) != 0) {
			printf("stage0dsu: %s holds %s, not Darwin %s*: not that one\n", dev, ver[0] ? ver : "no SystemVersion",
			    WANT_VERSION);
			if (unmount("/flash", 0) != 0) {
				printf("stage0dsu: unmounting %s: %s\n", dev, strerror(errno));
				reboot(0x08 /* RB_HALT */);
				return 1;
			}
			dev = NULL;
		}
	}
	if (dev == NULL) {
		dev = "/dev/ufs_dsu";
		if (mount_system(dev, 20) != 0) {
			printf("stage0dsu: cannot mount %s on /flash: %s\n", dev, strerror(errno));
			reboot(0x08 /* RB_HALT */);
			return 1;
		}
		product_version(ver, sizeof(ver));
	}
	printf("stage0dsu: %s on /flash (iPhone OS %s)\n", dev, ver[0] ? ver : "?");
	if (sysctlbyname("kern.hakios_switch_root", NULL, NULL, switch_to, sizeof(switch_to) - 1) != 0) {
		printf("stage0dsu: switching the root: %s\n", strerror(errno));
	} else {
		chdir("/");
		printf("stage0dsu: the system volume is the root, this image on /hakios\n");
		execve(init_argv[0], init_argv, envp);
		printf("stage0dsu: cannot run %s: %s\n", init_argv[0], strerror(errno));
	}
	reboot(0x08 /* RB_HALT */);
	return 1;
}
