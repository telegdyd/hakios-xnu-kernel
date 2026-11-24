/*
 * kstacks: xnu's stackshot of the kernel task (every kernel thread's stack, its wait event and what it waits on), as the
 * raw kcdata buffer into a file, for tools/kstacks.py to read with the kernel's symbols. For a hang: which thread holds,
 * which waits, where.
 *
 *   kstacks FILE
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>

/* bsd/sys/stackshot.h (not in the SDK) */
typedef struct {
	int             sc_pid;
	uint64_t        sc_flags;
	uint64_t        sc_delta_timestamp;
	uint64_t        sc_buffer;
	uint32_t        sc_size;
	uint64_t        sc_out_buffer_addr;
	uint64_t        sc_out_size_addr;
	uint64_t        sc_pagetable_mask;
} stackshot_config_t;
#define STACKSHOT_CONFIG_TYPE           1
#define STACKSHOT_KCDATA_FORMAT         0x10000ull
#define STACKSHOT_NO_IO_STATS           0x800000ull
#define STACKSHOT_THREAD_WAITINFO       0x1000000ull
#define STACKSHOT_DISABLE_LATENCY_INFO  0x40000000ull
#define STACKSHOT_SAVE_KEXT_LOADINFO    0x08ull
#define SYS_stack_snapshot_with_config  491

int
main(int argc, char **argv)
{
	stackshot_config_t config;
	uint64_t buffer = 0;
	uint32_t size = 0;

	if (argc != 2 && argc != 3) {
		fprintf(stderr, "usage: kstacks FILE [PID]   (PID: 0 the kernel task (default), -1 every task)\n");
		return 2;
	}
	memset(&config, 0, sizeof(config));
	config.sc_pid = argc == 3 ? atoi(argv[2]) : 0;
	config.sc_flags = STACKSHOT_KCDATA_FORMAT | STACKSHOT_THREAD_WAITINFO | STACKSHOT_NO_IO_STATS |
	    STACKSHOT_DISABLE_LATENCY_INFO | STACKSHOT_SAVE_KEXT_LOADINFO;
	config.sc_out_buffer_addr = (uint64_t)(uintptr_t)&buffer;
	config.sc_out_size_addr = (uint64_t)(uintptr_t)&size;
	if (syscall(SYS_stack_snapshot_with_config, STACKSHOT_CONFIG_TYPE, &config, sizeof(config)) != 0) {
		fprintf(stderr, "kstacks: stackshot: %s\n", strerror(errno));
		return 1;
	}
	int fd = open(argv[1], O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (fd < 0 || write(fd, (const void *)(uintptr_t)buffer, size) != (ssize_t)size) {
		fprintf(stderr, "kstacks: %s: %s\n", argv[1], strerror(errno));
		return 1;
	}
	close(fd);
	printf("kstacks: %u bytes in %s\n", size, argv[1]);
	return 0;
}
