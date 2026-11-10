/*
 * gpu_test: the GPU kept on (boot-arg gpu=1) through HakiosGPU's msm interface, without Mesa:
 * the smallest thing a program does with it. A buffer of commands (CP_MEM_WRITE: the 3D core
 * writes a number into a second buffer), submitted again and again; each time, the fence waited
 * for and the number read back. Then the connection is closed and opened again, as a program that
 * comes and goes does. It says how many submissions came back right and how long one took.
 *
 *   gpu_test [SUBMISSIONS [ROUNDS]]    default 1000 submissions in each of 3 rounds
 *
 * The structures and numbers are Linux's include/uapi/drm/msm_drm.h, as drivers/gpu/hakios.gpu.cpp has them.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct msm_timespec {
	int64_t seconds, nanoseconds;
};
struct msm_param {
	uint32_t pipe, param;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_new {
	uint64_t size;
	uint32_t flags, handle;
};
struct msm_gem_info {
	uint32_t handle, info;
	uint64_t value;
	uint32_t len, pad;
};
struct msm_gem_submit_cmd {
	uint32_t type, submit_idx, submit_offset, size, pad, nr_relocs;
	uint64_t relocs;
};
struct msm_gem_submit_bo {
	uint32_t flags, handle;
	uint64_t presumed;
};
struct msm_gem_submit {
	uint32_t flags, fence, nr_bos, nr_cmds;
	uint64_t bos, cmds;
	int32_t fence_fd;
	uint32_t queueid;
	uint64_t in_syncobjs, out_syncobjs;
	uint32_t nr_in_syncobjs, nr_out_syncobjs, syncobj_stride, pad;
};
struct msm_wait_fence {
	uint32_t fence, flags;
	struct msm_timespec timeout;
	uint32_t queueid;
};
struct drm_gem_close {
	uint32_t handle, pad;
};

#define DRM_GEM_CLOSE           0x09
#define MSM_GET_PARAM           0x40
#define MSM_GEM_NEW             0x42
#define MSM_GEM_INFO            0x43
#define MSM_GEM_SUBMIT          0x46
#define MSM_WAIT_FENCE          0x47
#define MSM_INFO_GET_IOVA       1
#define MSM_BO_WC               0x00020000
#define MSM_SUBMIT_BO_READ      1
#define MSM_SUBMIT_BO_WRITE     2
#define MSM_SUBMIT_CMD_BUF      1
#define CP_MEM_WRITE            0x3d

static io_connect_t connection;

static kern_return_t
call(unsigned number, void *data, size_t size)
{
	size_t returned = size;

	return IOConnectCallStructMethod(connection, number, data, size, data, &returned);
}

static uint32_t
odd_parity(uint32_t v)
{
	return 0x9669 >> (0xf & (v ^ v >> 4 ^ v >> 8 ^ v >> 12 ^ v >> 16 ^ v >> 20 ^ v >> 24 ^ v >> 28)) & 1;
}

static uint32_t
command(uint32_t opcode, uint32_t words)
{
	return 7u << 28 | words | odd_parity(words) << 15 | opcode << 16 | odd_parity(opcode) << 23;
}

/* A buffer of 4 KB: its handle, its address for the GPU, and where it is mapped here. */
static int
buffer(uint32_t *handle, uint64_t *gpu, volatile uint32_t **here)
{
	struct msm_gem_new made = { .size = 4096, .flags = MSM_BO_WC };
	struct msm_gem_info info = { .info = MSM_INFO_GET_IOVA };
	mach_vm_address_t address = 0;
	mach_vm_size_t size = 0;
	kern_return_t result;

	if ((result = call(MSM_GEM_NEW, &made, sizeof(made))) != 0) {
		printf("gpu_test: GEM_NEW: %#x\n", result);
		return -1;
	}
	info.handle = made.handle;
	if ((result = call(MSM_GEM_INFO, &info, sizeof(info))) != 0) {
		printf("gpu_test: GEM_INFO (address): %#x\n", result);
		return -1;
	}
	if ((result = IOConnectMapMemory64(connection, made.handle, mach_task_self(), &address, &size, kIOMapAnywhere)) != 0) {
		printf("gpu_test: mapping buffer %u: %#x\n", made.handle, result);
		return -1;
	}
	*handle = made.handle;
	*gpu = info.value;
	*here = (volatile uint32_t *)(uintptr_t)address;
	return 0;
}

static int
round_of(unsigned round, unsigned submissions)
{
	static const uint32_t asked[] = { 0x03, 0x02, 0x10, 0x04 };
	static const char *const names[] = { "chip id", "gmem", "highest bank bit", "max freq" };
	io_service_t service = IOServiceGetMatchingService(0, IOServiceMatching("HakiosGPU"));
	uint32_t commands_handle, data_handle, wrong = 0, failed = 0, last_fence = 0;
	uint64_t commands_gpu, data_gpu, began, ended;
	volatile uint32_t *commands, *data;
	mach_timebase_info_data_t base;
	struct drm_gem_close close_it = { 0 };

	if (service == 0 || IOServiceOpen(service, mach_task_self(), 0, &connection) != 0) {
		printf("gpu_test: round %u: no HakiosGPU to open (is the kernel's gpu=1 on, and did the GPU start?)\n", round);
		return -1;
	}
	IOObjectRelease(service);
	if (round == 1) {
		for (unsigned i = 0; i < sizeof(asked) / sizeof(asked[0]); i++) {
			struct msm_param p = { .pipe = 1, .param = asked[i] };
			kern_return_t result = call(MSM_GET_PARAM, &p, sizeof(p));

			printf("gpu_test: %s: %#llx%s\n", names[i], (unsigned long long)p.value, result ? " (refused)" : "");
		}
	}
	if (buffer(&commands_handle, &commands_gpu, &commands) || buffer(&data_handle, &data_gpu, &data)) {
		IOServiceClose(connection);
		return -1;
	}
	printf("gpu_test: round %u: commands at %#llx, the word at %#llx\n", round, (unsigned long long)commands_gpu,
	    (unsigned long long)data_gpu);
	mach_timebase_info(&base);
	began = mach_absolute_time();
	for (unsigned i = 1; i <= submissions; i++) {
		uint32_t value = round << 24 | i;
		struct msm_gem_submit_bo bos[2] = {
			{ MSM_SUBMIT_BO_READ, commands_handle, commands_gpu },
			{ MSM_SUBMIT_BO_READ | MSM_SUBMIT_BO_WRITE, data_handle, data_gpu },
		};
		struct msm_gem_submit_cmd cmd = { .type = MSM_SUBMIT_CMD_BUF, .submit_idx = 0, .submit_offset = 0, .size = 16 };
		struct msm_gem_submit submit = { .nr_bos = 2, .nr_cmds = 1, .bos = (uint64_t)(uintptr_t)bos,
			.cmds = (uint64_t)(uintptr_t)&cmd, .fence_fd = -1 };
		struct msm_wait_fence wait = { 0 };
		kern_return_t result;

		commands[0] = command(CP_MEM_WRITE, 3);
		commands[1] = (uint32_t)data_gpu;
		commands[2] = (uint32_t)(data_gpu >> 32);
		commands[3] = value;
		__asm__ volatile ("dsb sy" ::: "memory");
		if ((result = call(MSM_GEM_SUBMIT, &submit, sizeof(submit))) != 0 || submit.fence == 0) {
			printf("gpu_test: round %u: submission %u not taken: %#x\n", round, i, result);
			failed++;
			break;
		}
		wait.fence = submit.fence;
		wait.timeout.seconds = INT64_MAX / 2;   /* "no limit": the kernel's own patience, 5 s */
		if ((result = call(MSM_WAIT_FENCE, &wait, sizeof(wait))) != 0) {
			printf("gpu_test: round %u: submission %u (fence %u) did not come back: %#x\n", round, i, submit.fence, result);
			failed++;
			break;
		}
		if (data[0] != value) {
			if (wrong++ < 5) {
				printf("gpu_test: round %u: submission %u: the word is %#x, not %#x\n", round, i, data[0], value);
			}
		}
		last_fence = submit.fence;
	}
	ended = mach_absolute_time();
	printf("gpu_test: round %u: %u submissions, %u wrong, %u failed, last fence %u; %.1f us each\n", round, submissions,
	    wrong, failed, last_fence, (double)(ended - began) * base.numer / base.denom / 1000.0 / submissions);
	close_it.handle = commands_handle;
	call(DRM_GEM_CLOSE, &close_it, sizeof(close_it));
	close_it.handle = data_handle;
	call(DRM_GEM_CLOSE, &close_it, sizeof(close_it));
	IOServiceClose(connection);
	return wrong || failed ? -1 : 0;
}

int
main(int argc, char **argv)
{
	unsigned submissions = argc > 1 ? (unsigned)atoi(argv[1]) : 1000, rounds = argc > 2 ? (unsigned)atoi(argv[2]) : 3;
	int bad = 0;

	setvbuf(stdout, NULL, _IONBF, 0);
	for (unsigned round = 1; round <= rounds; round++) {
		bad |= round_of(round, submissions);
	}
	printf("gpu_test: %s\n", bad ? "FAILED" : "every submission came back with its word: THE GPU TAKES PROGRAMS' COMMANDS");
	return bad ? 1 : 0;
}
