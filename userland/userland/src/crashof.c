/*
 * crashof: where a program that dies at once crashes, when nothing writes a crash report (ReportCrash wrote none
 * for airplayd on Darwin 27).
 *
 *   usage: crashof [-u UID] PROGRAM [ARGS...]
 *
 * Takes the bad-access, bad-instruction, arithmetic, breakpoint, guard and crash exceptions of its own task, forks
 * and executes PROGRAM (as UID with -u) in the child, which keeps those exception ports. When the child faults, its
 * registers come here: printed, with its frame-pointer chain (40 frames, read through the task port the exception
 * carries, which a child that changed its user does not give), then the exception is refused so that the child dies
 * as it would have. Addresses in the dyld shared cache are printed unslid ("cache:" + address, for
 * device-info/darwin27-runs/dscsym.py), others as they are. Found airplayd's NULL in
 * AirPlayReceiverServerGetDispatchQueue (UI-PERF-PLAN.md Task 10).
 * Not done here: hardware breakpoints set with task_set_state(ARM_DEBUG_STATE64) on a child spawned suspended
 * reset the phone (2026-11-28).
 * Entitlements: stackof.entitlements.
 */
#include <mach/mach.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#define CACHE_BASE      0x180000000ull
#define FRAMES          40

extern const void *_dyld_get_shared_cache_range(size_t *length);
extern kern_return_t mach_vm_read_overwrite(vm_map_t, mach_vm_address_t, mach_vm_size_t, mach_vm_address_t,
    mach_vm_size_t *);

static uint64_t slide, cache_end;

static void
say_address(uint64_t a)
{
	a &= 0x0000007fffffffffull;     /* (the PAC bits off) */
	if (a >= CACHE_BASE + slide && a < cache_end) {
		printf(" cache:%llx", a - slide);
	} else {
		printf(" %#llx", a);
	}
}

#pragma pack(push, 4)
typedef struct {
	mach_msg_header_t head;
	mach_msg_body_t body;
	mach_msg_port_descriptor_t thread;
	mach_msg_port_descriptor_t task;
	NDR_record_t ndr;
	exception_type_t exception;
	mach_msg_type_number_t code_count;
	int64_t code[2];
	int flavor;
	mach_msg_type_number_t state_count;
	natural_t state[1024];
	mach_msg_trailer_t trailer;
} request_t;

typedef struct {
	mach_msg_header_t head;
	NDR_record_t ndr;
	kern_return_t result;
	int flavor;
	mach_msg_type_number_t state_count;
} reply_t;
#pragma pack(pop)

int
main(int argc, char **argv)
{
	const exception_mask_t mask = EXC_MASK_BAD_ACCESS | EXC_MASK_BAD_INSTRUCTION | EXC_MASK_ARITHMETIC | EXC_MASK_CRASH |
	    EXC_MASK_BREAKPOINT | EXC_MASK_GUARD;
	mach_port_t port;
	int first = 1, uid = -1, status = 0;
	size_t cache_length = 0;
	pid_t child;
	request_t request;

	setvbuf(stdout, NULL, _IOLBF, 0);
	if (argc > 3 && strcmp(argv[1], "-u") == 0) {
		uid = atoi(argv[2]);
		first = 3;
	}
	if (first >= argc) {
		fprintf(stderr, "usage: crashof [-u UID] PROGRAM [ARGS...]\n");
		return 2;
	}
	slide = (uint64_t)(uintptr_t)_dyld_get_shared_cache_range(&cache_length) - CACHE_BASE;
	cache_end = CACHE_BASE + slide + cache_length;
	if (mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &port) != KERN_SUCCESS ||
	    mach_port_insert_right(mach_task_self(), port, port, MACH_MSG_TYPE_MAKE_SEND) != KERN_SUCCESS ||
	    task_set_exception_ports(mach_task_self(), mask, port,
	    (exception_behavior_t)(EXCEPTION_STATE_IDENTITY | MACH_EXCEPTION_CODES), ARM_THREAD_STATE64) != KERN_SUCCESS) {
		printf("crashof: no exception port\n");
		return 1;
	}
	if ((child = fork()) == 0) {
		if (uid >= 0 && (setgid((gid_t)uid) != 0 || setuid((uid_t)uid) != 0)) {
			perror("crashof: setuid");
			_exit(127);
		}
		execv(argv[first], argv + first);
		perror("crashof: exec");
		_exit(127);
	}
	/* (this task's own faults no longer wanted: only the child's) */
	task_set_exception_ports(mach_task_self(), mask, MACH_PORT_NULL, EXCEPTION_DEFAULT, THREAD_STATE_NONE);
	printf("crashof: %s is pid %d (cache slide %#llx)\n", argv[first], child, slide);
	for (;;) {
		memset(&request, 0, sizeof(request));
		kern_return_t kr = mach_msg(&request.head, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(request), port, 500,
		    MACH_PORT_NULL);

		if (kr == MACH_RCV_TIMED_OUT) {
			if (waitpid(child, &status, WNOHANG) == child) {
				break;
			}
			continue;
		}
		if (kr != KERN_SUCCESS) {
			printf("crashof: mach_msg %#x\n", kr);
			break;
		}
		const arm_thread_state64_t *st = (const arm_thread_state64_t *)(void *)request.state;
		const task_t task = request.task.name;

		printf("crashof: exception %d codes %#llx %#llx\n", request.exception, (unsigned long long)request.code[0],
		    (unsigned long long)request.code[1]);
		printf("crashof: pc"); say_address(arm_thread_state64_get_pc(*st));
		printf(" lr"); say_address(arm_thread_state64_get_lr(*st));
		printf(" sp %#llx fp %#llx\n", (unsigned long long)arm_thread_state64_get_sp(*st),
		    (unsigned long long)arm_thread_state64_get_fp(*st));
		for (int r = 0; r < 29; r++) {
			printf("crashof: x%d", r);
			say_address(st->__x[r]);
			printf("\n");
		}
		printf("crashof: frames:");
		uint64_t fp = arm_thread_state64_get_fp(*st);
		for (int f = 0; f < FRAMES && fp != 0; f++) {
			uint64_t frame[2];
			mach_vm_size_t got = 0;

			if (mach_vm_read_overwrite(task, fp, sizeof(frame), (mach_vm_address_t)(uintptr_t)frame, &got) !=
			    KERN_SUCCESS || frame[1] == 0) {
				break;
			}
			say_address(frame[1]);
			if (frame[0] <= fp) {
				break;
			}
			fp = frame[0];
		}
		printf("\n");
		/* refused: the child dies of it as it would have */
		reply_t reply;
		memset(&reply, 0, sizeof(reply));
		reply.head.msgh_bits = MACH_MSGH_BITS(MACH_MSGH_BITS_REMOTE(request.head.msgh_bits), 0);
		reply.head.msgh_remote_port = request.head.msgh_remote_port;
		reply.head.msgh_size = (mach_msg_size_t)sizeof(reply);
		reply.head.msgh_id = request.head.msgh_id + 100;
		reply.ndr = NDR_record;
		reply.result = KERN_FAILURE;
		mach_msg(&reply.head, MACH_SEND_MSG, reply.head.msgh_size, 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE,
		    MACH_PORT_NULL);
		mach_port_deallocate(mach_task_self(), request.thread.name);
		mach_port_deallocate(mach_task_self(), task);
	}
	if (WIFSIGNALED(status)) {
		printf("crashof: pid %d killed by signal %d\n", child, WTERMSIG(status));
	} else {
		printf("crashof: pid %d exited %d\n", child, WEXITSTATUS(status));
	}
	return 0;
}
