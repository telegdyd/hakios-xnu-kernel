/* For the host tests: XNU's deadlines, as CLOCK_REALTIME nanoseconds (what kern/locks.h's timed sleep takes). */
#ifndef QRTR_TEST_SHIM_CLOCK_H
#define QRTR_TEST_SHIM_CLOCK_H

#include <stdint.h>
#include <time.h>

#define NSEC_PER_MSEC 1000000ull

static inline void
clock_interval_to_deadline(uint32_t interval, uint64_t scale, uint64_t *deadline)
{
	struct timespec now;

	clock_gettime(CLOCK_REALTIME, &now);
	*deadline = (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec + interval * scale;
}

#endif
