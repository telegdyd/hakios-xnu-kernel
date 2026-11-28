/*
 * How fast the GPU is asked to run (board feature gpu-dcvs; board/adreno.c): the pure part, shared with the host
 * test (workbench/adreno_dcvs_test.c). Speeds are counted from 1 (the slowest of the board's gpu { } speeds-khz) to
 * `top` (the fastest); times are of a monotonic clock, in nanoseconds; temperatures in thousandths of a degree.
 *
 * From the OnePlus 10 Pro's port (negroni-dev's hakios.dcvs.h, measured there with 220, 317 and 492 MHz: idle
 * 0.7 % busy, a command processor kept at work 98.5 %), with the speeds of a board instead of three.
 */
#ifndef _HAKIOS_ADRENO_DCVS_H
#define _HAKIOS_ADRENO_DCVS_H

#include <stdbool.h>
#include <stdint.h>

#define ADRENO_DCVS_IDLE_NS     200000000ull    /* this long without load: one speed down; a submission after this
                                                 * long without one: a new burst, at the fastest allowed at once */

/*
 * The fastest speed the GPU's own two sensors allow: from 75 degrees on (the hotter of the two) one below the top,
 * from 85 the slowest; back up only 5 degrees below where it went down. A sensor without a reading: the slowest,
 * as for the hottest (missing is not 0 degrees). `valid`: bit 0 and 1, which of the two have a reading.
 */
static inline unsigned
adreno_thermal_cap(unsigned previous, unsigned top, unsigned valid, int32_t t0, int32_t t1)
{
	int32_t hottest = t0 > t1 ? t0 : t1;
	unsigned warm = top > 1 ? top - 1 : 1;

	if (valid != 3 || hottest >= 85000 || (previous <= 1 && hottest >= 80000)) {
		return 1;
	}
	if (hottest >= 75000 || (previous < top && hottest >= 70000)) {
		return warm;
	}
	return top;
}

/*
 * The fastest speed the SoC's thermal limiter allows (board/thermal.c, hakios_thermal_step(): `step` of `most`, how far
 * it holds the processors back for the hottest of all the SoC's sensors): the top while it holds nothing back, the
 * slowest when it holds back all it can, and in between as far down the GPU's speeds as it is down its own rows, to
 * the nearest speed: a row or two of the processors' many costs the GPU nothing (the phone sits between thermal's
 * clear and trip most of a run, held back a row or so, and the GPU at one speed less lost its fastest for all of
 * it). A phone hot from its processors draws less with its GPU too.
 */
static inline unsigned
adreno_soc_cap(unsigned top, uint32_t step, uint32_t most)
{
	uint32_t down;

	if (top <= 1 || most == 0 || step == 0) {
		return top < 1 ? 1 : top;
	}
	if (step >= most) {
		return 1;
	}
	down = (step * (top - 1) + most / 2) / most;
	return down >= top ? 1 : top - down;
}

/*
 * The speed wanted now, from `level`, the speed it runs at. With the GMU's busy counter (`busy_valid`): from 40 %
 * busy the fastest (kept while at least 25 % so an active burst does not oscillate between the top two speeds),
 * from 20 % one below it; without it, work still queued for the fastest and a submission in the last 200 ms for
 * one below. Otherwise one speed down every 200 ms. Never above `cap`. `last_load`: when load was last seen,
 * kept between calls.
 */
static inline unsigned
adreno_dcvs_target(unsigned level, unsigned top, unsigned cap, bool busy_valid, unsigned busy_pct, bool pending,
    bool recent, uint64_t now, uint64_t *last_load)
{
	unsigned want = level, middle = top > 1 ? top - 1 : 1;
	bool high = busy_valid ? (busy_pct >= 40 || (level >= top && busy_pct >= 25)) : pending;
	bool some = busy_valid ? busy_pct >= 20 : recent;

	if (high) {
		want = top;
		*last_load = now;
	} else if (some) {
		want = middle;
		*last_load = now;
	} else if (now - *last_load >= ADRENO_DCVS_IDLE_NS) {
		want = level > 1 ? level - 1 : 1;
		*last_load = now;
	}
	if (want < 1) {
		want = 1;
	}
	return want > cap ? cap : want;
}

/* The GMU's busy counter (19.2 MHz ticks while the GPU is busy) over `elapsed_ns`: a percentage, or -1 for a reading
 * that cannot be (a reset, a gap under 1 ms or over a second, more busy than time with 1 % to spare). */
static inline int
adreno_busy_pct(uint64_t ticks, uint64_t previous, uint64_t elapsed_ns)
{
	uint64_t delta = ticks - previous, most = elapsed_ns * 192 / 10000;

	if (ticks < previous || elapsed_ns < 1000000 || elapsed_ns > 1000000000 || delta > most + most / 100) {
		return -1;
	}
	return delta >= most ? 100 : (int)(delta * 100 / most);
}

#endif
