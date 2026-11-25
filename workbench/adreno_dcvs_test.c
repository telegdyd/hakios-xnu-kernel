/*
 * gpu-dcvs's policy (drivers/board/adreno_dcvs.h): the cap the GPU's sensors set, the speed a load asks for, and the
 * busy counter's readings, with three speeds (the OnePlus 10 Pro's 220, 317, 492 MHz) and with others.
 */
#include <assert.h>
#include <stdio.h>

#include "../drivers/board/adreno_dcvs.h"

int
main(void)
{
	uint64_t last = 0;

	/* the hotter sensor counts; down at 75 and 85, back up 5 degrees lower; a sensor without a reading: the slowest */
	assert(adreno_thermal_cap(3, 3, 3, 74000, 75000) == 2);
	assert(adreno_thermal_cap(3, 3, 3, 74999, 60000) == 3);
	assert(adreno_thermal_cap(3, 3, 3, 85000, 60000) == 1);
	assert(adreno_thermal_cap(1, 3, 3, 80000, 60000) == 1);
	assert(adreno_thermal_cap(1, 3, 3, 79900, 60000) == 2);
	assert(adreno_thermal_cap(2, 3, 3, 70000, 60000) == 2);
	assert(adreno_thermal_cap(2, 3, 3, 69900, 60000) == 3);
	for (unsigned valid = 0; valid < 3; valid++) {
		assert(adreno_thermal_cap(3, 3, valid, 30000, 30000) == 1);
	}
	assert(adreno_thermal_cap(5, 5, 3, 76000, 0) == 4);
	assert(adreno_thermal_cap(1, 1, 3, 30000, 30000) == 1);

	/* the SoC's limiter: nothing held back the top, all of it the slowest, in between to the nearest speed */
	assert(adreno_soc_cap(3, 0, 10) == 3);
	assert(adreno_soc_cap(3, 0, 0) == 3);
	assert(adreno_soc_cap(3, 5, 0) == 3);
	assert(adreno_soc_cap(3, 2, 10) == 3);
	assert(adreno_soc_cap(3, 3, 10) == 2);
	assert(adreno_soc_cap(3, 7, 10) == 2);
	assert(adreno_soc_cap(3, 8, 10) == 1);
	assert(adreno_soc_cap(3, 10, 10) == 1);
	assert(adreno_soc_cap(3, 12, 10) == 1);
	assert(adreno_soc_cap(3, 4, 17) == 3);    /* (the phone: 17 rows; a run at 64-69 C held back 1 to 4) */
	assert(adreno_soc_cap(3, 5, 17) == 2);
	assert(adreno_soc_cap(3, 13, 17) == 1);
	assert(adreno_soc_cap(5, 1, 8) == 4);    /* (half a speed: rounded up) */
	assert(adreno_soc_cap(5, 7, 8) == 1);
	assert(adreno_soc_cap(5, 6, 8) == 2);
	assert(adreno_soc_cap(1, 3, 8) == 1);
	assert(adreno_soc_cap(0, 0, 8) == 1);

	/* with the busy counter */
	assert(adreno_dcvs_target(3, 3, 3, true, 0, false, false, 199999999, &last) == 3);
	assert(adreno_dcvs_target(3, 3, 3, true, 0, false, false, 200000000, &last) == 2);
	assert(adreno_dcvs_target(2, 3, 3, true, 0, false, false, 399999999, &last) == 2);
	assert(adreno_dcvs_target(2, 3, 3, true, 0, false, false, 400000000, &last) == 1);
	assert(adreno_dcvs_target(1, 3, 3, true, 60, false, false, 450000000, &last) == 3);
	assert(adreno_dcvs_target(3, 3, 3, true, 20, false, false, 500000000, &last) == 2);
	assert(adreno_dcvs_target(1, 3, 3, true, 19, true, true, 600000000, &last) == 1);   /* (pending: the counter says) */
	assert(adreno_dcvs_target(1, 3, 2, true, 100, false, false, 650000000, &last) == 2);
	/* without it: queued work, recent work */
	assert(adreno_dcvs_target(1, 3, 3, false, 0, true, false, 700000000, &last) == 3);
	assert(adreno_dcvs_target(3, 3, 3, false, 0, false, true, 750000000, &last) == 2);
	assert(adreno_dcvs_target(2, 3, 3, false, 0, false, false, 950000000, &last) == 1);
	for (unsigned top = 1; top <= 5; top++) {
		for (unsigned cap = 1; cap <= top; cap++) {
			for (unsigned level = 1; level <= top; level++) {
				for (unsigned busy = 0; busy <= 100; busy++) {
					unsigned got = adreno_dcvs_target(level, top, cap, busy & 1, busy, busy & 2, busy & 4,
					    1000000000ull + busy * 300000000ull, &last);
					assert(got >= 1 && got <= cap);
				}
			}
		}
	}

	/* the counter: 19.2 MHz ticks over the time between two looks */
	assert(adreno_busy_pct(960000, 0, 50000000) == 100);
	assert(adreno_busy_pct(480000, 0, 50000000) == 50);
	assert(adreno_busy_pct(969600, 0, 50000000) == 100);           /* 1 % over: sampling skew */
	assert(adreno_busy_pct(969601, 0, 50000000) == -1);
	assert(adreno_busy_pct(10, 20, 50000000) == -1);                /* backwards: a reset */
	assert(adreno_busy_pct(1000, 0, 999999) == -1);
	assert(adreno_busy_pct(1000, 0, 1000000001) == -1);
	assert(adreno_busy_pct(0, 0, 50000000) == 0);
	puts("adreno_dcvs_test: ok");
	return 0;
}
