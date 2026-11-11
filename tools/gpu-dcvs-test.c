/* cc -std=c11 -Wall -Wextra -Werror tools/gpu-dcvs-test.c -o /tmp/gpu-dcvs-test && /tmp/gpu-dcvs-test */
#include <assert.h>
#include <stdio.h>
#include "../drivers/gpu/hakios.dcvs.h"

int main(void)
{
	/* Hotter sensor wins, inclusive limits, missing sensors fail closed. */
	assert(gpu_thermal_cap(3, 3, 74000, 75000) == 2);
	assert(gpu_thermal_cap(3, 3, 85000, 60000) == 1);
	assert(gpu_thermal_cap(1, 3, 80000, 60000) == 1);
	assert(gpu_thermal_cap(1, 3, 79900, 60000) == 2);
	assert(gpu_thermal_cap(2, 3, 70000, 60000) == 2);
	assert(gpu_thermal_cap(2, 3, 69900, 60000) == 3);
	for (unsigned valid = 0; valid < 3; valid++)
		assert(gpu_thermal_cap(3, valid, 0, 0) == 1);

	unsigned long long last = 0;
	assert(gpu_dcvs_target(3, 3, 1, 0, 0, 0, 199999999, &last) == 3);
	assert(gpu_dcvs_target(3, 3, 1, 0, 0, 0, 200000000, &last) == 2);
	assert(gpu_dcvs_target(2, 3, 1, 0, 0, 0, 399999999, &last) == 2);
	assert(gpu_dcvs_target(2, 3, 1, 0, 0, 0, 400000000, &last) == 1);
	assert(gpu_dcvs_target(1, 3, 1, 60, 0, 0, 450000000, &last) == 3);
	assert(gpu_dcvs_target(1, 3, 1, 20, 0, 0, 500000000, &last) == 2);
	assert(gpu_dcvs_target(1, 3, 0, 0, 1, 0, 550000000, &last) == 3);
	/* Invalid counters use recent-work hints, never submission timing %. */
	assert(gpu_dcvs_target(1, 3, 0, 100, 0, 1, 600000000, &last) == 2);
	assert(gpu_dcvs_target(3, 3, 0, 100, 0, 0, 800000000, &last) == 2);
	for (unsigned cap = 1; cap <= 3; cap++) {
		for (unsigned level = 1; level <= 3; level++) {
			for (unsigned busy = 0; busy <= 100; busy++) {
				unsigned result = gpu_dcvs_target(level, cap, 1, busy, 1, 1, 1000000000, &last);
				assert(result >= 1 && result <= cap);
			}
		}
	}
	puts("PASS: thermal thresholds/recovery, sensor loss, load response, idle hysteresis, fallback, all ceilings");
	return 0;
}
