/* Pure policy, shared with the host scenario test. Times are monotonic ns.
 * Level 1/2/3 means 220/317/492 MHz, never an expanded frequency table. */
#ifndef HAKIOS_DCVS_H
#define HAKIOS_DCVS_H
static unsigned
gpu_thermal_cap(unsigned previous, unsigned valid, int t0, int t1)
{
	if (valid != 3) return 1;
	int hottest = t0 > t1 ? t0 : t1;
	if (hottest >= 85000) return 1;
	if (previous == 1 && hottest >= 80000) return 1;
	if (hottest >= 75000) return 2;
	if (previous < 3 && hottest >= 70000) return 2;
	return 3;
}

static unsigned
gpu_dcvs_target(unsigned level, unsigned cap, unsigned busy_valid, unsigned busy_pct,
    unsigned pending, unsigned recent, unsigned long long now, unsigned long long *last_load)
{
	unsigned want = level;
	if (pending || (busy_valid && busy_pct >= 60)) {
		want = 3;
		*last_load = now;
	} else if ((busy_valid && busy_pct >= 20) || (!busy_valid && recent)) {
		want = 2;
		*last_load = now;
	} else if (now - *last_load >= 200000000ULL) {
		want = level > 1 ? level - 1 : 1;
		*last_load = now;
	}
	return want > cap ? cap : want;
}
#endif
