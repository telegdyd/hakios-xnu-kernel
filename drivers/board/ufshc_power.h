/*
 * UFS power domain collapse while idle in Hibern8, so CX may collapse when APSS sleeps
 * (board/soc_sleep.c calls hakios_ufs_power_down() at llcc-off on the last core up).
 * Included by board/ufshc.c after host register helpers and hc_auto_restore().
 */
#define HC_GDSCR                0x4u            /* in hc_gcc (GCC + 0x87000): ufs_phy_gdsc */
#define HC_GDSC_COLLAPSE        (1u << 0)
#define HC_GDSC_RETAIN_FF       (1u << 11)
#define HC_GDSC_PWR_ON          (1u << 31)
#define HC_PD_CHECKS            8u

extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));
static vm_offset_t hc_gcc;                      /* GCC UFS block (+0x87000); 0 unless soc-sleep & gcc present */
static uint64_t hc_us_since(uint64_t start);

static const uint32_t hc_pd_regs[HC_PD_CHECKS] = {
	UFSHC_HCE, UFSHC_UTRLBA, UFSHC_UTRLBAU, UFSHC_UTRLRSR, UFSHC_AHIT, 0xdcu, 0xe0u, 0xfcu,
};
static uint32_t hc_pd_saved[HC_PD_CHECKS], hc_pd_back[HC_PD_CHECKS];
static int hc_pd_on = 1;                        /* boot-arg / sysctl ufs_power_down */
static volatile bool hc_pd_off;
static volatile uint32_t hc_pd_spin;
static volatile uint32_t hc_pd_users;
static bool hc_pd_lost;
static uint64_t hc_pd_downs, hc_pd_ups, hc_pd_refused, hc_pd_busy, hc_pd_not_h8, hc_pd_up_us_max, hc_pd_down_us_max;
static uint32_t hc_pd_gdscr_down, hc_pd_gdscr_up, hc_pd_h8[5];

static inline uint32_t
hc_pd_gdscr_read(void)
{
	return rd(hc_gcc + HC_GDSCR);
}

static inline void
hc_pd_gdscr_write(uint32_t value)
{
	wr(hc_gcc + HC_GDSCR, value);
}

static bool
hc_pd_wait(bool on, uint32_t us, uint64_t *most)
{
	uint64_t start = mach_absolute_time(), took;

	while (((hc_pd_gdscr_read() & HC_GDSC_PWR_ON) != 0) != on) {
		if (hc_us_since(start) > us) {
			return false;
		}
		__builtin_arm_yield();
	}
	took = hc_us_since(start);
	if (took > *most) {
		*most = took;
	}
	return true;
}

static void
hc_pd_lock(void)
{
	while (!os_atomic_cmpxchg(&hc_pd_spin, 0, 1, acquire)) {
		__builtin_arm_yield();
	}
}

static void
hc_pd_unlock(void)
{
	__atomic_store_n(&hc_pd_spin, 0, __ATOMIC_RELEASE);
}

static bool
hc_pd_up_locked(void)
{
	bool same = true;

	if (!hc_pd_off) {
		return true;
	}
	hc_pd_gdscr_write(hc_pd_gdscr_read() & ~HC_GDSC_COLLAPSE);
	hakios_barrier();
	if (!hc_pd_wait(true, 2000, &hc_pd_up_us_max)) {
		same = false;
	}
	hc_pd_gdscr_up = hc_pd_gdscr_read();
	for (uint64_t start = mach_absolute_time(); hc_us_since(start) < 2;) {
		__builtin_arm_yield();
	}
	for (unsigned i = 0; i < HC_PD_CHECKS && same; i++) {
		unsigned v = 0;

		if (hakios_mmio_try_read32 == NULL) {
			v = hc_read(hc_pd_regs[i]);
		} else if (hakios_mmio_try_read32((const volatile void *)(hc + hc_pd_regs[i]), &v) != 0) {
			v = 0xdeadbeefu;
		}
		hc_pd_back[i] = v;
		if (v != hc_pd_saved[i]) {
			same = false;
		}
	}
	hc_pd_off = false;
	hc_pd_ups++;
	if (!same) {
		hc_pd_lost = true;
		hc_dead = true;
	}
	return same;
}

static bool
hc_pd_down_locked(void)
{
	uint32_t g;

	for (unsigned i = 0; i < HC_PD_CHECKS; i++) {
		hc_pd_saved[i] = hc_read(hc_pd_regs[i]);
	}
	g = hc_pd_gdscr_read();
	hc_pd_gdscr_write(g | HC_GDSC_RETAIN_FF);
	hakios_barrier();
	hc_pd_gdscr_write(g | HC_GDSC_RETAIN_FF | HC_GDSC_COLLAPSE);
	hakios_barrier();
	if (!hc_pd_wait(false, 500, &hc_pd_down_us_max)) {
		hc_pd_gdscr_write(g | HC_GDSC_RETAIN_FF);
		hakios_barrier();
		(void)hc_pd_wait(true, 2000, &hc_pd_up_us_max);
		hc_pd_refused++;
		return false;
	}
	hc_pd_gdscr_down = hc_pd_gdscr_read();
	hc_pd_off = true;
	hc_pd_downs++;
	return true;
}

static bool
hc_pd_link_h8(void)
{
	hc_pd_h8[0] = hc_read(0x2700u);
	hc_pd_h8[1] = hc_read(0x270cu);
	hc_pd_h8[2] = hc_read(0x2704u);
	hc_pd_h8[3] = hc_read(0x2710u);
	hc_pd_h8[4] = hc_read(0x2708u);
	return ((hc_pd_h8[0] - hc_pd_h8[1]) & 0xffffu) == 1u && hc_pd_h8[2] == hc_pd_h8[3] && hc_pd_h8[4] == 0;
}

static bool
hc_pd_idle(void)
{
	return hc != 0 && hc_gcc != 0 && hc_usable && hc_in && !hc_dead && !hc_closed &&
	       hc_auto_owned && hc_proven && *hc_loader.take_back == 0 &&
	       __atomic_load_n(&hc_pd_users, __ATOMIC_ACQUIRE) == 0 && hc_in_flight == 0 &&
	       hc_read(UFSHC_UTRLDBR) == 0 && hc_read(UFSHC_UTMRLDBR) == 0;
}

bool hakios_ufs_power_down(void);
bool
hakios_ufs_power_down(void)
{
	bool off;

	if (hc_pd_off) {
		return true;
	}
	if (!hc_pd_on || hc_pd_lost || hc_gcc == 0) {
		return false;
	}
	if (!os_atomic_cmpxchg(&hc_pd_spin, 0, 1, acquire)) {
		hc_pd_busy++;
		return false;
	}
	off = hc_pd_off;
	if (!off) {
		if (!hc_pd_idle()) {
			hc_pd_busy++;
		} else if (!hc_pd_link_h8()) {
			hc_pd_not_h8++;
		} else {
			off = hc_pd_down_locked();
		}
	}
	hc_pd_unlock();
	return off;
}

static bool
hc_pd_wake(void)
{
	bool ok;

	if (!hc_pd_off) {
		return true;
	}
	hc_pd_lock();
	ok = hc_pd_up_locked();
	hc_pd_unlock();
	return ok;
}

static void
hc_pd_hold(void)
{
	(void)__atomic_add_fetch(&hc_pd_users, 1, __ATOMIC_ACQ_REL);
	if (hc_pd_off && hc_gcc != 0) {
		boolean_t istate = ml_set_interrupts_enabled(FALSE);

		(void)hc_pd_wake();
		(void)ml_set_interrupts_enabled(istate);
	}
}

static void
hc_pd_release(void)
{
	(void)__atomic_sub_fetch(&hc_pd_users, 1, __ATOMIC_ACQ_REL);
}

static void
hc_pd_final(void)
{
	hc_pd_on = 0;
	(void)__atomic_add_fetch(&hc_pd_users, 1, __ATOMIC_ACQ_REL);
	if (hc_pd_off && hc_gcc != 0) {
		__atomic_store_n(&hc_pd_spin, 0, __ATOMIC_RELEASE);
		(void)hc_pd_wake();
	}
}

static int
hc_pd_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = hc_pd_on, changed = 0, error = sysctl_io_number(req, value, sizeof(value), &value, &changed);

	if (error || !changed) {
		return error;
	}
	if (value != 0 && value != 1) {
		return EINVAL;
	}
	if (value && hc_pd_lost) {
		return EIO;
	}
	hc_pd_on = value;
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ufs_power_down, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0, hc_pd_sysctl, "I",
    "UFS power domain off at llcc-off while idle in Hibern8 (flop retention): 1 on, 0 off");

static int
hc_pd_cycle_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = 0, changed = 0, error = sysctl_io_number(req, value, sizeof(value), &value, &changed);
	boolean_t istate;
	bool down = false, up = true, h8 = false;

	if (error || !changed) {
		return error;
	}
	if (value != 1) {
		return EINVAL;
	}
	if (hc_pd_lost || hc_gcc == 0) {
		return hc_pd_lost ? EIO : ENXIO;
	}
	lck_mtx_lock(&hc_lock);
	hc_pd_hold();
	for (int i = 0; i < 100 && !(h8 = hc_pd_link_h8()); i++) {
		lck_mtx_unlock(&hc_lock);
		IOSleep(10);
		lck_mtx_lock(&hc_lock);
	}
	istate = ml_set_interrupts_enabled(FALSE);
	hc_pd_lock();
	hc_pd_release();
	if (!h8) {
		error = EAGAIN;
	} else if (!hc_pd_idle()) {
		error = EBUSY;
	} else {
		down = hc_pd_down_locked();
		if (down) {
			for (uint64_t start = mach_absolute_time(); hc_us_since(start) < 2000;) {
				__builtin_arm_yield();
			}
			up = hc_pd_up_locked();
		}
		error = !down ? EBUSY : up ? 0 : EIO;
	}
	hc_pd_unlock();
	(void)ml_set_interrupts_enabled(istate);
	lck_mtx_unlock(&hc_lock);
	hc_say("ufs: native: power cycle: %s (GDSCR down %08x up %08x)\n",
	    error == 0 ? "registers kept" : error == EIO ? "REGISTERS LOST: path off" : "not done",
	    hc_pd_gdscr_down, hc_pd_gdscr_up);
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ufs_power_cycle, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0, hc_pd_cycle_sysctl, "I",
    "test: write 1 for one UFS power domain off/on now, registers checked");

static int
hc_pd_stats_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[1024];
	int n;

	if (req->oldptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, text, sizeof(text));
	}
	n = snprintf(text, sizeof(text),
	    "on %d off %u lost %u downs %llu ups %llu refused %llu busy %llu not_h8 %llu "
	    "down_us_max %llu up_us_max %llu users %u\ngdscr now %08x down %08x up %08x; h8 counters %u %u %u %u %u\nchecks",
	    hc_pd_on, hc_pd_off, hc_pd_lost, hc_pd_downs, hc_pd_ups, hc_pd_refused, hc_pd_busy, hc_pd_not_h8,
	    hc_pd_down_us_max, hc_pd_up_us_max, hc_pd_users, hc_gcc != 0 ? hc_pd_gdscr_read() : 0u,
	    hc_pd_gdscr_down, hc_pd_gdscr_up, hc_pd_h8[0], hc_pd_h8[1], hc_pd_h8[2], hc_pd_h8[3], hc_pd_h8[4]);
	for (unsigned i = 0; i < HC_PD_CHECKS && n > 0 && n < (int)sizeof(text) - 32; i++) {
		n += snprintf(text + n, sizeof(text) - (size_t)n, " %x:%08x>%08x",
		    hc_pd_regs[i], hc_pd_saved[i], hc_pd_back[i]);
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ufs_power_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, hc_pd_stats_sysctl, "A",
    "UFS power domain: counts, GDSCR, the registers saved at the last power-down and read back at the power-up");
