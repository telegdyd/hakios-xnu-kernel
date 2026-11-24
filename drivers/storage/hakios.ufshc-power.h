/*
 * UFS power domain off while idle (POWER-COLLAPSE-PLAN.md 5e), so that CX may collapse under it.
 * Included by hakios.ufshc.c after the clock helpers.
 *
 * Why: CX collapses (cx.lvl 0 in the sleep set) with ufs_phy_gdsc on left the UFS clock running in most llcc-off asks
 * afterwards and the phone froze minutes later (soc-sleep run8, run9): the domain's logic loses its supply under a
 * power switch that says on. So hakios.rpmh-arc.h collapses CX only while this domain is off.
 *
 * How: the link is already in Hibern8 after 150 ms idle (AHIT, owned by this kernel). At llcc-off, the last core up
 * (hakios_rpmh_llcc_down, interrupts off, every other core down) calls hakios_ufs_power_down(): nothing taken, nothing
 * rung, the controller's Hibern8 counters say the link is in Hibern8 (hardware entries one more than exits, no
 * software ones), nobody holding the domain on: the GDSC's flop retention set (GDSCR bit 11, RETAIN_FF_ENABLE, as
 * gdsc.c sets it for its RETAIN_FF_ENABLE domains) and SW_COLLAPSE, and PWR_ON awaited clear. The controller, UniPro
 * and the PHY (all in that domain, sm8450.dtsi) keep their state in the retention flops; the branch clocks are left as
 * they are (gdsc.c: clocks already on take 4 cycles more after the domain comes back).
 *
 * Back on at the next doorbell (hc_ring, interrupts off) or before any register access outside a request
 * (hc_pd_hold: the sysctls, the list's switches, the handover): SW_COLLAPSE clear, PWR_ON awaited, and the registers
 * this kernel set read back through hakios_mmio_try_read32 against their values at the power-down (HCE, the list's
 * address and run bit, AHIT, CFG1, CFG2, AH8_CFG). A mismatch (retention lost) turns the path off (hc_dead) and power
 * downs off for the boot (ufs_power_lost); the doorbell then exits Hibern8 as after an ordinary idle.
 *
 * Boot-arg ufs_power_down=0: never; sysctl kern.hakios.ufs_power_down (0/1) the same at run time.
 * kern.hakios.ufs_power_cycle (write 1): one off/on now, with the checks, from a thread (a test).
 * kern.hakios.ufs_power_stats: counts and the last check.
 */
#define HC_GDSCR                0x4             /* in hc_gcc (0x187000): GCC 0x87004, ufs_phy_gdsc */
#define HC_GDSC_COLLAPSE        (1u << 0)
#define HC_GDSC_RETAIN_FF       (1u << 11)
#define HC_GDSC_PWR_ON          (1u << 31)
#define HC_PD_CHECKS            8

extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));
static vm_offset_t hc_gcc;                      /* GCC 0x187000 (the UFS block); also the clock helpers' */
static uint64_t hc_us_since(uint64_t start);

static const uint32_t hc_pd_regs[HC_PD_CHECKS] = {
	UFSHC_HCE, UFSHC_UTRLBA, UFSHC_UTRLBAU, UFSHC_UTRLRSR, UFSHC_AHIT, 0xdc, 0xe0, 0xfc,
};
static uint32_t hc_pd_saved[HC_PD_CHECKS], hc_pd_back[HC_PD_CHECKS];
static int hc_pd_on = 1;                        /* boot-arg / sysctl ufs_power_down */
static volatile bool hc_pd_off;                 /* the domain is off now */
static volatile uint32_t hc_pd_spin;            /* power-up/down exclusion: a doorbell's processor vs the last core */
static volatile uint32_t hc_pd_users;           /* holding the domain on for register access */
static bool hc_pd_lost;                         /* a check failed: no more power downs this boot */
static uint64_t hc_pd_downs, hc_pd_ups, hc_pd_refused, hc_pd_busy, hc_pd_not_h8, hc_pd_up_us_max, hc_pd_down_us_max;
static uint32_t hc_pd_gdscr_down, hc_pd_gdscr_up, hc_pd_h8[5];

static inline volatile uint32_t *
hc_pd_gdscr(void)
{
	return (volatile uint32_t *)(hc_gcc + HC_GDSCR);
}

static bool
hc_pd_wait(bool on, uint32_t us, uint64_t *most)
{
	uint64_t start = mach_absolute_time(), took;

	while (((*hc_pd_gdscr() & HC_GDSC_PWR_ON) != 0) != on) {
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
	os_atomic_store(&hc_pd_spin, 0, release);
}

/* With hc_pd_spin. The domain on, its registers checked; false: they were lost (the path is off then). */
static bool
hc_pd_up_locked(void)
{
	bool same = true;

	if (!hc_pd_off) {
		return true;
	}
	*hc_pd_gdscr() &= ~HC_GDSC_COLLAPSE;
	__asm__ volatile ("dsb sy" ::: "memory");
	if (!hc_pd_wait(true, 2000, &hc_pd_up_us_max)) {
		same = false;
	}
	hc_pd_gdscr_up = *hc_pd_gdscr();
	/* (gdsc.c: 1 us before the clocks of a domain just powered are used) */
	for (uint64_t start = mach_absolute_time(); hc_us_since(start) < 2;) {
		__builtin_arm_yield();
	}
	for (unsigned i = 0; i < HC_PD_CHECKS && same; i++) {
		unsigned v = 0;

		if (hakios_mmio_try_read32 == NULL) {
			v = hc_read(hc_pd_regs[i]);
		} else if (hakios_mmio_try_read32((const volatile void *)(hc + hc_pd_regs[i]), &v) != 0) {
			v = 0xdeadbeef;         /* (the read faulted) */
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
		hc_dead = true;         /* (no hc_off(): its restore would write registers that are not what they were) */
	}
	return same;
}

/* With hc_pd_spin, nothing in flight, the link checked in Hibern8. */
static bool
hc_pd_down_locked(void)
{
	uint32_t g;

	for (unsigned i = 0; i < HC_PD_CHECKS; i++) {
		hc_pd_saved[i] = hc_read(hc_pd_regs[i]);
	}
	g = *hc_pd_gdscr();
	*hc_pd_gdscr() = g | HC_GDSC_RETAIN_FF;
	__asm__ volatile ("dsb sy" ::: "memory");
	*hc_pd_gdscr() = g | HC_GDSC_RETAIN_FF | HC_GDSC_COLLAPSE;
	__asm__ volatile ("dsb sy" ::: "memory");
	if (!hc_pd_wait(false, 500, &hc_pd_down_us_max)) {
		/* (it stayed on: someone else's vote, or the hardware's; back as it was, still on, nothing lost) */
		*hc_pd_gdscr() = g | HC_GDSC_RETAIN_FF;
		__asm__ volatile ("dsb sy" ::: "memory");
		(void)hc_pd_wait(true, 2000, &hc_pd_up_us_max);
		hc_pd_refused++;
		return false;
	}
	hc_pd_gdscr_down = *hc_pd_gdscr();
	hc_pd_off = true;
	hc_pd_downs++;
	return true;
}

/* The link in Hibern8: hardware entries one more than exits, no software ones (the vendor's counters, 0x2700..) */
static bool
hc_pd_link_h8(void)
{
	hc_pd_h8[0] = hc_read(0x2700);
	hc_pd_h8[1] = hc_read(0x270c);
	hc_pd_h8[2] = hc_read(0x2704);
	hc_pd_h8[3] = hc_read(0x2710);
	hc_pd_h8[4] = hc_read(0x2708);
	return ((hc_pd_h8[0] - hc_pd_h8[1]) & 0xffff) == 1 && hc_pd_h8[2] == hc_pd_h8[3] && hc_pd_h8[4] == 0;
}

static bool
hc_pd_idle(void)
{
	return hc && hc_gcc && hc_usable && hc_in && !hc_dead && !hc_closed && hc_auto_owned && hc_write_window.enabled &&
	       *hc_loader.take_back == 0 && os_atomic_load(&hc_pd_users, acquire) == 0 && hc_in_flight == 0 &&
	       hc_read(UFSHC_UTRLDBR) == 0 && hc_read(UFSHC_UTMRLDBR) == 0;
}

/*
 * hakios.rpmh-arc.h, at llcc-off: the last core up, interrupts off, every other core down. True: the domain is off
 * (now or already).
 */
bool hakios_ufs_power_down(void);
bool
hakios_ufs_power_down(void)
{
	bool off;

	if (hc_pd_off) {
		return true;
	}
	if (!hc_pd_on || hc_pd_lost) {
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

/* Before a doorbell (hc_ring: hc_lock, interrupts off, the slot already taken). False: the registers were lost. */
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

/* The domain on, and kept on until hc_pd_release (any register access outside a request in flight) */
static void
hc_pd_hold(void)
{
	os_atomic_inc(&hc_pd_users, acq_rel);
	if (hc_pd_off && hc_gcc) {
		boolean_t istate = ml_set_interrupts_enabled(FALSE);

		(void)hc_pd_wake();
		ml_set_interrupts_enabled(istate);
	}
}

static void
hc_pd_release(void)
{
	os_atomic_dec(&hc_pd_users, acq_rel);
}

/* the loader's way back (other processors stopped, no locks): on for good */
static void
hc_pd_final(void)
{
	hc_pd_on = 0;
	os_atomic_inc(&hc_pd_users, acq_rel);
	if (hc_pd_off && hc_gcc) {
		os_atomic_store(&hc_pd_spin, 0, release);       /* (a stopped processor cannot be in it: they stop at idle) */
		(void)hc_pd_wake();
	}
}

static int
hc_pd_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = hc_pd_on, error = sysctl_handle_int(oidp, &value, 0, req);

	if (error || req->newptr == USER_ADDR_NULL) {
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

/* one off/on now (a test): the controller idle, the link in Hibern8 within a second */
static int
hc_pd_cycle_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = 0, error = sysctl_handle_int(oidp, &value, 0, req);
	boolean_t istate;
	bool down = false, up = true, h8 = false;

	if (error || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	if (value != 1) {
		return EINVAL;
	}
	if (hc_pd_lost) {
		return EIO;
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
	ml_set_interrupts_enabled(istate);
	lck_mtx_unlock(&hc_lock);
	hc_say("ufs: native: power cycle: %s (GDSCR down %08x up %08x)\n", error == 0 ? "registers kept" :
	    error == EIO ? "REGISTERS LOST: path off" : "not done", hc_pd_gdscr_down, hc_pd_gdscr_up);
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
	n = snprintf(text, sizeof(text), "on %d off %u lost %u downs %llu ups %llu refused %llu busy %llu not_h8 %llu "
	    "down_us_max %llu up_us_max %llu users %u\ngdscr now %08x down %08x up %08x; h8 counters %u %u %u %u %u\nchecks",
	    hc_pd_on, hc_pd_off, hc_pd_lost, hc_pd_downs, hc_pd_ups, hc_pd_refused, hc_pd_busy, hc_pd_not_h8, hc_pd_down_us_max,
	    hc_pd_up_us_max, hc_pd_users, hc_gcc ? *hc_pd_gdscr() : 0, hc_pd_gdscr_down, hc_pd_gdscr_up, hc_pd_h8[0], hc_pd_h8[1],
	    hc_pd_h8[2], hc_pd_h8[3], hc_pd_h8[4]);
	for (unsigned i = 0; i < HC_PD_CHECKS && n > 0 && n < (int)sizeof(text) - 32; i++) {
		n += snprintf(text + n, sizeof(text) - (size_t)n, " %x:%08x>%08x", hc_pd_regs[i], hc_pd_saved[i], hc_pd_back[i]);
	}
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ufs_power_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, hc_pd_stats_sysctl, "A",
    "UFS power domain: counts, GDSCR, the registers saved at the last power-down and read back at the power-up");
