/* UFS controller/UniPro hardware clock gating, owned independently of AHIT.
 * Masks and sequencing: OnePlus sm8450 ufs-qcom.c; GCC sm8450 clock driver.
 * Included after the host register helpers and AHIT restoration helper. */
#define HC_CFG2 0xe0
#define HC_AH8_CFG 0xfc
#define HC_CLK_UTP 1
#define HC_CLK_REQ 2
#define HC_CLK_GCC 4
#define HC_CLK_UNIPRO 8
/* hc_gcc: hakios.ufshc-power.h */
static const uint32_t hc_gcc_regs[] = { 0x18, 0x64, 0x6c, 0xa4, 0xd4 };
static uint32_t hc_gcc_original[5], hc_cfg2_original, hc_ah8_original;
static const uint32_t hc_dme_attrs[] = { 0xa00b, 0x9004, 0xd002 };
/* Preserve PA bit 8: vendor policy depends on clock rate and disable-cgc.
 * The low eight gates are permitted by both vendor policies. */
static const uint32_t hc_dme_masks[] = { 0x3ff, 0xff, 0x200 };
static uint32_t hc_dme_original[3], hc_dme_last[3];
static int hc_clock_boot_mask = 15, hc_clock_mask;
static bool hc_clock_saved, hc_clock_dirty, hc_uic_failed;
static uint64_t hc_clock_errors, hc_uic_commands, hc_uic_errors;
static uint64_t hc_us_since(uint64_t start);

static uint32_t hc_gcc_read(unsigned n) {
    return *(volatile uint32_t *)(hc_gcc + hc_gcc_regs[n]);
}
static bool hc_gcc_mask(unsigned n, uint32_t value) {
    uint32_t now = hc_gcc_read(n);
    *(volatile uint32_t *)(hc_gcc + hc_gcc_regs[n]) = (now & ~2u) | (value & 2u);
    __asm__ volatile ("dsb sy" ::: "memory");
    return (hc_gcc_read(n) & 2u) == (value & 2u);
}
static bool hc_reg_mask(uint32_t reg, uint32_t mask, uint32_t value) {
    hc_write(reg, (hc_read(reg) & ~mask) | (value & mask));
    return (hc_read(reg) & mask) == (value & mask);
}

/* Local DME GET/SET only, controller exclusively owned and AHIT stopped.
 * No link-startup/reset, device queries, task management, or SCSI writes.
 * A timeout prevents subsequent UIC commands; acknowledge only UIC completion.
 * Never read the clear-on-read UIC error-code registers. */
static int hc_dme(bool set, uint32_t attr, uint32_t *value) {
    uint64_t start;
    if (hc_uic_failed) return EIO;
    for (start = mach_absolute_time(); !(hc_read(UFSHC_HCS) & 8); ) {
        if (hc_us_since(start) > 500000) goto timeout;
        __builtin_arm_yield();
    }
    hc_write(UFSHC_IS, 0x400);
    hc_write(0x94, attr << 16);
    hc_write(0x98, 0);
    hc_write(0x9c, set ? *value : 0);
    hc_write(0x90, set ? 2 : 1);
    hc_uic_commands++;
    for (start = mach_absolute_time(); !(hc_read(UFSHC_IS) & 0x400); ) {
        if (hc_us_since(start) > 500000) goto timeout;
        __builtin_arm_yield();
    }
    uint32_t result = hc_read(0x98) & 0xff;
    if (!set) *value = hc_read(0x9c);
    hc_write(UFSHC_IS, 0x400);
    if (result) { hc_uic_errors++; return ENOTSUP; }
    return 0;
timeout:
    hc_uic_errors++;
    hc_uic_failed = true;
    return ETIMEDOUT;
}

/* With hc_lock, or after all other processors have stopped at handover.
 * Restore GCC auto-gating first when disabling, so clocks remain available
 * while controller/UniPro settings are returned to their original values. */
static int hc_clock_program(int mask) {
    uint32_t ahit = hc_read(UFSHC_AHIT);
    int error = 0;
    hc_write(UFSHC_AHIT, 0);
    if (hc_read(UFSHC_AHIT) != 0) return EIO;
    if (!hc_clock_saved) {
        hc_cfg2_original = hc_read(HC_CFG2);
        hc_ah8_original = hc_read(HC_AH8_CFG);
        for (unsigned i=0; i<5; i++) hc_gcc_original[i] = hc_gcc_read(i);
        for (unsigned i=0; i<3; i++) {
            error = hc_dme(false, hc_dme_attrs[i], &hc_dme_original[i]);
            if (error) goto out;
            hc_dme_last[i] = hc_dme_original[i];
        }
        hc_clock_saved = true;
    }
    hc_clock_dirty = true;
    for (unsigned i=0; i<5; i++) {
        if (!hc_gcc_mask(i, (mask & HC_CLK_GCC) ? 2 : hc_gcc_original[i])) error = EIO;
    }
    if (error) goto out;
    /* Stop issuing clock requests only after restoring GCC availability. */
    if (!(mask & HC_CLK_REQ) && !hc_reg_mask(HC_AH8_CFG, 0x3c, hc_ah8_original)) { error=EIO; goto out; }
    if (!hc_reg_mask(HC_CFG2, 0xff, (mask & HC_CLK_UTP) ? 0xff : hc_cfg2_original)) { error=EIO; goto out; }
    for (unsigned i=0; i<3; i++) {
        uint32_t now, back;
        error = hc_dme(false, hc_dme_attrs[i], &now);
        if (error) goto out;
        uint32_t desired = (now & ~hc_dme_masks[i]) |
            ((mask & HC_CLK_UNIPRO) ? hc_dme_masks[i] : (hc_dme_original[i] & hc_dme_masks[i]));
        if (now != desired) {
            error = hc_dme(true, hc_dme_attrs[i], &desired);
            if (error) goto out;
        }
        error = hc_dme(false, hc_dme_attrs[i], &back);
        if (error) goto out;
        hc_dme_last[i] = back;
        if ((back & hc_dme_masks[i]) != (desired & hc_dme_masks[i])) { error=EIO; goto out; }
    }
    if ((mask & HC_CLK_REQ) && !hc_reg_mask(HC_AH8_CFG, 0x3c, 0x3c)) { error=EIO; goto out; }
    hc_clock_mask = mask;
    hc_clock_dirty = mask != 0;
out:
    hc_write(UFSHC_AHIT, ahit);
    if (hc_read(UFSHC_AHIT) != ahit) error = EIO;
    return error;
}
static bool hc_clock_restore(void) {
    if (!hc_clock_dirty) return true;
    int error = hc_clock_program(0);
    if (error) hc_clock_errors++;
    return !error;
}
static bool hc_power_restore(void) {
    bool timer_ok = hc_auto_restore();
    bool clocks_ok = hc_clock_restore();
    return timer_ok && clocks_ok;
}
static int hc_clock_set(int mask) {
    int error;
    if (mask < 0 || mask > 15) return EINVAL;
    lck_mtx_lock(&hc_lock);
    hc_pd_hold();
    if (!hc || !hc_gcc || !hc_usable || hc_closed || hc_dead || !hc_in ||
        !hc_write_window.enabled || *hc_loader.take_back) error = ENXIO;
    else if (hc_in_flight || hc_read(UFSHC_UTRLDBR) || hc_read(UFSHC_UTMRLDBR)) error = EBUSY;
    else if ((hc_read(UFSHC_HCS) & 0xf) != 0xf ||
        (hc_read(UFSHC_IS) & (IS_FATAL | 0x60))) error = EIO;
    else {
        error = hc_clock_program(mask);
        if (error) {
            hc_clock_errors++;
            /* Keep automatic entry disabled if clock/UIC restoration is
             * uncertain, rather than restarting idle with a partial setup. */
            (void)hc_power_restore();
        }
    }
    hc_pd_release();
    lck_mtx_unlock(&hc_lock);
    return error;
}
static int hc_clock_sysctl(__unused struct sysctl_oid *oidp, __unused void *arg1,
    __unused int arg2, struct sysctl_req *req) {
    int value, error;
    lck_mtx_lock(&hc_lock); value=hc_clock_mask; lck_mtx_unlock(&hc_lock);
    error=sysctl_handle_int(oidp,&value,0,req);
    return error || req->newptr == USER_ADDR_NULL ? error : hc_clock_set(value);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, ufs_clock_gating, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED,
    0, 0, hc_clock_sysctl, "I", "UFS idle clocks: 1 UTP, 2 requests, 4 GCC hardware control, 8 UniPro; 0 restores");
