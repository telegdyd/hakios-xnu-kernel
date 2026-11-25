/* Internal RC0 link PM. PCIe ordering/timing: Linux v6.12 pcie/aspm.c;
 * PARF AUX sync/CLKREQ policy and 150*1024 ns L1.2 threshold: OnePlus
 * sm8450 pci-msm.c and this phone's live.dts. No L0s, PCI-PM D-state,
 * rail or bandwidth changes here. Caller owns link startup/shutdown. */
#ifndef HAKIOS_PCIE_PM_H
#define HAKIOS_PCIE_PM_H
#define PM_SS_MASK 0xe3ffff0fu
static int pm_requested, pm_active, pm_error;
static unsigned pm_prepared;
static bool pm_valid, pm_owned;
static unsigned pm_sys_old, pm_clkreq_old, pm_ack_old;
static unsigned long long pm_samples, pm_l1_samples, pm_ss_samples;
static unsigned pm_last_ltssm, pm_last_status;
static int pm_parf(unsigned off, unsigned mask, unsigned value)
{
	unsigned old, actual;
	CHECK(rd(parf + off, &old));
	CHECK(wr(parf + off, (old & ~mask) | (value & mask)));
	CHECK(rd(parf + off, &actual));
	return (actual & mask) == (value & mask) ? 0 : EIO;
}

static int pm_cfg(unsigned i, unsigned off, unsigned size, unsigned mask, unsigned want)
{
	unsigned old, actual;
	CHECK(hakios_pcie_cfg_read(0, pm_ports[i].bdf, off, size, &old));
	CHECK(hakios_pcie_cfg_write(0, pm_ports[i].bdf, off, size, (old & ~mask) | (want & mask)));
	CHECK(hakios_pcie_cfg_read(0, pm_ports[i].bdf, off, size, &actual));
	return (actual & mask) == (want & mask) ? 0 : EIO;
}
static int pm_retrain(void)
{
	unsigned v;
	CHECK(hakios_pcie_cfg_read(0, 0, pm_ports[0].pc + 0x10, 2, &v));
	CHECK(hakios_pcie_cfg_write(0, 0, pm_ports[0].pc + 0x10, 2, v | (1u << 5)));
	delay(100);
	for (unsigned us = 0; us < 100000; us += 100) {
		CHECK(hakios_pcie_cfg_read(0, 0, pm_ports[0].pc + 0x12, 2, &v));
		if (!(v & (1u << 11)) && (!(pm_ports[0].lc & (1u << 20)) || (v & (1u << 13)))) return 0;
		delay(100);
	}
	return ETIMEDOUT;
}
static int pm_disable(void)
{
	CHECK(pm_cfg(1, pm_ports[1].pc + 0x10, 2, 3, 0));
	CHECK(pm_cfg(0, pm_ports[0].pc + 0x10, 2, 3, 0));
	for (int i = 1; i >= 0; i--) if (pm_ports[i].ss)
		CHECK(pm_cfg(i, pm_ports[i].ss + 8, 4, 15, 0));
	return 0;
}
static int pm_restore(void)
{
	if (!pm_owned) return 0;
	int first = pm_disable(), e;
	pm_active = 0;
	if (first) return first; /* never change timing with an uncertain L1/L1SS enable */
#define PM_TRY(x) do { e = (x); if (!first && e) first = e; } while (0)
	for (unsigned i = 0; i < 2; i++) {
		if (pm_ports[i].ss) {
			PM_TRY(pm_cfg(i, pm_ports[i].ss + 12, 4, 0xfb, pm_ports[i].s2));
			PM_TRY(pm_cfg(i, pm_ports[i].ss + 8, 4, PM_SS_MASK & ~15u, pm_ports[i].s1));
		}
		PM_TRY(pm_cfg(i, pm_ports[i].pc + 0x28, 2, 1u << 10, pm_ports[i].dctl2));
	}
	PM_TRY(pm_parf(0, 8, pm_sys_old));
	PM_TRY(pm_parf(0x2b0, 0xa, pm_clkreq_old));
	PM_TRY(pm_cfg(0, 0x70c, 4, 1u << 30, pm_ack_old));
	for (int i = 1; i >= 0; i--)
		PM_TRY(pm_cfg(i, pm_ports[i].pc + 0x10, 2, 0x143, pm_ports[i].ctl & ~3u));
	if (!first && (!(pm_ports[0].ctl & 0x40) || !(pm_ports[1].ctl & 0x40))) PM_TRY(pm_retrain());
#undef PM_TRY
	if (first) return first; /* keep substates disabled after a partial restore */
	/* Original substates first at parent, then child; L1 last. */
	for (unsigned i = 0; i < 2; i++) if (pm_ports[i].ss)
		CHECK(pm_cfg(i, pm_ports[i].ss + 8, 4, 15, pm_ports[i].s1));
	for (unsigned i = 0; i < 2; i++) CHECK(pm_cfg(i, pm_ports[i].pc + 0x10, 2, 3, pm_ports[i].ctl));
	pm_owned = false; pm_prepared = 0;
	return 0;
}
static int pm_prepare(unsigned mode)
{
	if (!mode) return 0;
	if (!pm_valid || mode > 3) return ENOTSUP;
	unsigned acceptable = (pm_ports[1].dc >> 9) & 7;
	for (unsigned i = 0; i < 2; i++) {
		unsigned exit = (pm_ports[i].lc >> 15) & 7;
		if (acceptable != 7 && (exit == 7 || exit > acceptable)) return ENOTSUP;
	}
	for (unsigned i = 0; i < 2; i++) {
		if (!(pm_ports[i].lc & (1u << 11))) return ENOTSUP;
		if (mode >= 2 && (!pm_ports[i].ss || !(pm_ports[i].sc & 8))) return ENOTSUP;
		if (mode == 3 && (!(pm_ports[i].sc & 4) || !(pm_ports[i].dc2 & (1u << 11)))) return ENOTSUP;
		if (!(pm_ports[i].sta & (1u << 12))) return ENOTSUP; /* both use the slot clock */
	}
	CHECK(rd(parf, &pm_sys_old));
	CHECK(rd(parf + 0x2b0, &pm_clkreq_old));
	CHECK(hakios_pcie_cfg_read(0, 0, 0x70c, 4, &pm_ack_old));
	pm_owned = true;
	pm_prepared = mode;
	CHECK(pm_disable());
	CHECK(pm_cfg(1, pm_ports[1].pc + 0x10, 2, 0x40, 0x40));
	CHECK(pm_cfg(0, pm_ports[0].pc + 0x10, 2, 0x40, 0x40));
	if (!(pm_ports[0].ctl & 0x40) || !(pm_ports[1].ctl & 0x40)) CHECK(pm_retrain());
	/* AUX/core sync and hardware CLKREQ, as the vendor's link-PM path. */
	CHECK(pm_parf(0, 8, 0));
	CHECK(pm_parf(0x2b0, 0xa, 0));
	/* DesignWare ENTER_ASPM: idle L0 may enter L1 directly. With this
	 * clear, entry requires L0s first, which these profiles leave off.
	 * Same root-port policy as Linux pcie-tegra194.c (PORT_AFR). */
	CHECK(pm_cfg(0, 0x70c, 4, 1u << 30, 1u << 30));
	unsigned v;
	CHECK(rd(parf, &v)); if (v & (1u << 3)) return EIO;
	CHECK(rd(parf + 0x2b0, &v)); if (v & 0xa) return EIO;
	return 0;
}
static unsigned pm_power_us(unsigned scale, unsigned value)
{
	return scale == 0 ? value * 2 : scale == 1 ? value * 10 : scale == 2 ? value * 100 : ~0u;
}
static int pm_activate(unsigned mode)
{
	if (!mode) return 0;
	if (!pm_owned) return EINVAL;
	CHECK(pm_disable());
	if (mode >= 2) {
		unsigned cm = 0, power = 0, power_word = 0;
		for (unsigned i = 0; i < 2; i++) {
			unsigned c = (pm_ports[i].sc >> 8) & 255;
			if (c > cm) cm = c;
			unsigned candidates[2] = { ((pm_ports[i].sc >> 16) & 3) | (((pm_ports[i].sc >> 19) & 31) << 3), pm_ports[i].s2 & 0xfb };
			for (unsigned j = 0; j < 2; j++) {
				unsigned p = pm_power_us(candidates[j] & 3, (candidates[j] >> 3) & 31);
				if (p == ~0u) return ENOTSUP;
				if (p >= power) { power = p; power_word = candidates[j]; }
			}
		}
		/* Round the spec's 6 us + restore + power-on up, and retain the
		 * device tree's more conservative 150*1024 ns minimum threshold. */
		unsigned ns = (6 + cm + power) * 1000;
		if (ns < 153600) ns = 153600;
		unsigned scale = 0, factor = 1;
		while ((ns + factor - 1) / factor > 1023 && scale < 5) { scale++; factor *= 32; }
		unsigned threshold = ((ns + factor - 1) / factor) << 16 | scale << 29;
		for (unsigned i = 0; i < 2; i++) {
			CHECK(pm_cfg(i, pm_ports[i].ss + 12, 4, 0xfb, power_word));
			CHECK(pm_cfg(i, pm_ports[i].ss + 8, 4, 0xe3ff0000, threshold));
			if (mode == 3) CHECK(pm_cfg(i, pm_ports[i].pc + 0x28, 2, 1u << 10, 1u << 10));
		}
		CHECK(pm_cfg(0, pm_ports[0].ss + 8, 4, 0xff00, cm << 8));
		/*
		 * ASPM L1.2/L1.1 (bits 2, 3), and the PCI-PM ones (bits 0 L1.2, 1 L1.1) where both ends have them: in D3hot the
		 * link is in L1 by PCI power management, not ASPM, and only these let that L1 go down to L1.2 (PCIe 5.0, 7.8.3.3;
		 * Linux aspm.c PCI_L1SS_CTL1_PCIPM_L1_*; pci-msm.c msm_pcie_config_l1ss writes all four). run74: with ASPM's
		 * alone every WoW doze ended ~80 ms after D3hot whatever the traffic, the firmware's "wow awake" timeout
		 * (amss20.bin: misc_consective_wow_awake_to_cnt, "Did not exit from l1ss within timeout"), and the DSP's
		 * power-down after 5 ms in L1ss never had L1ss to see.
		 */
		unsigned pcipm = pm_ports[0].sc & pm_ports[1].sc & (mode == 3 ? 3u : 2u);
		unsigned enable = (mode == 3 ? 12 : 8) | pcipm;
		CHECK(pm_cfg(0, pm_ports[0].ss + 8, 4, 15, enable));
		CHECK(pm_cfg(1, pm_ports[1].ss + 8, 4, 15, enable));
	}
	if (pm_ports[1].lc & (1u << 18)) CHECK(pm_cfg(1, pm_ports[1].pc + 0x10, 2, 1u << 8, 1u << 8));
	CHECK(pm_cfg(0, pm_ports[0].pc + 0x10, 2, 3, 2));
	CHECK(pm_cfg(1, pm_ports[1].pc + 0x10, 2, 3, 2));
	pm_active = mode;
	return 0;
}
#endif
