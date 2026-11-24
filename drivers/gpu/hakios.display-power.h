/*
 * ---- The display's hardware off while the panel sleeps (SM8450; POWER-COLLAPSE-PLAN.md 5e) -------------------------
 *
 * Included by hakios.backlight.cpp (it uses gDSI, gIntf, bl_say from there).
 *
 * Why: screen off only puts the panel to sleep (sleepPanel: DCS 0x28, 0x10). The display controller (MDSS), the DSI
 * controller, the DSI PHY and its PLL, and the display clock controller's PLLs stay powered and clocked, all on MMCX,
 * which needs CX at its level or above (Linux rpmhpd sm8450: mmcx_w_cx_parent). HLOS's sleep set dropped CX to 0 and
 * left MMCX at the boot loader's vote, and every run with a CX collapse froze minutes later (run5, 8, 9, 14). So the
 * collapse needs the display off as Linux has it while its screen is off: DSI link clocks off, the DSI PLL and PHY off,
 * MDP clocks off, the MDSS power domains off with flop retention (dispcc-sm8450.c: mdss_gdsc 0x9000 and mdss_int2_gdsc
 * 0xb000, HW_CTRL | RETAIN_FF_ENABLE), the display clock controller's PLLs off. Then MMCX can go to 0 in the sleep set
 * with CX (hakios.rpmh-arc.h asks hakios_display_off()).
 *
 * Power-up is the same backwards, before the panel is woken. The DSI PHY's and PLL's registers are not in a retained
 * domain (MMCX at 0 loses them; Linux reprograms both at each power-up, dsi_7nm_phy_enable and
 * dsi_7nm_pll_restore_state): they are written back, in Linux's order, from the values the boot loader left (read once,
 * at the first power-down), and the PLL is started and its lock waited for (dsi_pll_7nm_vco_prepare). The MDSS and the
 * DSI controller keep their flops; a list of their registers is read back against the values at the first power-down,
 * and so are the PHY's and PLL's after the restore: any difference is counted and the first ones logged.
 *
 * Nothing touches the MDSS or the DSI controller while it is down: hakios_display_hold() (MiniIOMobileFramebuffer's flip
 * and fb_dpu) fails then, dsi_packet() refuses, and the power-down waits for holders to let go (300 ms at most, or it
 * is not done). A step that fails on the way down puts back what was done and leaves the display on; one that fails
 * on the way up is logged and no power-down is tried again until the next boot.
 *
 * Boot-arg display_power_down=0: never (the display stays powered while the panel sleeps, and so no CX collapse).
 * kern.hakios.display_power_stats (read): counts, times, the last failure. kern.hakios.display_power_cycle (write
 * milliseconds, with the panel asleep): down, that long, up, the panel woken (its frames counted), asleep and down
 * again; read: the result of the last one.
 */

extern "C" int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));

#define DP_DISPCC_SM8450        0x0af00000UL
#define DP_DISPCC_SIZE          0x10000
#define DP_DSI0_SM8450          0x0ae94000UL    /* DSI0 controller 0x000, PHY common 0x400, lanes 0x600, PLL 0x900 */
#define DP_DSI0_SIZE            0x1000
#define DP_MDSS_SM8450          0x0ae00000UL
#define DP_MDSS_SIZE            0x38000

/* dispcc: GDSCR (SW_COLLAPSE 0, HW_CTRL 1, RETAIN_FF 11, PWR_ON 31); branch CBCR (CLK_ENABLE 0, CLK_OFF 31); RCG CMD
 * (ROOT_EN 1, ROOT_OFF 31); Lucid EVO PLL (MODE +0: OUTCTRL 0, RESET_N 2, LOCK_DET 31; OPMODE +4; USER_CTL +0x18: outputs
 * 0-2, FSM vote 25) */
#define DP_GDSC_MDSS            0x9000
#define DP_GDSC_INT2            0xb000
#define DP_GDSC_COLLAPSE        (1u << 0)
#define DP_GDSC_HW_CTRL         (1u << 1)
#define DP_GDSC_RETAIN_FF       (1u << 11)
#define DP_GDSC_PWR_ON          (1u << 31)
#define DP_PLL_MODE             0x00
#define DP_PLL_OPMODE           0x04
#define DP_PLL_USER_CTL         0x18

/* the DSI link's branches, then the MDP's and the register bus last (Linux: link clocks, then MDSS's) */
static const struct {
	uint32_t        offset;
	const char *    name;
} kDPLink[] = {
	{ 0x8038, "esc0" }, { 0x8004, "pclk0" }, { 0x802c, "byte0_intf" }, { 0x8028, "byte0" },
}, kDPCore[] = {
	{ 0x800c, "mdp" }, { 0x801c, "mdp_lut" }, { 0x8014, "rot" }, { 0x8024, "vsync" }, { 0xa004, "mdp1" },
	{ 0xa014, "mdp_lut1" }, { 0xa00c, "rot1" }, { 0xa01c, "vsync1" }, { 0xa020, "ahb1" },
	/* (the display RSC's and the non-GDSC part's: no Linux driver takes them, clk_disable_unused turns them off) */
	{ 0xc008, "rscc_vsync" }, { 0xc00c, "rscc_ahb" }, { 0xc004, "non_gdsc_ahb" }, { 0x80a4, "ahb" },
};
/* the RCGs by what feeds them: 0 disp_cc_pll0 (mdp: parent map 5), 1 disp_cc_pll1 (ahb, rot: 6), 2 the DSI PLL */
static const struct {
	uint32_t        offset;
	const char *    name;
	uint32_t        feed;
} kDPRCG[] = {
	{ 0x80ec, "mdp_clk_src", 0 }, { 0x8104, "rot_clk_src", 1 }, { 0x8324, "ahb_clk_src", 1 },
	{ 0x8134, "byte0_clk_src", 2 }, { 0x80bc, "pclk0_clk_src", 2 },
};
#define DP_N(a)                 (sizeof(a) / sizeof((a)[0]))
static const uint32_t kDPPLL[] = { 0x0000, 0x1000 };     /* disp_cc_pll0, disp_cc_pll1 */

/* the DSI 7nm/5nm PHY (dsi_phy_7nm.xml), from DSI0's base */
#define PHY(o)                  (0x400 + (o))
#define LANE(l, o)              (0x600 + 0x80 * (l) + (o))
#define PLL(o)                  (0x900 + (o))
#define PHY_CLK_CFG0            PHY(0x10)
#define PHY_CLK_CFG1            PHY(0x14)
#define PHY_GLBL_CTRL           PHY(0x18)
#define PHY_RBUF_CTRL           PHY(0x1c)
#define PHY_VREG_CTRL_0         PHY(0x20)
#define PHY_CTRL_0              PHY(0x24)
#define PHY_CTRL_2              PHY(0x2c)
#define PHY_CTRL_3              PHY(0x30)
#define PHY_LANE_CFG0           PHY(0x34)
#define PHY_LANE_CFG1           PHY(0x38)
#define PHY_PLL_CNTRL           PHY(0x3c)
#define PHY_LANE_CTRL0          PHY(0xa0)
#define PHY_TIMING_CTRL(n)      PHY(0xb4 + 4 * (n))
#define PHY_VREG_CTRL_1         PHY(0x110)
#define PHY_CTRL_4              PHY(0x114)
#define PHY_DIGTOP_SPARE4       PHY(0x128)
#define PHY_PHY_STATUS          PHY(0x140)
#define PHY_DIGTOP_SPARE10      PHY(0x1ac)
#define LANE_LPRX_CTRL          0x14
#define PLL_SYSTEM_MUXES        PLL(0x28)
#define PLL_OUTDIV_RATE         PLL(0x154)
#define PLL_COMMON_STATUS_ONE   PLL(0x1b0)

/*
 * What is written back at power-up, in this order (dsi_7nm_phy_enable, then dsi_7nm_pll_restore_state's
 * vco_set_rate: dsi_pll_commit, dsi_pll_config_hzindep_reg, dsi_pll_ssc_commit), each from the boot loader's value.
 */
static const uint16_t kDPPhyRestore[] = {
	PHY_CTRL_4, PHY_LANE_CFG0, PHY_LANE_CFG1, PHY_GLBL_CTRL, PHY_VREG_CTRL_0, PHY_VREG_CTRL_1,
	PHY(0x10c), PHY(0xec), PHY(0xf0), PHY(0x104), PHY(0x108), PHY(0xf4), PHY(0xf8), PHY(0xfc), PHY(0x100),
};
static const uint16_t kDPPhyAfter[] = {          /* after CTRL_0 0x7f */
	PHY_LANE_CTRL0, PHY_CTRL_2,
	PHY_TIMING_CTRL(0), PHY_TIMING_CTRL(1), PHY_TIMING_CTRL(2), PHY_TIMING_CTRL(3), PHY_TIMING_CTRL(4),
	PHY_TIMING_CTRL(5), PHY_TIMING_CTRL(6), PHY_TIMING_CTRL(7), PHY_TIMING_CTRL(8), PHY_TIMING_CTRL(9),
	PHY_TIMING_CTRL(10), PHY_TIMING_CTRL(11), PHY_TIMING_CTRL(12), PHY_TIMING_CTRL(13),
	PHY(0xa4), PHY(0xa8), PHY(0xac), PHY(0xb0),
};
static const uint8_t kDPLaneRegs[] = { 0x14, 0x10, 0x00, 0x04, 0x08, 0x18 };   /* LPRX_CTRL, PIN_SWAP, CFG0-2, TX_DCTRL */
static const uint16_t kDPPllRestore[] = {
	PLL_OUTDIV_RATE, PHY_CLK_CFG0,
	PLL(0xbc), PLL(0xe0), PLL(0xe4), PLL(0xe8), PLL(0xec), PLL(0x158), PLL(0x194), PLL(0x250), PLL(0x19c),
	PLL(0x258), PLL(0x240), PLL(0x18), PLL(0x04), PLL(0x10), PLL(0x20), PLL(0x24), PLL(0x44), PLL(0x68), PLL(0x78),
	PLL(0xa8), PLL(0xb8), PLL(0xc8), PLL(0x160), PLL(0x168), PLL(0x170), PLL(0x178), PLL(0x190), PLL(0x90), PLL(0x94),
	PLL(0x260),
	PLL(0x120), PLL(0x124), PLL(0x128), PLL(0x12c), PLL(0x130), PLL(0x134), PLL(0x150),
	/* (configuration Linux leaves at reset, the boot loader's own values back too) */
	PLL(0x00), PLL(0x08), PLL(0x0c), PLL(0x14), PLL(0x1c), PLL(0x30),
};
/* the PHY's state once running, compared with the boot loader's after each power-up */
static const uint16_t kDPPhyRunning[] = { PHY_CTRL_0, PHY_CTRL_3, PHY_CLK_CFG1, PHY_RBUF_CTRL, PHY_PLL_CNTRL, PLL_SYSTEM_MUXES,
    PHY_DIGTOP_SPARE10 };

/* retained across the power-down: read back against the first power-down's values */
static const uint32_t kDPMdssCheck[] = {
	0x05000, 0x05014, 0x05024, 0x05030, 0x07014,                    /* VIG0 size, address, stride, format; VIG1 address */
	0x16000, 0x16014, 0x160f4,                                      /* CTL0 layer mixer 0, top, interfaces */
	0x36284, 0x36288, 0x3628c, 0x362a0,                             /* INTF_1 tear check, vsync, height, start */
};
static const uint16_t kDPDsiCheck[] = { 0x004, 0x03c, 0x040, 0x044, 0x058, 0x05c, 0x084, 0x0b0, 0x0c4, 0x0cc, 0x2a8, 0x2ac };

#define DP_PHY_SAVED    (DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter) + DP_N(kDPPllRestore) + DP_N(kDPPhyRunning))

/*
 * The MDSS and the DSI controller do not keep their registers across the power-down (run16's first try: mdss_gdsc with
 * RETAIN_FF set off and on, and every register of the pipes, CTL0 and the DSI controller read 0). Linux reprograms
 * them at each resume; here each register window the sm8450 DPU has (dpu_8_1_sm8450.h, dpu_hw_catalog.c: sub-blocks;
 * MDSS's own top, the VBIF) is read at each power-down and, after the power-up, every word that differs is written
 * back, in this order, but for the registers that start, flush, reset or count something (dp_skip). Then the DSI
 * controller's, its enable last; then CTL0's flushes; then the auto-refresh.
 */
enum { DPW_MDSS, DPW_VBIF, DPW_TOP, DPW_SSPP, DPW_LM, DPW_DSC, DPW_M3D, DPW_PP, DPW_INTF, DPW_CTL, DPW_DSI };
static const struct {
	uint32_t        base;           /* from MDSS (0xae00000); the VBIF's from 0xaeb0000, the DSI's from DSI0 */
	uint16_t        len;
	uint8_t         kind;
} kDPWin[] = {
	{ 0x00000, 0x200, DPW_MDSS },
	/* the VBIF: what Linux's dpu_hw_vbif.c programs (clock force, QoS remap, write gather, in/out limits, memory types,
	 * the clients' QoS remaps). (Run16's third try read all 0x1040 of it; the hang then was the MDP top's, the window
	 * after: the last line said before a hang is lost.) Without it the pipes fetched through a VBIF at reset */
	{ 0x008, 0x8, DPW_VBIF }, { 0x020, 0x10, DPW_VBIF }, { 0x0ac, 0x2c, DPW_VBIF }, { 0x160, 0x8, DPW_VBIF },
	{ 0x550, 0x80, DPW_VBIF },
	/* the MDP top: only what Linux's dpu_hw_top.c and dpu_hw_interrupts.c write (interrupt enable, spare, the pipes'
	 * clock controls, split display, out ctl and vsync select, DCE select). Reading all 0x494 hangs: runs 3 and 4 both
	 * died at the window after the MDSS (VBIF or not); the last line said before a hang never reaches the log */
	/* (0x004 DISP_INTF_SEL: which kind of interface each INTF is, INTF_1 the DSI; the boot loader's; mdss_mdp_hwio.h) */
	/* the whole MDP top again, but for the words in kDPNever: the first save reads it a word at a time, each address
	 * said first, so that a hang names its register (the DSC took no flush with only Linux's handful of it back) */
	{ 0x01000, 0x494, DPW_TOP },
#define DPW_VIG(b)      { (b), 0x32c, DPW_SSPP }, { (b) + 0xa00, 0xa0, DPW_SSPP }, { (b) + 0x1a00, 0x100, DPW_SSPP }
	DPW_VIG(0x05000), DPW_VIG(0x07000), DPW_VIG(0x09000), DPW_VIG(0x0b000),
#undef DPW_VIG
	{ 0x25000, 0x32c, DPW_SSPP }, { 0x27000, 0x32c, DPW_SSPP }, { 0x29000, 0x32c, DPW_SSPP }, { 0x2b000, 0x32c, DPW_SSPP },
	{ 0x45000, 0x320, DPW_LM }, { 0x46000, 0x320, DPW_LM }, { 0x47000, 0x320, DPW_LM }, { 0x48000, 0x320, DPW_LM },
	{ 0x49000, 0x320, DPW_LM }, { 0x4a000, 0x320, DPW_LM },
#define DPW_DCE(b)      { (b), 0x4, DPW_DSC }, { (b) + 0x100, 0x9c, DPW_DSC }, { (b) + 0x200, 0x9c, DPW_DSC }, \
	                { (b) + 0xf00, 0x10, DPW_DSC }, { (b) + 0xf80, 0x10, DPW_DSC }
	DPW_DCE(0x81000), DPW_DCE(0x82000),
#undef DPW_DCE
	{ 0x4f000, 0x8, DPW_M3D }, { 0x50000, 0x8, DPW_M3D }, { 0x51000, 0x8, DPW_M3D }, { 0x66f00, 0x8, DPW_M3D },
	/* (the pingpongs: only their dither on DPU 8, sc7280_pp_sblk; the main block's length is 0) */
	{ 0x6a0e0, 0x20, DPW_PP }, { 0x6b0e0, 0x20, DPW_PP }, { 0x6c0e0, 0x20, DPW_PP }, { 0x6d0e0, 0x20, DPW_PP },
	{ 0x6e0e0, 0x20, DPW_PP }, { 0x6f0e0, 0x20, DPW_PP }, { 0x668e0, 0x20, DPW_PP }, { 0x66ce0, 0x20, DPW_PP },
	/* INTF_1 only, the panel's: INTF_0 and INTF_3 are DisplayPort's and INTF_2 DSI1's, unclocked here (run16's second
	 * try hung reading them at the first power-down) */
	{ 0x36000, 0x300, DPW_INTF },
	{ 0x16000, 0x204, DPW_CTL }, { 0x17000, 0x204, DPW_CTL }, { 0x18000, 0x204, DPW_CTL }, { 0x19000, 0x204, DPW_CTL },
	{ 0x1a000, 0x204, DPW_CTL }, { 0x1b000, 0x204, DPW_CTL },
	{ 0x000, 0x400, DPW_DSI },
};
#define DP_WIN_WORDS    8192
#define DP_MDSS_SIZE_ALL        0x90000
#define DP_VBIF_SM8450          0x0aeb0000UL
#define DP_VBIF_SIZE            0x2000
static uint32_t gDPWinVal[DP_WIN_WORDS];
static uint32_t gDPWinWords, gDPReplayed, gDPReplayedMax;
static volatile uint8_t * gDPVbif;
static bool gDPWinRead, gDPWinBad[96];        /* the windows read once with the fault-surviving read; the ones that fault */
static_assert(sizeof(kDPWin) / sizeof(kDPWin[0]) <= 96, "gDPWinBad: one per window");

/* registers not written back: what starts, flushes, resets, counts or reports something (o: within the window's block) */
static bool
dp_skip(uint8_t kind, uint32_t o)
{
	switch (kind) {
	case DPW_MDSS:
		return o == 0x0 || (o >= 0x10 && o < 0x20);                     /* HW_REV; interrupt status, clear */
	case DPW_TOP:
		/* version; the interrupts' status and clear registers */
		/* and the clock statuses, danger/safe status (dpu_hwio.h) */
		return o == 0x0 || o == 0xc || o == 0x14 || o == 0x18 || o == 0x20 || o == 0x24 || o == 0x2c || o == 0x2b0 ||
		       o == 0x2b8 || o == 0x2c0 || o == 0x2c8 || o == 0x360 || o == 0x364 || o == 0x3ac || o == 0x3b4 || o == 0x3bc ||
		       o == 0x3d4;
	case DPW_INTF:
		/* frame and line counts, interrupts, status, the tear counters; the auto-refresh is put back last */
		/* (0x284, the tear check's enable, is written after the rest) */
		return o == 0xac || o == 0xb0 || (o >= 0x1c0 && o < 0x1d0) || o == 0x26c || o == 0x284 || o == 0x298 || o == 0x2b0 ||
		       o == 0x2b4;
	case DPW_CTL:
		/* flush, start, software reset, prepare; the per-block flushes */
		return o == 0x18 || o == 0x1c || o == 0x30 || o == 0xd0 || (o >= 0x100 && o < 0x120) || o >= 0x200;
	case DPW_DSI:
		/* (msm dsi.xml) version, status, FIFO status, ack errors, the DMA/MDP/BTA/reset triggers, lane status (0xa4),
		 * timeout status (0xbc), interrupts (0x110, written after), reset (0x114), clock status (0x11c), PHY reset, the
		 * test pattern's FIFO port, triggers and reset, version 0x1f0. Written back: LANE_CTRL 0xa8, LP_TIMER 0xb4,
		 * CLKOUT_TIMING 0xc0, CLK_CTRL 0x118 (the controller's clocks: the pixel path; run16 skipped these by a
		 * register's mistake and the panel got no frames). DSI_CTRL (0x4) is written last */
		return o == 0x0 || o == 0x4 || o == 0x8 || o == 0xc || o == 0x68 || o == 0x90 || o == 0x94 || o == 0x98 ||
		       o == 0x9c || o == 0xa4 || o == 0xbc || o == 0x110 || o == 0x114 || o == 0x11c || o == 0x12c || o == 0x17c ||
		       o == 0x184 || o == 0x190 || o == 0x1ec || o == 0x1f0 || o == 0x1f4;
	default:
		return false;
	}
}

/* words of the kept windows never read nor written: reading them hangs the SoC (found by the first save's word-by-word
 * read of the MDP top; MDSS addresses) */
static const struct {
	uint32_t        from, to;               /* [from, to) */
} kDPNever[] = {
	/* the hole SM8450 has in the MDP top (DPU_MDP_PERIPH_0_REMOVED: MDP_PERIPH_TOP0 0x380 up to MDP_PERIPH_TOP0_END,
	 * CLK_CTRL3 0x3a8; Linux's snapshot reads the top around it): run16 hung reading 0x1380, then 0x13a0. And the
	 * other watchdog vsync timers (MDP_WD_TIMER_2..4), not used: the panel's vsync is its TE pin */
	{ 0x1380, 0x13a8 }, { 0x1420, 0x144c },
};

static bool
dp_never(uint8_t kind, uint32_t at)
{
	if (kind == DPW_VBIF || kind == DPW_DSI) {
		return false;
	}
	for (uint32_t i = 0; i < sizeof(kDPNever) / sizeof(kDPNever[0]); i++) {
		if (at >= kDPNever[i].from && at < kDPNever[i].to) {
			return true;
		}
	}
	return false;
}

/* gDPOn: boot-arg display_power_down (hakios.backlight.cpp, where the catalogue finds it) */
static volatile uint8_t * gDPCC;                /* dispcc */
static volatile uint8_t * gDP;                  /* DSI0 controller, PHY, PLL */
static volatile uint8_t * gDPMdss;
static volatile uint8_t * gDPGcc;               /* GCC 0x137000: the display's bus branches */
#define DP_GCC_DISP_PAGE        0x00137000UL
static const struct {
	uint32_t        offset;
	const char *    name;
} kDPGccAxi[] = { { 0xc, "gcc_disp_hf_axi" }, { 0x14, "gcc_disp_sf_axi" } };       /* (GCC 0x3700c, 0x37014) */
static uint32_t gDPAxiWas[2];
static bool gDPReady, gDPBroken, gDPSaved, gDPDead;
static volatile uint32_t gDPState;              /* 0 on, 1 going down or down, 2 coming up */
static volatile bool gDPOff;                    /* all of it off: what the CX collapse asks */
static volatile int32_t gDPUsers;
static uint32_t gDPPhy[DP_PHY_SAVED], gDPLane[5][sizeof(kDPLaneRegs)], gDPCtrl0;
static uint32_t gDPMdssWas[DP_N(kDPMdssCheck)], gDPDsiWas[DP_N(kDPDsiCheck)];
static uint32_t gDPLinkWas[DP_N(kDPLink)], gDPCoreWas[DP_N(kDPCore)], gDPPllMode[2], gDPPllUser[2];
static uint32_t gDPGdscWas[2], gDPAutorefresh;
static bool gDPPllOff[2], gDPGdscOff[2];
static struct {
	uint32_t        downs, ups, refused, up_failed, mdss_diff, dsi_diff, phy_diff, lock_us_max, down_us_max, up_us_max;
	uint32_t        last_lock_us, cycles, cycle_frames, cycle_failed;
	char            why[160], cycle[200];
} gDPStat;

static inline uint32_t
dp_rd(volatile uint8_t * b, uint32_t o)
{
	return *(volatile uint32_t *)(b + o);
}

static inline void
dp_wr(volatile uint8_t * b, uint32_t o, uint32_t v)
{
	*(volatile uint32_t *)(b + o) = v;
	__builtin_arm_dsb(0xf);
}

static uint32_t
dp_us(uint64_t since)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - since, &ns);
	return (uint32_t) (ns / NSEC_PER_USEC);
}

/* wait for (reg & mask) == want, up to us microseconds */
static bool
dp_wait(volatile uint8_t * b, uint32_t o, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t i = 0; i <= us; i++) {
		if ((dp_rd(b, o) & mask) == want) {
			return true;
		}
		IODelay(1);
	}
	return false;
}

static void
dp_why(const char * format, ...) __printflike(1, 2);

static void
dp_why(const char * format, ...)
{
	va_list list;

	va_start(list, format);
	vsnprintf(gDPStat.why, sizeof(gDPStat.why), format, list);
	va_end(list);
	bl_say("display power: %s\n", gDPStat.why);
}

/* another power-down worth trying after one was refused (a holder, the controller busy): at most a few per sleep */
static inline bool
dp_retry(void)
{
	return gDPOn && !gDPBroken && !gDPOff;
}

/* MDSS and DSI controller registers usable now (not down, not going down) */
static inline bool
dp_regs_on(void)
{
	return __atomic_load_n(&gDPState, __ATOMIC_ACQUIRE) == 0;
}

extern "C" bool hakios_display_hold(void);
extern "C" void hakios_display_release(void);
extern "C" bool hakios_display_off(void);

/* MiniIOMobileFramebuffer: the MDSS's registers may be used until hakios_display_release() (false: they may not) */
bool
hakios_display_hold(void)
{
	if (!dp_regs_on()) {
		return false;
	}
	__atomic_add_fetch(&gDPUsers, 1, __ATOMIC_SEQ_CST);
	if (!dp_regs_on()) {
		__atomic_sub_fetch(&gDPUsers, 1, __ATOMIC_SEQ_CST);
		return false;
	}
	return true;
}

void
hakios_display_release(void)
{
	__atomic_sub_fetch(&gDPUsers, 1, __ATOMIC_SEQ_CST);
}

/* hakios.rpmh-arc.h, with interrupts off on the last core up: the whole display off (MMCX may go to 0 with CX) */
bool
hakios_display_off(void)
{
	return gDPOff;
}

static bool
dp_map(void)
{
	if (gDPReady) {
		return true;
	}
	gDPCC = (volatile uint8_t *) ml_io_map(DP_DISPCC_SM8450, DP_DISPCC_SIZE);
	gDP = (volatile uint8_t *) ml_io_map(DP_DSI0_SM8450, DP_DSI0_SIZE);
	gDPMdss = (volatile uint8_t *) ml_io_map(DP_MDSS_SM8450, DP_MDSS_SIZE_ALL);
	gDPGcc = (volatile uint8_t *) ml_io_map(DP_GCC_DISP_PAGE, 0x1000);
	gDPVbif = (volatile uint8_t *) ml_io_map(DP_VBIF_SM8450, DP_VBIF_SIZE);
	if (gDPCC == NULL || gDP == NULL || gDPMdss == NULL || gDPGcc == NULL || gDPVbif == NULL || gIntf == NULL ||
	    hakios_mmio_try_read32 == NULL) {
		return false;
	}
	gDPWinWords = 0;
	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		gDPWinWords += kDPWin[w].len / 4;
	}
	if (gDPWinWords > DP_WIN_WORDS) {
		bl_say("display power: %u words of registers to keep, room for %u\n", gDPWinWords, (uint32_t) DP_WIN_WORDS);
		return false;
	}
	unsigned v;
	if (hakios_mmio_try_read32(gDPCC + DP_GDSC_MDSS, &v) != 0 || hakios_mmio_try_read32(gDP + PHY(0), &v) != 0 ||
	    hakios_mmio_try_read32(gDPGcc + 0xc, &v) != 0) {
		return false;
	}
	gDPReady = true;
	return true;
}

static void dp_windows(bool save, uint8_t from, uint8_t to);

/* the boot loader's state, once (the first power-down), and the retained registers' values at each */
static void
dp_save(void)
{
	uint32_t n = 0;

	if (!gDPSaved) {
		for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++) {
			gDPPhy[n++] = dp_rd(gDP, kDPPhyRestore[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++) {
			gDPPhy[n++] = dp_rd(gDP, kDPPhyAfter[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++) {
			gDPPhy[n++] = dp_rd(gDP, kDPPllRestore[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPhyRunning); i++) {
			gDPPhy[n++] = dp_rd(gDP, kDPPhyRunning[i]);
		}
		for (uint32_t l = 0; l < 5; l++) {
			for (uint32_t i = 0; i < sizeof(kDPLaneRegs); i++) {
				gDPLane[l][i] = dp_rd(gDP, LANE(l, kDPLaneRegs[i]));
			}
		}
		gDPCtrl0 = dp_rd(gDP, PHY_CTRL_0);
		gDPSaved = true;
		bl_say("display power: the boot loader's state: PHY ctrl_0 %#x clk_cfg0 %#x clk_cfg1 %#x pll_cntrl %#x vreg %#x/%#x, PLL "
		    "dec %#x frac %#x/%#x/%#x outdiv_rate %#x; GDSCs %#x %#x; PLLs mode %#x %#x user %#x %#x\n", gDPCtrl0,
		    dp_rd(gDP, PHY_CLK_CFG0), dp_rd(gDP, PHY_CLK_CFG1), dp_rd(gDP, PHY_PLL_CNTRL), dp_rd(gDP, PHY_VREG_CTRL_0),
		    dp_rd(gDP, PHY_VREG_CTRL_1), dp_rd(gDP, PLL(0xe0)), dp_rd(gDP, PLL(0xe4)), dp_rd(gDP, PLL(0xe8)),
		    dp_rd(gDP, PLL(0xec)), dp_rd(gDP, PLL_OUTDIV_RATE), dp_rd(gDPCC, DP_GDSC_MDSS), dp_rd(gDPCC, DP_GDSC_INT2),
		    dp_rd(gDPCC, kDPPLL[0] + DP_PLL_MODE), dp_rd(gDPCC, kDPPLL[1] + DP_PLL_MODE),
		    dp_rd(gDPCC, kDPPLL[0] + DP_PLL_USER_CTL), dp_rd(gDPCC, kDPPLL[1] + DP_PLL_USER_CTL));
	}
	/* (the retained ones as they are now: the flips change the pipes' addresses) */
	for (uint32_t i = 0; i < DP_N(kDPMdssCheck); i++) {
		gDPMdssWas[i] = dp_rd(gDPMdss, kDPMdssCheck[i]);
	}
	for (uint32_t i = 0; i < DP_N(kDPDsiCheck); i++) {
		gDPDsiWas[i] = dp_rd(gDP, kDPDsiCheck[i]);
	}
	dp_windows(true, 0, 0);         /* every window, to be written back at the power-up */
}

static inline volatile uint8_t *
dp_win_base(uint8_t kind)
{
	return kind == DPW_VBIF ? gDPVbif : kind == DPW_DSI ? gDP : gDPMdss;
}

/* every window's words read (save) or the ones that differ written back (from..to: kinds, in the table's order) */
static void
dp_windows(bool save, uint8_t from, uint8_t to)
{
	uint32_t n = 0;

	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		volatile uint8_t * b = dp_win_base(kDPWin[w].kind);
		uint32_t block = kDPWin[w].base & ~0xfffu;      /* (dp_skip's offsets are from the block's base) */

		if (save && !gDPWinRead) {
			/* the first time: each word through the read that survives a fault (a window that faults is not kept) */
			for (uint32_t o = 0; o < kDPWin[w].len; o += 4) {
				unsigned v;

				if (dp_never(kDPWin[w].kind, kDPWin[w].base + o)) {
					continue;
				}
				if (hakios_mmio_try_read32(b + kDPWin[w].base + o, &v) != 0) {
					gDPWinBad[w] = true;
				}
			}
			if (gDPWinBad[w]) {
				bl_say("display power: window %u (kind %u at %#x) faults on reading: not kept\n", w, kDPWin[w].kind,
				    kDPWin[w].base);
			}
		}
		if (gDPWinBad[w]) {
			n += kDPWin[w].len / 4;
			continue;
		}
		for (uint32_t o = 0; o < kDPWin[w].len; o += 4, n++) {
			uint32_t at = kDPWin[w].base + o;

			if (dp_never(kDPWin[w].kind, at)) {
				gDPWinVal[n] = 0;
				continue;
			}
			if (save) {
				gDPWinVal[n] = *(volatile uint32_t *)(b + at);
			} else if (kDPWin[w].kind >= from && kDPWin[w].kind <= to &&
			    !dp_skip(kDPWin[w].kind, kDPWin[w].kind == DPW_DSI || kDPWin[w].kind == DPW_VBIF ? at : at - block) &&
			    *(volatile uint32_t *)(b + at) != gDPWinVal[n]) {
				*(volatile uint32_t *)(b + at) = gDPWinVal[n];
				gDPReplayed++;
			}
		}
	}
	gDPWinRead = gDPWinRead || save;
	__builtin_arm_dsb(0xf);
}

/* the saved value of a window word (kind, address as in kDPWin); UINT32_MAX if not kept */
static uint32_t
dp_win_saved(uint8_t kind, uint32_t at)
{
	uint32_t n = 0;

	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		if (kDPWin[w].kind == kind && at >= kDPWin[w].base && at < kDPWin[w].base + kDPWin[w].len) {
			return gDPWinVal[n + (at - kDPWin[w].base) / 4];
		}
		n += kDPWin[w].len / 4;
	}
	return UINT32_MAX;
}

/* the saved PHY value of register o (UINT32_MAX: not saved) */
static uint32_t
dp_phy_saved(uint32_t o)
{
	uint32_t n = 0;

	for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++, n++) {
		if (kDPPhyRestore[i] == o) {
			return gDPPhy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++, n++) {
		if (kDPPhyAfter[i] == o) {
			return gDPPhy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++, n++) {
		if (kDPPllRestore[i] == o) {
			return gDPPhy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPhyRunning); i++, n++) {
		if (kDPPhyRunning[i] == o) {
			return gDPPhy[n];
		}
	}
	return UINT32_MAX;
}

/* a branch off (bit 0 cleared), and whether it says stopped within 200 us */
static bool
dp_branch_off(uint32_t o, uint32_t * was)
{
	*was = dp_rd(gDPCC, o);
	if (!(*was & 1u)) {
		return true;
	}
	dp_wr(gDPCC, o, *was & ~1u);
	return dp_wait(gDPCC, o, 1u << 31, 1u << 31, 200);
}

static bool
dp_branch_on(uint32_t o, uint32_t was)
{
	if (!(was & 1u)) {
		return true;
	}
	dp_wr(gDPCC, o, was);
	return (was & (1u << 31)) != 0 || dp_wait(gDPCC, o, 1u << 31, 0, 500);
}

/* the DSI PLL and PHY off (dsi_pll_7nm_vco_unprepare, dsi_7nm_phy_disable) */
static void
dp_phy_off(void)
{
	dp_wr(gDP, PHY_CLK_CFG1, dp_rd(gDP, PHY_CLK_CFG1) & ~(1u << 5));
	dp_wr(gDP, PHY_PLL_CNTRL, 0);
	dp_wr(gDP, PHY_RBUF_CTRL, 0);
	dp_wr(gDP, PLL_SYSTEM_MUXES, 0);
	dp_wr(gDP, PHY_CTRL_0, dp_rd(gDP, PHY_CTRL_0) & ~(1u << 5));
	IODelay(1);
	dp_wr(gDP, LANE(0, LANE_LPRX_CTRL), 0);
	dp_wr(gDP, PHY_DIGTOP_SPARE10, 0);
	IODelay(2);
	dp_wr(gDP, PHY_CTRL_0, dp_rd(gDP, PHY_CTRL_0) & ~0x1fu);
	dp_wr(gDP, PHY_LANE_CTRL0, 0);
	dp_wr(gDP, PHY_CTRL_0, 0);
}

/* the PHY and PLL from the boot loader's values, the PLL started and locked (false: no reference or no lock) */
static bool
dp_phy_on(void)
{
	uint32_t n = 0;
	uint64_t t;

	/* dsi_7nm_phy_enable */
	dp_wr(gDP, PHY_DIGTOP_SPARE10, 1);
	IODelay(500);
	if (!dp_wait(gDP, PHY_PHY_STATUS, 1u, 1u, 1000)) {
		dp_why("the DSI PHY's reference not ready (phy_status %#x)", dp_rd(gDP, PHY_PHY_STATUS));
		return false;
	}
	dp_wr(gDP, PHY_CTRL_0, (1u << 6) | (1u << 5));
	dp_wr(gDP, PHY_PLL_CNTRL, 0);
	dp_wr(gDP, PHY_RBUF_CTRL, 0);
	for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++) {
		dp_wr(gDP, kDPPhyRestore[i], gDPPhy[n++]);
	}
	dp_wr(gDP, PHY_CTRL_3, 0);
	dp_wr(gDP, PHY_CTRL_0, 0x7f);
	for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++) {
		dp_wr(gDP, kDPPhyAfter[i], gDPPhy[n++]);
	}
	dp_wr(gDP, PHY_CLK_CFG1, dp_phy_saved(PHY_CLK_CFG1) & 0xcu);      /* the PLL's source (dsi_7nm_set_usecase) */
	for (uint32_t l = 0; l < 5; l++) {
		for (uint32_t i = 0; i < sizeof(kDPLaneRegs); i++) {
			dp_wr(gDP, LANE(l, kDPLaneRegs[i]), gDPLane[l][i]);
		}
	}
	/* dsi_7nm_pll_restore_state: dividers, the mux, then every register vco_set_rate writes */
	for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++) {
		dp_wr(gDP, kDPPllRestore[i], gDPPhy[n++]);
	}
	dp_wr(gDP, PHY_CLK_CFG1, (dp_rd(gDP, PHY_CLK_CFG1) & ~3u) | (dp_phy_saved(PHY_CLK_CFG1) & 3u));
	/* dsi_pll_7nm_vco_prepare: bias, start, lock, the digital reset, the global clock */
	dp_wr(gDP, PHY_CTRL_0, dp_rd(gDP, PHY_CTRL_0) | (1u << 5));
	dp_wr(gDP, PLL_SYSTEM_MUXES, 0xc0);
	IODelay(1);
	t = mach_absolute_time();
	dp_wr(gDP, PHY_PLL_CNTRL, 1);
	if (!dp_wait(gDP, PLL_COMMON_STATUS_ONE, 1u, 1u, 5000)) {
		dp_why("the DSI PLL did not lock in 5 ms (status %#x)", dp_rd(gDP, PLL_COMMON_STATUS_ONE));
		return false;
	}
	gDPStat.last_lock_us = dp_us(t);
	gDPStat.lock_us_max = MAX(gDPStat.lock_us_max, gDPStat.last_lock_us);
	dp_wr(gDP, PHY_DIGTOP_SPARE4, 1);
	dp_wr(gDP, PHY_DIGTOP_SPARE4, 0);
	dp_wr(gDP, PHY_CTRL_3, 0x04);
	dp_wr(gDP, PHY_CLK_CFG1, dp_rd(gDP, PHY_CLK_CFG1) | (1u << 5) | (1u << 4));
	dp_wr(gDP, PHY_RBUF_CTRL, dp_phy_saved(PHY_RBUF_CTRL));       /* (the resync FIFO: the boot loader has it on) */
	return true;
}

/* a GDSC off with its flops kept (gdsc_disable: hardware control off first); false: it stayed on (put back) */
static bool
dp_gdsc_off(uint32_t o, uint32_t * was)
{
	uint32_t v = *was = dp_rd(gDPCC, o);

	if (!(v & DP_GDSC_PWR_ON)) {
		return true;
	}
	if (v & DP_GDSC_HW_CTRL) {
		v &= ~DP_GDSC_HW_CTRL;
		dp_wr(gDPCC, o, v);
		IODelay(1);
		if (!dp_wait(gDPCC, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 500)) {
			dp_wr(gDPCC, o, *was);
			return false;
		}
	}
	v |= DP_GDSC_RETAIN_FF;
	dp_wr(gDPCC, o, v);
	dp_wr(gDPCC, o, v | DP_GDSC_COLLAPSE);
	if (!dp_wait(gDPCC, o, DP_GDSC_PWR_ON, 0, 500)) {
		dp_wr(gDPCC, o, v);
		dp_wait(gDPCC, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 2000);
		dp_wr(gDPCC, o, *was);
		return false;
	}
	return true;
}

static bool
dp_gdsc_on(uint32_t o, uint32_t was)
{
	uint32_t v = dp_rd(gDPCC, o);

	dp_wr(gDPCC, o, v & ~DP_GDSC_COLLAPSE);
	if (!dp_wait(gDPCC, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 2000)) {
		return false;
	}
	IODelay(1);                     /* (gdsc_enable: no clock within 400 ns of the memories' power) */
	if (was & DP_GDSC_HW_CTRL) {
		dp_wr(gDPCC, o, (v & ~DP_GDSC_COLLAPSE) | DP_GDSC_HW_CTRL);
		IODelay(1);
	}
	return true;
}

/* a dispcc PLL off as clk_alpha_pll_reset_lucid_evo_ops does (not one in FSM mode or not running) */
static void
dp_pll_off(uint32_t i)
{
	uint32_t p = kDPPLL[i];

	gDPPllMode[i] = dp_rd(gDPCC, p + DP_PLL_MODE);
	gDPPllUser[i] = dp_rd(gDPCC, p + DP_PLL_USER_CTL);
	gDPPllOff[i] = false;
	if ((gDPPllUser[i] & (1u << 25)) || !(dp_rd(gDPCC, p + DP_PLL_OPMODE) & 1u) || !(gDPPllMode[i] & 1u)) {
		return;
	}
	dp_wr(gDPCC, p + DP_PLL_MODE, gDPPllMode[i] & ~1u);
	dp_wr(gDPCC, p + DP_PLL_USER_CTL, gDPPllUser[i] & ~7u);
	dp_wr(gDPCC, p + DP_PLL_OPMODE, 0);
	dp_wr(gDPCC, p + DP_PLL_MODE, gDPPllMode[i] & ~5u);
	gDPPllOff[i] = true;
}

static bool
dp_pll_on(uint32_t i)
{
	uint32_t p = kDPPLL[i];

	if (!gDPPllOff[i]) {
		return true;
	}
	gDPPllOff[i] = false;
	dp_wr(gDPCC, p + DP_PLL_MODE, (gDPPllMode[i] & ~1u) | (1u << 2));
	dp_wr(gDPCC, p + DP_PLL_OPMODE, 1);
	if (!dp_wait(gDPCC, p + DP_PLL_MODE, 1u << 31, 1u << 31, 1000)) {
		return false;
	}
	dp_wr(gDPCC, p + DP_PLL_USER_CTL, gDPPllUser[i]);
	dp_wr(gDPCC, p + DP_PLL_MODE, gDPPllMode[i]);
	return true;
}

static uint32_t
dp_compare(const char * what, volatile uint8_t * b, const uint32_t * offsets, const uint16_t * offsets16, uint32_t n,
    const uint32_t * was)
{
	uint32_t diff = 0;
	char line[200];
	int at = 0;

	line[0] = 0;
	for (uint32_t i = 0; i < n; i++) {
		uint32_t o = offsets != NULL ? offsets[i] : offsets16[i], v = dp_rd(b, o);

		if (v != was[i]) {
			diff++;
			if (at < (int) sizeof(line) - 40) {
				at += snprintf(line + at, sizeof(line) - (size_t) at, " %#x:%#x(was %#x)", o, v, was[i]);
			}
		}
	}
	if (diff != 0) {
		bl_say("display power: %u of %s's registers not as before:%s\n", diff, what, line);
	}
	return diff;
}

static void dp_up(void);

/*
 * The panel asleep (the backlight's work loop, after sleepPanel): the display off. False: not done (the reason in
 * gDPStat.why), everything as it was.
 */
static bool
dp_down(void)
{
	uint64_t began = mach_absolute_time();
	uint32_t i, g;

	if (!gDPOn || gDPBroken || gDPOff) {
		return gDPOff;
	}
	if (!dp_map()) {
		dp_why("the display's registers not mapped or not readable: never");
		gDPBroken = true;
		return false;
	}
	__atomic_store_n(&gDPState, 1, __ATOMIC_SEQ_CST);
	for (i = 0; i < 300 && __atomic_load_n(&gDPUsers, __ATOMIC_SEQ_CST) != 0; i++) {
		IOSleep(1);
	}
	if (__atomic_load_n(&gDPUsers, __ATOMIC_SEQ_CST) != 0) {
		__atomic_store_n(&gDPState, 0, __ATOMIC_RELEASE);
		gDPStat.refused++;
		dp_why("the framebuffer held the display for 300 ms");
		return false;
	}
	/* (the DSI lock: no command in flight; dsi_packet() refuses from now on) */
	IOLockLock(gDSILock);
	IOLockUnlock(gDSILock);
	if (dp_rd(gDP, 0x008) & (DSI_STATUS_DMA_BUSY | DSI_STATUS_MDP_BUSY)) {
		__atomic_store_n(&gDPState, 0, __ATOMIC_RELEASE);
		gDPStat.refused++;
		dp_why("the DSI controller busy (status %#x)", dp_rd(gDP, 0x008));
		return false;
	}
	dp_save();
	gDPAutorefresh = intf_rd(INTF_TEAR_AUTOREFRESH_CONFIG);
	if (gDPAutorefresh & (1u << 31)) {
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, gDPAutorefresh & ~(1u << 31));
	}
	/* the DSI link: its clocks, the PLL and the PHY */
	for (i = 0; i < DP_N(kDPLink); i++) {
		if (!dp_branch_off(kDPLink[i].offset, &gDPLinkWas[i])) {
			bl_say("display power: dispcc %s (%#x) not stopped: %#x\n", kDPLink[i].name, kDPLink[i].offset,
			    dp_rd(gDPCC, kDPLink[i].offset));
		}
	}
	for (i = 0; i < DP_N(kDPRCG); i++) {
		if (kDPRCG[i].feed == 2 && !(dp_rd(gDPCC, kDPRCG[i].offset) & (1u << 31))) {
			bl_say("display power: dispcc %s still running (%#x) as the DSI PLL goes off\n", kDPRCG[i].name,
			    dp_rd(gDPCC, kDPRCG[i].offset));
		}
	}
	dp_phy_off();
	/* the MDP's clocks and the register bus, then the domains */
	for (i = 0; i < DP_N(kDPCore); i++) {
		if (!dp_branch_off(kDPCore[i].offset, &gDPCoreWas[i])) {
			bl_say("display power: dispcc %s (%#x) not stopped: %#x\n", kDPCore[i].name, kDPCore[i].offset,
			    dp_rd(gDPCC, kDPCore[i].offset));
		}
	}
	/* the MDSS's ports into the interconnect stopped before its domain goes (msm_mdss_disable: its clocks, these with
	 * them, then the GDSC); a domain cut under a clocked bus port can hang the interconnect */
	for (i = 0; i < DP_N(kDPGccAxi); i++) {
		gDPAxiWas[i] = dp_rd(gDPGcc, kDPGccAxi[i].offset);
		if (gDPAxiWas[i] & 1u) {
			dp_wr(gDPGcc, kDPGccAxi[i].offset, gDPAxiWas[i] & ~1u);
		}
	}
	for (g = 0; g < 2; g++) {
		uint32_t o = g == 0 ? DP_GDSC_INT2 : DP_GDSC_MDSS;

		gDPGdscOff[g] = false;
		if (!dp_gdsc_off(o, &gDPGdscWas[g])) {
			dp_why("dispcc GDSC %#x stayed on (%#x): the display put back on", o, dp_rd(gDPCC, o));
			gDPStat.refused++;
			goto back;
		}
		gDPGdscOff[g] = true;
	}
	/* the clock controller's PLLs, once nothing they feed runs */
	for (i = 0; i < DP_N(kDPRCG); i++) {
		/* (one fed by that PLL now: CFG_RCGR, CMD + 4, bits 10:8 the source; pll0 is 1 in map 5, pll1 4 or 6 in map 6) */
		uint32_t src = (dp_rd(gDPCC, kDPRCG[i].offset + 4) >> 8) & 7u;
		bool fed = kDPRCG[i].feed == 0 ? src == 1 : kDPRCG[i].feed == 1 ? src == 4 || src == 6 : false;

		if (fed && !(dp_rd(gDPCC, kDPRCG[i].offset) & (1u << 31))) {
			dp_why("dispcc %s still running (%#x): its PLL cannot go off, the display put back on", kDPRCG[i].name,
			    dp_rd(gDPCC, kDPRCG[i].offset));
			gDPStat.refused++;
			goto back;
		}
	}
	for (i = 0; i < 2; i++) {
		dp_pll_off(i);
	}
	gDPOff = true;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	gDPStat.downs++;
	gDPStat.down_us_max = MAX(gDPStat.down_us_max, dp_us(began));
	if (gDPStat.downs <= 3 || (gDPStat.downs % 50) == 0) {
		bl_say("display power: off in %u us (%u times; PLLs off %u %u; GDSCs now %#x %#x)\n", dp_us(began), gDPStat.downs,
		    gDPPllOff[0], gDPPllOff[1], dp_rd(gDPCC, DP_GDSC_MDSS), dp_rd(gDPCC, DP_GDSC_INT2));
	}
	return true;
back:
	dp_up();                /* (gDPState 1: the whole way back, for what was done) */
	return false;
}

/* Before the panel is woken (or anything else uses the MDSS): the display on again. */
static void
dp_up(void)
{
	uint64_t began = mach_absolute_time();
	bool ok = true, regs = true;    /* regs: the domains and the register bus on (else nothing reads them: it would hang) */
	uint32_t i, g;

	if (__atomic_load_n(&gDPState, __ATOMIC_ACQUIRE) == 0 || gDPDead) {
		return;
	}
	gDPOff = false;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	__atomic_store_n(&gDPState, 2, __ATOMIC_SEQ_CST);
	for (i = 0; i < 2; i++) {
		if (!dp_pll_on(i)) {
			dp_why("dispcc PLL %u did not lock (mode %#x)", i, dp_rd(gDPCC, kDPPLL[i] + DP_PLL_MODE));
			ok = false;
		}
	}
	for (g = 2; g-- > 0;) {
		uint32_t o = g == 0 ? DP_GDSC_INT2 : DP_GDSC_MDSS;

		if (gDPGdscOff[g]) {
			gDPGdscOff[g] = false;
			if (!dp_gdsc_on(o, gDPGdscWas[g])) {
				dp_why("dispcc GDSC %#x did not come on (%#x)", o, dp_rd(gDPCC, o));
				ok = regs = false;
			}
		}
	}
	for (i = DP_N(kDPGccAxi); i-- > 0;) {
		if (gDPAxiWas[i] & 1u) {
			dp_wr(gDPGcc, kDPGccAxi[i].offset, gDPAxiWas[i]);
		}
	}
	for (i = DP_N(kDPCore); i-- > 0;) {
		if (!dp_branch_on(kDPCore[i].offset, gDPCoreWas[i])) {
			dp_why("dispcc %s did not start (%#x)", kDPCore[i].name, dp_rd(gDPCC, kDPCore[i].offset));
			ok = false;
			regs = regs && kDPCore[i].offset != 0x80a4 && kDPCore[i].offset != 0xa020;
		}
	}
	if (!regs) {
		/* the display's registers are not to be touched again: the panel stays dark until the next boot */
		gDPDead = gDPBroken = true;
		gDPStat.up_failed++;
		bl_say("display power: the display's domains or register bus did NOT come back; its registers are not touched "
		    "again until the next boot (%s)\n", gDPStat.why);
		return;
	}
	gDPReplayed = 0;
	dp_windows(false, DPW_MDSS, DPW_CTL);
	if (gDPSaved) {
		if (!dp_phy_on()) {
			ok = false;
		}
		gDPStat.phy_diff += dp_compare("the DSI PHY", gDP, NULL, kDPPhyRunning, DP_N(kDPPhyRunning),
		    &gDPPhy[DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter) + DP_N(kDPPllRestore)]);
		gDPStat.phy_diff += dp_compare("the DSI PLL", gDP, NULL, kDPPllRestore, DP_N(kDPPllRestore),
		    &gDPPhy[DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter)]);
	}
	for (i = DP_N(kDPLink); i-- > 0;) {
		if (!dp_branch_on(kDPLink[i].offset, gDPLinkWas[i])) {
			dp_why("dispcc %s did not start (%#x)", kDPLink[i].name, dp_rd(gDPCC, kDPLink[i].offset));
			ok = false;
		}
	}
	/* the DSI controller's registers, its interrupt enables, then the controller on (dsi_ctrl_config, then enable) */
	dp_windows(false, DPW_DSI, DPW_DSI);
	dp_wr(gDP, DSI_INT_CTRL, dp_win_saved(DPW_DSI, DSI_INT_CTRL) & DSI_INT_ENABLES);
	dp_wr(gDP, DSI_CTRL, dp_win_saved(DPW_DSI, DSI_CTRL));
	/* CTL0: its blocks' flushes from what it drives, then the whole flush (dpu_hw_ctl: pipes 0-2, 11, 12, 18, 24, 25;
	 * mixers 6-10, 20; CTL 17; DSC 22; merge_3d 23; interface 31), taken at the next frame */
	/* the blocks' own flushes, in dpu_hw_ctl_trigger_flush_v1's order (MERGE_3D, INTF, DSC), then CTL_FLUSH. CTL_DSC_FLUSH
	 * is 0x104 (run16 wrote 0x10c, not a flush register: CTL_FLUSH's DSC bit 22 then named no DSC and was never taken,
	 * the frame never started) */
	dp_wr(gDPMdss, 0x16100, dp_win_saved(DPW_CTL, 0x160e4));       /* MERGE_3D_FLUSH: the active ones */
	dp_wr(gDPMdss, 0x16110, dp_win_saved(DPW_CTL, 0x160f4));       /* INTF_FLUSH */
	dp_wr(gDPMdss, 0x16104, dp_win_saved(DPW_CTL, 0x160e8));       /* DSC_FLUSH */
	/* only what is active, as Linux builds it (dpu_hw_ctl): a flush bit for a block outside the pipeline is never taken
	 * and held CTL0 (run16: bits 22 and 23 left pending, MERGE_3D not active, no frame). The fetching pipes from
	 * FETCH_PIPE_ACTIVE (0xfc: VIG0-3 bits 16-19, DMA0-3 bits 0-3), the mixers with a layer (CTL_LAYER 0x00-0x14),
	 * DSC and MERGE_3D when active, CTL and the interface always */
	uint32_t flush = 1u << 17 | 1u << 31, fetch = dp_win_saved(DPW_CTL, 0x160fc);
	static const uint8_t kFetchFlush[] = { 0, 1, 2, 18 }, kDmaFlush[] = { 11, 12, 24, 25 }, kLmFlush[] = { 6, 7, 8, 9, 10, 20 };
	/* (dpu_hw_ctl fetch_tbl: VIG0-3 are FETCH_PIPE_ACTIVE bits 16-19, DMA0-3 bits 0-3; the panel's pipes are VIG0 and
	 * VIG1, fetch 0x30000, the flips' flush 0x20003. Run16 had these the wrong way round: VIG0/1 never flushed, their
	 * active addresses 0, and the first frame read physical 0 (the SMMU does not translate the display): a reset) */
	for (i = 0; i < 4; i++) {
		flush |= (fetch & (1u << (16 + i))) ? 1u << kFetchFlush[i] : 0;
		flush |= (fetch & (1u << i)) ? 1u << kDmaFlush[i] : 0;
	}
	for (i = 0; i < 6; i++) {
		/* (CTL_LAYER: LM0-4 at 0x00-0x10, LM5 at 0x24; 0x14 is CTL_TOP) */
		uint32_t layer = dp_win_saved(DPW_CTL, i == 5 ? 0x16024 : 0x16000 + 4 * i);

		flush |= layer != 0 && layer != UINT32_MAX ? 1u << kLmFlush[i] : 0;
	}
	flush |= dp_win_saved(DPW_CTL, 0x160e8) != 0 ? 1u << 22 : 0;
	flush |= dp_win_saved(DPW_CTL, 0x160e4) != 0 ? 1u << 23 : 0;
	dp_wr(gDPMdss, 0x16018, flush);
	gDPReplayedMax = MAX(gDPReplayedMax, gDPReplayed);
	/* the tear check on after its settings (dpu_hw_intf_setup_te_config, then enable_tearcheck): its counter takes
	 * the init value as it is enabled */
	intf_wr(0x284, dp_win_saved(DPW_INTF, 0x36284));
	if (gDPSaved) {
		gDPStat.mdss_diff += dp_compare("the MDSS", gDPMdss, kDPMdssCheck, NULL, DP_N(kDPMdssCheck), gDPMdssWas);
		gDPStat.dsi_diff += dp_compare("the DSI controller", gDP, NULL, kDPDsiCheck, DP_N(kDPDsiCheck), gDPDsiWas);
	}
	if (gDPAutorefresh & (1u << 31)) {
		intf_wr(INTF_TEAR_AUTOREFRESH_CONFIG, gDPAutorefresh);
	}
	__atomic_store_n(&gDPState, 0, __ATOMIC_RELEASE);
	gDPStat.ups++;
	gDPStat.up_us_max = MAX(gDPStat.up_us_max, dp_us(began));
	if (!ok) {
		gDPStat.up_failed++;
		gDPBroken = true;
		bl_say("display power: on again with FAILURES in %u us; no power-down again until the next boot\n", dp_us(began));
	} else if (gDPStat.ups <= 3 || (gDPStat.ups % 50) == 0) {
		bl_say("display power: on in %u us (PLL locked in %u us; %u times; differences so far: MDSS %u DSI %u PHY %u)\n",
		    dp_us(began), gDPStat.last_lock_us, gDPStat.ups, gDPStat.mdss_diff, gDPStat.dsi_diff, gDPStat.phy_diff);
	}
}

SYSCTL_DECL(_kern_hakios);

static int
dp_stats_sysctl(__unused struct sysctl_oid * oidp, __unused void * arg1, __unused int arg2, struct sysctl_req * req)
{
	char text[640];
	int n = snprintf(text, sizeof(text), "display_power_down %u%s%s: off now %u; downs %u ups %u refused %u failed ups %u; "
	    "max down %u us, up %u us, PLL lock %u us (last %u); registers not as before: MDSS %u DSI %u PHY/PLL %u; "
	    "last reason: %s\ncycles %u (frames %u, failed %u): %s\n", gDPOn, gDPBroken ? " BROKEN" : "",
	    gDPReady ? "" : " (not mapped yet)", gDPOff, gDPStat.downs, gDPStat.ups, gDPStat.refused, gDPStat.up_failed,
	    gDPStat.down_us_max, gDPStat.up_us_max, gDPStat.lock_us_max, gDPStat.last_lock_us, gDPStat.mdss_diff,
	    gDPStat.dsi_diff, gDPStat.phy_diff, gDPStat.why[0] ? gDPStat.why : "-", gDPStat.cycles, gDPStat.cycle_frames,
	    gDPStat.cycle_failed, gDPStat.cycle[0] ? gDPStat.cycle : "-");

	return SYSCTL_OUT(req, text, (size_t) MIN(n, (int) sizeof(text) - 1) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, display_power_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    dp_stats_sysctl, "A", "the display's hardware off while the panel sleeps: counts, times, the last failure");

