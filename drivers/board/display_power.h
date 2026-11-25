/*
 * Display sleep/wake (Samsung AMB670YF08 panel on-sequence) and SM8450 MDSS/DSI hardware
 * power-down while the panel sleeps (feature "display-power").
 *
 * Included by drivers/board/backlight.c (uses profile, gcc, dispcc, intf, dsi_lock,
 * rd/wr, say, sleep_ms, delay, now_us).
 *
 * Why hardware power-down: screen off puts the panel to sleep (DCS 0x28, 0x10), stopping TE
 * and frame transfers, while MDSS, DSI0, DSI PHY/PLL, and dispcc PLLs stay powered on MMCX.
 * Powering them down (DSI link clocks off, DSI PLL/PHY off, MDP clocks off, MDSS/INT2 GDSCs
 * collapsed with flop retention, dispcc PLLs off) allows MMCX/CX collapse while the panel
 * sleeps. Power-up reverses the sequence and replays the saved MDSS/VBIF/DSI/PHY/PLL state
 * before waking the panel.
 */
#ifndef _HAKIOS_DISPLAY_POWER_H
#define _HAKIOS_DISPLAY_POWER_H

#include <stdarg.h>

extern int hakios_mmio_try_read32(const volatile void *, unsigned *) __attribute__((weak));

/* Boot-arg display_power_down=0: keep the display hardware powered while the panel sleeps (default 1). */
static TUNABLE(uint32_t, dp_on_arg, "display_power_down", 1);

/*
 * Samsung AMB670YF08 <DSIInitSequence>: 101 packets, 7 waits.
 * Each entry: TYPE LEN BYTES... (DSI data type), or 0xff MS (wait).
 */
#define PANEL_ON_PACKETS 101

static const uint8_t kPanelOn[720] = {
	0x39, 0x81, 0x9e, 0x12, 0x00, 0x00, 0xab, 0x30, 0x80, 0x09, 0x6c, 0x04, 0x38, 0x00, 0x24, 0x02, 0x1c, 0x02, 0x1c,
	0x02, 0x00, 0x02, 0x3b, 0x00, 0x20, 0x03, 0x35, 0x00, 0x07, 0x00, 0x0e, 0x03, 0x34, 0x02, 0xd4, 0x18, 0x00, 0x10,
	0xf0, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0f, 0x0f, 0x33, 0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77,
	0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00, 0x2a, 0x40, 0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa, 0x3a, 0xf8, 0x3b,
	0x38, 0x3b, 0x78, 0x3b, 0xb6, 0x4b, 0xb6, 0x4b, 0xf4, 0x4b, 0xf4, 0x6c, 0x34, 0x84, 0x74, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x15, 0x02, 0x9d, 0x01,
	0x05, 0x01, 0x11,
	0xff, 0x0c,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x31, 0xc5,
	0x39, 0x22, 0xc5, 0x29, 0x14, 0x00, 0x00, 0x00, 0x11, 0x10, 0x50, 0x05, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84,
	0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca, 0x84, 0xca,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x11,
	0x05, 0x01, 0x2c,
	0x05, 0x01, 0x00,
	0xff, 0x19,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x22, 0xb9,
	0x39, 0x03, 0xb9, 0xa1, 0xb1,
	0x39, 0x04, 0xb0, 0x00, 0x3a, 0xb9,
	0x15, 0x02, 0xb9, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x26, 0xb9,
	0x39, 0x03, 0xb9, 0x00, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x24, 0xb9,
	0x15, 0x02, 0xb9, 0x21,
	0x39, 0x04, 0xb0, 0x00, 0x38, 0xb9,
	0x15, 0x02, 0xb9, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x2a, 0xb9,
	0x39, 0x03, 0xb9, 0x00, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0xff, 0x79,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x16, 0xf2,
	0x39, 0x03, 0xf2, 0x1b, 0x50,
	0x39, 0x04, 0xb0, 0x00, 0x08, 0xcb,
	0x15, 0x02, 0xcb, 0x24,
	0x15, 0x02, 0x60, 0x00,
	0x39, 0x03, 0xbd, 0x23, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x10, 0xbd,
	0x15, 0x02, 0xbd, 0x00,
	0x39, 0x04, 0xb0, 0x00, 0x16, 0xbd,
	0x15, 0x02, 0xbd, 0x77,
	0x39, 0x04, 0xb0, 0x00, 0x14, 0xbd,
	0x15, 0x02, 0xbd, 0x00,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x03, 0xf2, 0x00, 0x01,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0x35, 0x00,
	0x15, 0x02, 0xb9, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x05, 0x2a, 0x00, 0x00, 0x04, 0x37,
	0x39, 0x05, 0x2b, 0x00, 0x00, 0x09, 0x6b,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0xc3, 0x89,
	0x39, 0x04, 0xb0, 0x00, 0x24, 0xc3,
	0x39, 0x12, 0xc3, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00, 0xfd, 0x00,
	0xfd,
	0x39, 0x04, 0xb0, 0x00, 0x3a, 0xc3,
	0x39, 0x12, 0xc3, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00, 0xb0, 0x00,
	0xb0,
	0x39, 0x04, 0xb0, 0x00, 0x50, 0xc3,
	0x39, 0x12, 0xc3, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00, 0x43, 0x00,
	0x43,
	0x39, 0x04, 0xb0, 0x00, 0x61, 0xc3,
	0x39, 0x17, 0xc3, 0xcc, 0xcc, 0xcc, 0xcc, 0xc0, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe,
	0x00, 0xfe, 0x00, 0xfe, 0x00, 0xfe,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x2b, 0xf6,
	0x39, 0x04, 0xf6, 0x60, 0x63, 0x69,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x46, 0xf4,
	0x15, 0x02, 0xf4, 0x08,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x18, 0xb1,
	0x39, 0x02, 0xb1, 0x05,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x0d, 0x63,
	0x15, 0x02, 0x63, 0x01,
	0x39, 0x04, 0xb0, 0x00, 0x0c, 0x63,
	0x15, 0x02, 0x63, 0x20,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x15, 0x02, 0x53, 0x28,
	0x15, 0x02, 0xf7, 0x0f,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x15, 0x02, 0x55, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x39, 0x03, 0xf0, 0x5a, 0x5a,
	0x39, 0x04, 0xb0, 0x00, 0x52, 0x1f,
	0x15, 0x02, 0x1f, 0x02,
	0x39, 0x04, 0xb0, 0x00, 0x54, 0x1f,
	0x15, 0x02, 0x1f, 0x00,
	0x39, 0x03, 0xf0, 0xa5, 0xa5,
	0x05, 0x01, 0x29,
};

#define DP_DSI0_SIZE            0x1000
#define DP_MDSS_SIZE_ALL        0x90000
#define DP_VBIF_SIZE            0x2000

#define DP_GDSC_MDSS            0x9000
#define DP_GDSC_INT2            0xb000
#define DP_GDSC_COLLAPSE        (1u << 0)
#define DP_GDSC_HW_CTRL         (1u << 1)
#define DP_GDSC_RETAIN_FF       (1u << 11)
#define DP_GDSC_PWR_ON          (1u << 31)
#define DP_PLL_MODE             0x00
#define DP_PLL_OPMODE           0x04
#define DP_PLL_USER_CTL         0x18

#define DP_N(a)                 (sizeof(a) / sizeof((a)[0]))

static const struct {
	uint32_t offset;
	const char *name;
} kDPLink[] = {
	{ 0x8038, "esc0" }, { 0x8004, "pclk0" }, { 0x802c, "byte0_intf" }, { 0x8028, "byte0" },
}, kDPCore[] = {
	{ 0x800c, "mdp" }, { 0x801c, "mdp_lut" }, { 0x8014, "rot" }, { 0x8024, "vsync" }, { 0xa004, "mdp1" },
	{ 0xa014, "mdp_lut1" }, { 0xa00c, "rot1" }, { 0xa01c, "vsync1" }, { 0xa020, "ahb1" },
	{ 0xc008, "rscc_vsync" }, { 0xc00c, "rscc_ahb" }, { 0xc004, "non_gdsc_ahb" }, { 0x80a4, "ahb" },
};
static const struct {
	uint32_t offset;
	const char *name;
	uint32_t feed;
} kDPRCG[] = {
	{ 0x80ec, "mdp_clk_src", 0 }, { 0x8104, "rot_clk_src", 1 }, { 0x8324, "ahb_clk_src", 1 },
	{ 0x8134, "byte0_clk_src", 2 }, { 0x80bc, "pclk0_clk_src", 2 },
};
static const uint32_t kDPPLL[] = { 0x0000, 0x1000 };     /* disp_cc_pll0, disp_cc_pll1 */

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

static const uint16_t kDPPhyRestore[] = {
	PHY_CTRL_4, PHY_LANE_CFG0, PHY_LANE_CFG1, PHY_GLBL_CTRL, PHY_VREG_CTRL_0, PHY_VREG_CTRL_1,
	PHY(0x10c), PHY(0xec), PHY(0xf0), PHY(0x104), PHY(0x108), PHY(0xf4), PHY(0xf8), PHY(0xfc), PHY(0x100),
};
static const uint16_t kDPPhyAfter[] = {
	PHY_LANE_CTRL0, PHY_CTRL_2,
	PHY_TIMING_CTRL(0), PHY_TIMING_CTRL(1), PHY_TIMING_CTRL(2), PHY_TIMING_CTRL(3), PHY_TIMING_CTRL(4),
	PHY_TIMING_CTRL(5), PHY_TIMING_CTRL(6), PHY_TIMING_CTRL(7), PHY_TIMING_CTRL(8), PHY_TIMING_CTRL(9),
	PHY_TIMING_CTRL(10), PHY_TIMING_CTRL(11), PHY_TIMING_CTRL(12), PHY_TIMING_CTRL(13),
	PHY(0xa4), PHY(0xa8), PHY(0xac), PHY(0xb0),
};
static const uint8_t kDPLaneRegs[] = { 0x14, 0x10, 0x00, 0x04, 0x08, 0x18 };
static const uint16_t kDPPllRestore[] = {
	PLL_OUTDIV_RATE, PHY_CLK_CFG0,
	PLL(0xbc), PLL(0xe0), PLL(0xe4), PLL(0xe8), PLL(0xec), PLL(0x158), PLL(0x194), PLL(0x250), PLL(0x19c),
	PLL(0x258), PLL(0x240), PLL(0x18), PLL(0x04), PLL(0x10), PLL(0x20), PLL(0x24), PLL(0x44), PLL(0x68), PLL(0x78),
	PLL(0xa8), PLL(0xb8), PLL(0xc8), PLL(0x160), PLL(0x168), PLL(0x170), PLL(0x178), PLL(0x190), PLL(0x90), PLL(0x94),
	PLL(0x260),
	PLL(0x120), PLL(0x124), PLL(0x128), PLL(0x12c), PLL(0x130), PLL(0x134), PLL(0x150),
	PLL(0x00), PLL(0x08), PLL(0x0c), PLL(0x14), PLL(0x1c), PLL(0x30),
};
static const uint16_t kDPPhyRunning[] = {
	PHY_CTRL_0, PHY_CTRL_3, PHY_CLK_CFG1, PHY_RBUF_CTRL, PHY_PLL_CNTRL, PLL_SYSTEM_MUXES, PHY_DIGTOP_SPARE10,
};

static const uint32_t kDPMdssCheck[] = {
	0x05000, 0x05014, 0x05024, 0x05030, 0x07014,
	0x16000, 0x16014, 0x160f4,
	0x36284, 0x36288, 0x3628c, 0x362a0,
};
static const uint16_t kDPDsiCheck[] = { 0x004, 0x03c, 0x040, 0x044, 0x058, 0x05c, 0x084, 0x0b0, 0x0c4, 0x0cc, 0x2a8, 0x2ac };

#define DP_PHY_SAVED    (DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter) + DP_N(kDPPllRestore) + DP_N(kDPPhyRunning))

enum { DPW_MDSS, DPW_VBIF, DPW_TOP, DPW_SSPP, DPW_LM, DPW_DSC, DPW_M3D, DPW_PP, DPW_INTF, DPW_CTL, DPW_DSI };
static const struct {
	uint32_t base;
	uint16_t len;
	uint8_t kind;
} kDPWin[] = {
	{ 0x00000, 0x200, DPW_MDSS },
	{ 0x008, 0x8, DPW_VBIF }, { 0x020, 0x10, DPW_VBIF }, { 0x0ac, 0x2c, DPW_VBIF }, { 0x160, 0x8, DPW_VBIF },
	{ 0x550, 0x80, DPW_VBIF },
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
	{ 0x6a0e0, 0x20, DPW_PP }, { 0x6b0e0, 0x20, DPW_PP }, { 0x6c0e0, 0x20, DPW_PP }, { 0x6d0e0, 0x20, DPW_PP },
	{ 0x6e0e0, 0x20, DPW_PP }, { 0x6f0e0, 0x20, DPW_PP }, { 0x668e0, 0x20, DPW_PP }, { 0x66ce0, 0x20, DPW_PP },
	{ 0x36000, 0x300, DPW_INTF },
	{ 0x16000, 0x204, DPW_CTL }, { 0x17000, 0x204, DPW_CTL }, { 0x18000, 0x204, DPW_CTL }, { 0x19000, 0x204, DPW_CTL },
	{ 0x1a000, 0x204, DPW_CTL }, { 0x1b000, 0x204, DPW_CTL },
	{ 0x000, 0x400, DPW_DSI },
};
#define DP_WIN_WORDS    8192
static uint32_t dp_win_val[DP_WIN_WORDS];
static uint32_t dp_win_words, dp_replayed, dp_replayed_max;
static bool dp_win_read, dp_win_bad[96];
_Static_assert(sizeof(kDPWin) / sizeof(kDPWin[0]) <= 96, "dp_win_bad: one per window");

static bool
dp_skip(uint8_t kind, uint32_t o)
{
	switch (kind) {
	case DPW_MDSS:
		return o == 0x0 || (o >= 0x10 && o < 0x20);
	case DPW_TOP:
		return o == 0x0 || o == 0xc || o == 0x14 || o == 0x18 || o == 0x20 || o == 0x24 || o == 0x2c || o == 0x2b0 ||
		       o == 0x2b8 || o == 0x2c0 || o == 0x2c8 || o == 0x360 || o == 0x364 || o == 0x3ac || o == 0x3b4 || o == 0x3bc ||
		       o == 0x3d4;
	case DPW_INTF:
		return o == 0xac || o == 0xb0 || (o >= 0x1c0 && o < 0x1d0) || o == 0x26c || o == 0x284 || o == 0x298 || o == 0x2b0 ||
		       o == 0x2b4;
	case DPW_CTL:
		return o == 0x18 || o == 0x1c || o == 0x30 || o == 0xd0 || (o >= 0x100 && o < 0x120) || o >= 0x200;
	case DPW_DSI:
		return o == 0x0 || o == 0x4 || o == 0x8 || o == 0xc || o == 0x68 || o == 0x90 || o == 0x94 || o == 0x98 ||
		       o == 0x9c || o == 0xa4 || o == 0xbc || o == 0x110 || o == 0x114 || o == 0x11c || o == 0x12c || o == 0x17c ||
		       o == 0x184 || o == 0x190 || o == 0x1ec || o == 0x1f0 || o == 0x1f4;
	default:
		return false;
	}
}

static const struct {
	uint32_t from, to;
} kDPNever[] = {
	{ 0x1380, 0x13a8 }, { 0x1420, 0x144c },
};

static bool
dp_never(uint8_t kind, uint32_t at)
{
	if (kind == DPW_VBIF || kind == DPW_DSI) {
		return false;
	}
	for (uint32_t i = 0; i < DP_N(kDPNever); i++) {
		if (at >= kDPNever[i].from && at < kDPNever[i].to) {
			return true;
		}
	}
	return false;
}

/* GCC display AXI branches (offsets in GCC): stopped before MDSS GDSC collapse */
static const struct {
	uint32_t offset;
	const char *name;
} kDPGccAxi[] = { { 0x3700c, "gcc_disp_hf_axi" }, { 0x37014, "gcc_disp_sf_axi" } };

static vm_offset_t dp_dsi, dp_mdss, dp_vbif;
static uint32_t dp_axi_was[2];
static bool dp_ready, dp_broken, dp_saved, dp_dead;
static volatile uint32_t dp_state;              /* 0 on, 1 going down or down, 2 coming up */
static volatile bool dp_off;
static volatile int32_t dp_users;
static uint32_t dp_phy[DP_PHY_SAVED], dp_lane[5][sizeof(kDPLaneRegs)], dp_ctrl0;
static uint32_t dp_mdss_was[DP_N(kDPMdssCheck)], dp_dsi_was[DP_N(kDPDsiCheck)];
static uint32_t dp_link_was[DP_N(kDPLink)], dp_core_was[DP_N(kDPCore)], dp_pll_mode[2], dp_pll_user[2];
static uint32_t dp_gdsc_was[2], dp_autorefresh;
static bool dp_pll_off_flag[2], dp_gdsc_off_flag[2];
static struct {
	uint32_t downs, ups, refused, up_failed, mdss_diff, dsi_diff, phy_diff, lock_us_max, down_us_max, up_us_max;
	uint32_t last_lock_us, cycles, cycle_frames, cycle_failed;
	char why[160], cycle[224];
} dp_stat;

static inline bool
dp_enabled(void)
{
	return profile.has_display_power && dp_on_arg != 0;
}

static inline bool
dp_retry(void)
{
	return dp_enabled() && !dp_broken && !dp_off;
}

static inline bool
dp_regs_on(void)
{
	return __atomic_load_n(&dp_state, __ATOMIC_ACQUIRE) == 0;
}

bool hakios_display_hold(void);
void hakios_display_release(void);
bool hakios_display_off(void);

bool
hakios_display_hold(void)
{
	if (!dp_regs_on()) {
		return false;
	}
	__atomic_add_fetch(&dp_users, 1, __ATOMIC_SEQ_CST);
	if (!dp_regs_on()) {
		__atomic_sub_fetch(&dp_users, 1, __ATOMIC_SEQ_CST);
		return false;
	}
	return true;
}

void
hakios_display_release(void)
{
	__atomic_sub_fetch(&dp_users, 1, __ATOMIC_SEQ_CST);
}

bool
hakios_display_off(void)
{
	return dp_off;
}

static int
dp_try_rd(vm_offset_t address, unsigned *value)
{
#if defined(HAKIOS_MMIO_TEST)
	*value = rd(address);
	return 0;
#else
	if (hakios_mmio_try_read32 == NULL) {
		return ENOTSUP;
	}
	return hakios_mmio_try_read32((const volatile void *)address, value);
#endif
}

static void
dp_why(const char *format, ...) __printflike(1, 2);

static void
dp_why(const char *format, ...)
{
	va_list list;

	va_start(list, format);
	vsnprintf(dp_stat.why, sizeof(dp_stat.why), format, list);
	va_end(list);
	say("display power: %s\n", dp_stat.why);
}

static bool
dp_wait(vm_offset_t base, uint32_t o, uint32_t mask, uint32_t want, uint32_t us)
{
	for (uint32_t i = 0; i <= us; i++) {
		if ((rd(base + o) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

static bool
dp_map(void)
{
	unsigned v;

	if (dp_ready) {
		return true;
	}
	if (!profile.has_display_power || profile.mdss == 0 || profile.vbif == 0 || profile.dsi == 0 ||
	    dispcc == 0 || gcc == 0 || intf == 0) {
		return false;
	}
	dp_dsi = ml_io_map(profile.dsi, DP_DSI0_SIZE);
	dp_mdss = ml_io_map(profile.mdss, DP_MDSS_SIZE_ALL);
	dp_vbif = ml_io_map(profile.vbif, DP_VBIF_SIZE);
	if (dp_dsi == 0 || dp_mdss == 0 || dp_vbif == 0) {
		return false;
	}
	dp_win_words = 0;
	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		dp_win_words += kDPWin[w].len / 4;
	}
	if (dp_win_words > DP_WIN_WORDS) {
		say("display power: %u words of registers to keep, room for %u\n", dp_win_words, (uint32_t)DP_WIN_WORDS);
		return false;
	}
	if (dp_try_rd(dispcc + DP_GDSC_MDSS, &v) != 0 || dp_try_rd(dp_dsi + PHY(0), &v) != 0 ||
	    dp_try_rd(gcc + kDPGccAxi[0].offset, &v) != 0) {
		return false;
	}
	dp_ready = true;
	return true;
}

static inline vm_offset_t
dp_win_base(uint8_t kind)
{
	return kind == DPW_VBIF ? dp_vbif : kind == DPW_DSI ? dp_dsi : dp_mdss;
}

static void
dp_windows(bool save, uint8_t from, uint8_t to)
{
	uint32_t n = 0;

	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		vm_offset_t b = dp_win_base(kDPWin[w].kind);
		uint32_t block = kDPWin[w].base & ~0xfffu;

		if (save && !dp_win_read) {
			for (uint32_t o = 0; o < kDPWin[w].len; o += 4) {
				unsigned v;

				if (dp_never(kDPWin[w].kind, kDPWin[w].base + o)) {
					continue;
				}
				if (dp_try_rd(b + kDPWin[w].base + o, &v) != 0) {
					dp_win_bad[w] = true;
				}
			}
			if (dp_win_bad[w]) {
				say("display power: window %u (kind %u at %#x) faults on reading: not kept\n", w, kDPWin[w].kind,
				    kDPWin[w].base);
			}
		}
		if (dp_win_bad[w]) {
			n += kDPWin[w].len / 4;
			continue;
		}
		for (uint32_t o = 0; o < kDPWin[w].len; o += 4, n++) {
			uint32_t at = kDPWin[w].base + o;

			if (dp_never(kDPWin[w].kind, at)) {
				dp_win_val[n] = 0;
				continue;
			}
			if (save) {
				dp_win_val[n] = rd(b + at);
			} else if (kDPWin[w].kind >= from && kDPWin[w].kind <= to &&
			    !dp_skip(kDPWin[w].kind, kDPWin[w].kind == DPW_DSI || kDPWin[w].kind == DPW_VBIF ? at : at - block) &&
			    rd(b + at) != dp_win_val[n]) {
				wr(b + at, dp_win_val[n]);
				dp_replayed++;
			}
		}
	}
	dp_win_read = dp_win_read || save;
}

static uint32_t
dp_win_saved(uint8_t kind, uint32_t at)
{
	uint32_t n = 0;

	for (uint32_t w = 0; w < DP_N(kDPWin); w++) {
		if (kDPWin[w].kind == kind && at >= kDPWin[w].base && at < kDPWin[w].base + kDPWin[w].len) {
			return dp_win_val[n + (at - kDPWin[w].base) / 4];
		}
		n += kDPWin[w].len / 4;
	}
	return UINT32_MAX;
}

static uint32_t
dp_phy_saved(uint32_t o)
{
	uint32_t n = 0;

	for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++, n++) {
		if (kDPPhyRestore[i] == o) {
			return dp_phy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++, n++) {
		if (kDPPhyAfter[i] == o) {
			return dp_phy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++, n++) {
		if (kDPPllRestore[i] == o) {
			return dp_phy[n];
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPhyRunning); i++, n++) {
		if (kDPPhyRunning[i] == o) {
			return dp_phy[n];
		}
	}
	return UINT32_MAX;
}

static void
dp_save(void)
{
	uint32_t n = 0;

	if (!dp_saved) {
		for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++) {
			dp_phy[n++] = rd(dp_dsi + kDPPhyRestore[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++) {
			dp_phy[n++] = rd(dp_dsi + kDPPhyAfter[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++) {
			dp_phy[n++] = rd(dp_dsi + kDPPllRestore[i]);
		}
		for (uint32_t i = 0; i < DP_N(kDPPhyRunning); i++) {
			dp_phy[n++] = rd(dp_dsi + kDPPhyRunning[i]);
		}
		for (uint32_t l = 0; l < 5; l++) {
			for (uint32_t i = 0; i < sizeof(kDPLaneRegs); i++) {
				dp_lane[l][i] = rd(dp_dsi + LANE(l, kDPLaneRegs[i]));
			}
		}
		dp_ctrl0 = rd(dp_dsi + PHY_CTRL_0);
		dp_saved = true;
		say("display power: the boot loader's state: PHY ctrl_0 %#x clk_cfg0 %#x clk_cfg1 %#x pll_cntrl %#x vreg %#x/%#x, PLL "
		    "dec %#x frac %#x/%#x/%#x outdiv_rate %#x; GDSCs %#x %#x; PLLs mode %#x %#x user %#x %#x\n", dp_ctrl0,
		    rd(dp_dsi + PHY_CLK_CFG0), rd(dp_dsi + PHY_CLK_CFG1), rd(dp_dsi + PHY_PLL_CNTRL), rd(dp_dsi + PHY_VREG_CTRL_0),
		    rd(dp_dsi + PHY_VREG_CTRL_1), rd(dp_dsi + PLL(0xe0)), rd(dp_dsi + PLL(0xe4)), rd(dp_dsi + PLL(0xe8)),
		    rd(dp_dsi + PLL(0xec)), rd(dp_dsi + PLL_OUTDIV_RATE), rd(dispcc + DP_GDSC_MDSS), rd(dispcc + DP_GDSC_INT2),
		    rd(dispcc + kDPPLL[0] + DP_PLL_MODE), rd(dispcc + kDPPLL[1] + DP_PLL_MODE),
		    rd(dispcc + kDPPLL[0] + DP_PLL_USER_CTL), rd(dispcc + kDPPLL[1] + DP_PLL_USER_CTL));
	}
	for (uint32_t i = 0; i < DP_N(kDPMdssCheck); i++) {
		dp_mdss_was[i] = rd(dp_mdss + kDPMdssCheck[i]);
	}
	for (uint32_t i = 0; i < DP_N(kDPDsiCheck); i++) {
		dp_dsi_was[i] = rd(dp_dsi + kDPDsiCheck[i]);
	}
	dp_windows(true, 0, 0);
}

static bool
dp_branch_off(uint32_t o, uint32_t *was)
{
	*was = rd(dispcc + o);
	if (!(*was & 1u)) {
		return true;
	}
	wr(dispcc + o, *was & ~1u);
	return dp_wait(dispcc, o, 1u << 31, 1u << 31, 200);
}

static bool
dp_branch_on(uint32_t o, uint32_t was)
{
	if (!(was & 1u)) {
		return true;
	}
	wr(dispcc + o, was);
	return (was & (1u << 31)) != 0 || dp_wait(dispcc, o, 1u << 31, 0, 500);
}

static void
dp_phy_off(void)
{
	wr(dp_dsi + PHY_CLK_CFG1, rd(dp_dsi + PHY_CLK_CFG1) & ~(1u << 5));
	wr(dp_dsi + PHY_PLL_CNTRL, 0);
	wr(dp_dsi + PHY_RBUF_CTRL, 0);
	wr(dp_dsi + PLL_SYSTEM_MUXES, 0);
	wr(dp_dsi + PHY_CTRL_0, rd(dp_dsi + PHY_CTRL_0) & ~(1u << 5));
	delay(1);
	wr(dp_dsi + LANE(0, LANE_LPRX_CTRL), 0);
	wr(dp_dsi + PHY_DIGTOP_SPARE10, 0);
	delay(2);
	wr(dp_dsi + PHY_CTRL_0, rd(dp_dsi + PHY_CTRL_0) & ~0x1fu);
	wr(dp_dsi + PHY_LANE_CTRL0, 0);
	wr(dp_dsi + PHY_CTRL_0, 0);
}

static bool
dp_phy_on(void)
{
	uint32_t n = 0;
	uint64_t t;

	wr(dp_dsi + PHY_DIGTOP_SPARE10, 1);
	delay(500);
	if (!dp_wait(dp_dsi, PHY_PHY_STATUS, 1u, 1u, 1000)) {
		dp_why("the DSI PHY's reference not ready (phy_status %#x)", rd(dp_dsi + PHY_PHY_STATUS));
		return false;
	}
	wr(dp_dsi + PHY_CTRL_0, (1u << 6) | (1u << 5));
	wr(dp_dsi + PHY_PLL_CNTRL, 0);
	wr(dp_dsi + PHY_RBUF_CTRL, 0);
	for (uint32_t i = 0; i < DP_N(kDPPhyRestore); i++) {
		wr(dp_dsi + kDPPhyRestore[i], dp_phy[n++]);
	}
	wr(dp_dsi + PHY_CTRL_3, 0);
	wr(dp_dsi + PHY_CTRL_0, 0x7f);
	for (uint32_t i = 0; i < DP_N(kDPPhyAfter); i++) {
		wr(dp_dsi + kDPPhyAfter[i], dp_phy[n++]);
	}
	wr(dp_dsi + PHY_CLK_CFG1, dp_phy_saved(PHY_CLK_CFG1) & 0xcu);
	for (uint32_t l = 0; l < 5; l++) {
		for (uint32_t i = 0; i < sizeof(kDPLaneRegs); i++) {
			wr(dp_dsi + LANE(l, kDPLaneRegs[i]), dp_lane[l][i]);
		}
	}
	for (uint32_t i = 0; i < DP_N(kDPPllRestore); i++) {
		wr(dp_dsi + kDPPllRestore[i], dp_phy[n++]);
	}
	wr(dp_dsi + PHY_CLK_CFG1, (rd(dp_dsi + PHY_CLK_CFG1) & ~3u) | (dp_phy_saved(PHY_CLK_CFG1) & 3u));
	wr(dp_dsi + PHY_CTRL_0, rd(dp_dsi + PHY_CTRL_0) | (1u << 5));
	wr(dp_dsi + PLL_SYSTEM_MUXES, 0xc0);
	delay(1);
	t = now_us();
	wr(dp_dsi + PHY_PLL_CNTRL, 1);
	if (!dp_wait(dp_dsi, PLL_COMMON_STATUS_ONE, 1u, 1u, 5000)) {
		dp_why("the DSI PLL did not lock in 5 ms (status %#x)", rd(dp_dsi + PLL_COMMON_STATUS_ONE));
		return false;
	}
	dp_stat.last_lock_us = (uint32_t)(now_us() - t);
	if (dp_stat.last_lock_us > dp_stat.lock_us_max) {
		dp_stat.lock_us_max = dp_stat.last_lock_us;
	}
	wr(dp_dsi + PHY_DIGTOP_SPARE4, 1);
	wr(dp_dsi + PHY_DIGTOP_SPARE4, 0);
	wr(dp_dsi + PHY_CTRL_3, 0x04);
	wr(dp_dsi + PHY_CLK_CFG1, rd(dp_dsi + PHY_CLK_CFG1) | (1u << 5) | (1u << 4));
	wr(dp_dsi + PHY_RBUF_CTRL, dp_phy_saved(PHY_RBUF_CTRL));
	return true;
}

static bool
dp_gdsc_off(uint32_t o, uint32_t *was)
{
	uint32_t v = *was = rd(dispcc + o);

	if (!(v & DP_GDSC_PWR_ON)) {
		return true;
	}
	if (v & DP_GDSC_HW_CTRL) {
		v &= ~DP_GDSC_HW_CTRL;
		wr(dispcc + o, v);
		delay(1);
		if (!dp_wait(dispcc, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 500)) {
			wr(dispcc + o, *was);
			return false;
		}
	}
	v |= DP_GDSC_RETAIN_FF;
	wr(dispcc + o, v);
	wr(dispcc + o, v | DP_GDSC_COLLAPSE);
	if (!dp_wait(dispcc, o, DP_GDSC_PWR_ON, 0, 500)) {
		wr(dispcc + o, v);
		dp_wait(dispcc, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 2000);
		wr(dispcc + o, *was);
		return false;
	}
	return true;
}

static bool
dp_gdsc_on(uint32_t o, uint32_t was)
{
	uint32_t v = rd(dispcc + o);

	wr(dispcc + o, v & ~DP_GDSC_COLLAPSE);
	if (!dp_wait(dispcc, o, DP_GDSC_PWR_ON, DP_GDSC_PWR_ON, 2000)) {
		return false;
	}
	delay(1);
	if (was & DP_GDSC_HW_CTRL) {
		wr(dispcc + o, (v & ~DP_GDSC_COLLAPSE) | DP_GDSC_HW_CTRL);
		delay(1);
	}
	return true;
}

static void
dp_pll_off(uint32_t i)
{
	uint32_t p = kDPPLL[i];

	dp_pll_mode[i] = rd(dispcc + p + DP_PLL_MODE);
	dp_pll_user[i] = rd(dispcc + p + DP_PLL_USER_CTL);
	dp_pll_off_flag[i] = false;
	if ((dp_pll_user[i] & (1u << 25)) || !(rd(dispcc + p + DP_PLL_OPMODE) & 1u) || !(dp_pll_mode[i] & 1u)) {
		return;
	}
	wr(dispcc + p + DP_PLL_MODE, dp_pll_mode[i] & ~1u);
	wr(dispcc + p + DP_PLL_USER_CTL, dp_pll_user[i] & ~7u);
	wr(dispcc + p + DP_PLL_OPMODE, 0);
	wr(dispcc + p + DP_PLL_MODE, dp_pll_mode[i] & ~5u);
	dp_pll_off_flag[i] = true;
}

static bool
dp_pll_on(uint32_t i)
{
	uint32_t p = kDPPLL[i];

	if (!dp_pll_off_flag[i]) {
		return true;
	}
	dp_pll_off_flag[i] = false;
	wr(dispcc + p + DP_PLL_MODE, (dp_pll_mode[i] & ~1u) | (1u << 2));
	wr(dispcc + p + DP_PLL_OPMODE, 1);
	if (!dp_wait(dispcc, p + DP_PLL_MODE, 1u << 31, 1u << 31, 1000)) {
		return false;
	}
	wr(dispcc + p + DP_PLL_USER_CTL, dp_pll_user[i]);
	wr(dispcc + p + DP_PLL_MODE, dp_pll_mode[i]);
	return true;
}

static uint32_t
dp_compare(const char *what, vm_offset_t b, const uint32_t *offsets, const uint16_t *offsets16, uint32_t n,
    const uint32_t *was)
{
	uint32_t diff = 0;
	char line[200];
	int at = 0;

	line[0] = 0;
	for (uint32_t i = 0; i < n; i++) {
		uint32_t o = offsets != NULL ? offsets[i] : offsets16[i], v = rd(b + o);

		if (v != was[i]) {
			diff++;
			if (at < (int)sizeof(line) - 40) {
				at += snprintf(line + at, sizeof(line) - (size_t)at, " %#x:%#x(was %#x)", o, v, was[i]);
			}
		}
	}
	if (diff != 0) {
		say("display power: %u of %s's registers not as before:%s\n", diff, what, line);
	}
	return diff;
}

static void dp_up(void);

static bool
dp_down(void)
{
	uint64_t began = now_us();
	uint32_t i, g;

	if (!dp_enabled() || dp_broken || dp_off) {
		return dp_off;
	}
	if (!dp_map()) {
		dp_why("the display's registers not mapped or not readable: never");
		dp_broken = true;
		return false;
	}
	__atomic_store_n(&dp_state, 1, __ATOMIC_SEQ_CST);
	for (i = 0; i < 300 && __atomic_load_n(&dp_users, __ATOMIC_SEQ_CST) != 0; i++) {
		sleep_ms(1);
	}
	if (__atomic_load_n(&dp_users, __ATOMIC_SEQ_CST) != 0) {
		__atomic_store_n(&dp_state, 0, __ATOMIC_RELEASE);
		dp_stat.refused++;
		dp_why("the framebuffer held the display for 300 ms");
		return false;
	}
	lck_mtx_lock(&dsi_lock);
	lck_mtx_unlock(&dsi_lock);
	if (rd(dp_dsi + DSI_STATUS) & (DSI_STATUS_CMD_DMA_BUSY | DSI_STATUS_CMD_MDP_BUSY)) {
		__atomic_store_n(&dp_state, 0, __ATOMIC_RELEASE);
		dp_stat.refused++;
		dp_why("the DSI controller busy (status %#x)", rd(dp_dsi + DSI_STATUS));
		return false;
	}
	dp_save();
	dp_autorefresh = rd(intf + INTF_TEAR_AUTOREFRESH_CONFIG);
	if (dp_autorefresh & (1u << 31)) {
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, dp_autorefresh & ~(1u << 31));
	}
	for (i = 0; i < DP_N(kDPLink); i++) {
		if (!dp_branch_off(kDPLink[i].offset, &dp_link_was[i])) {
			say("display power: dispcc %s (%#x) not stopped: %#x\n", kDPLink[i].name, kDPLink[i].offset,
			    rd(dispcc + kDPLink[i].offset));
		}
	}
	for (i = 0; i < DP_N(kDPRCG); i++) {
		if (kDPRCG[i].feed == 2 && !(rd(dispcc + kDPRCG[i].offset) & (1u << 31))) {
			say("display power: dispcc %s still running (%#x) as the DSI PLL goes off\n", kDPRCG[i].name,
			    rd(dispcc + kDPRCG[i].offset));
		}
	}
	dp_phy_off();
	for (i = 0; i < DP_N(kDPCore); i++) {
		if (!dp_branch_off(kDPCore[i].offset, &dp_core_was[i])) {
			say("display power: dispcc %s (%#x) not stopped: %#x\n", kDPCore[i].name, kDPCore[i].offset,
			    rd(dispcc + kDPCore[i].offset));
		}
	}
	for (i = 0; i < DP_N(kDPGccAxi); i++) {
		dp_axi_was[i] = rd(gcc + kDPGccAxi[i].offset);
		if (dp_axi_was[i] & 1u) {
			wr(gcc + kDPGccAxi[i].offset, dp_axi_was[i] & ~1u);
		}
	}
	for (g = 0; g < 2; g++) {
		uint32_t o = g == 0 ? DP_GDSC_INT2 : DP_GDSC_MDSS;

		dp_gdsc_off_flag[g] = false;
		if (!dp_gdsc_off(o, &dp_gdsc_was[g])) {
			dp_why("dispcc GDSC %#x stayed on (%#x): the display put back on", o, rd(dispcc + o));
			dp_stat.refused++;
			goto back;
		}
		dp_gdsc_off_flag[g] = true;
	}
	for (i = 0; i < DP_N(kDPRCG); i++) {
		uint32_t src = (rd(dispcc + kDPRCG[i].offset + 4) >> 8) & 7u;
		bool fed = kDPRCG[i].feed == 0 ? src == 1 : kDPRCG[i].feed == 1 ? src == 4 || src == 6 : false;

		if (fed && !(rd(dispcc + kDPRCG[i].offset) & (1u << 31))) {
			dp_why("dispcc %s still running (%#x): its PLL cannot go off, the display put back on", kDPRCG[i].name,
			    rd(dispcc + kDPRCG[i].offset));
			dp_stat.refused++;
			goto back;
		}
	}
	for (i = 0; i < 2; i++) {
		dp_pll_off(i);
	}
	dp_off = true;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	dp_stat.downs++;
	{
		uint32_t us = (uint32_t)(now_us() - began);
		if (us > dp_stat.down_us_max) {
			dp_stat.down_us_max = us;
		}
		if (dp_stat.downs <= 3 || (dp_stat.downs % 50) == 0) {
			say("display power: off in %u us (%u times; PLLs off %u %u; GDSCs now %#x %#x)\n", us, dp_stat.downs,
			    dp_pll_off_flag[0], dp_pll_off_flag[1], rd(dispcc + DP_GDSC_MDSS), rd(dispcc + DP_GDSC_INT2));
		}
	}
	return true;
back:
	dp_up();
	return false;
}

static void
dp_up(void)
{
	uint64_t began = now_us();
	bool ok = true, regs = true;
	uint32_t i, g;

	if (__atomic_load_n(&dp_state, __ATOMIC_ACQUIRE) == 0 || dp_dead) {
		return;
	}
	dp_off = false;
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	__atomic_store_n(&dp_state, 2, __ATOMIC_SEQ_CST);
	for (i = 0; i < 2; i++) {
		if (!dp_pll_on(i)) {
			dp_why("dispcc PLL %u did not lock (mode %#x)", i, rd(dispcc + kDPPLL[i] + DP_PLL_MODE));
			ok = false;
		}
	}
	for (g = 2; g-- > 0;) {
		uint32_t o = g == 0 ? DP_GDSC_INT2 : DP_GDSC_MDSS;

		if (dp_gdsc_off_flag[g]) {
			dp_gdsc_off_flag[g] = false;
			if (!dp_gdsc_on(o, dp_gdsc_was[g])) {
				dp_why("dispcc GDSC %#x did not come on (%#x)", o, rd(dispcc + o));
				ok = regs = false;
			}
		}
	}
	for (i = DP_N(kDPGccAxi); i-- > 0;) {
		if (dp_axi_was[i] & 1u) {
			wr(gcc + kDPGccAxi[i].offset, dp_axi_was[i]);
		}
	}
	for (i = DP_N(kDPCore); i-- > 0;) {
		if (!dp_branch_on(kDPCore[i].offset, dp_core_was[i])) {
			dp_why("dispcc %s did not start (%#x)", kDPCore[i].name, rd(dispcc + kDPCore[i].offset));
			ok = false;
			regs = regs && kDPCore[i].offset != 0x80a4 && kDPCore[i].offset != 0xa020;
		}
	}
	if (!regs) {
		dp_dead = dp_broken = true;
		dp_stat.up_failed++;
		say("display power: the display's domains or register bus did NOT come back; its registers are not touched "
		    "again until the next boot (%s)\n", dp_stat.why);
		return;
	}
	dp_replayed = 0;
	dp_windows(false, DPW_MDSS, DPW_CTL);
	if (dp_saved) {
		if (!dp_phy_on()) {
			ok = false;
		}
		dp_stat.phy_diff += dp_compare("the DSI PHY", dp_dsi, NULL, kDPPhyRunning, DP_N(kDPPhyRunning),
		    &dp_phy[DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter) + DP_N(kDPPllRestore)]);
		dp_stat.phy_diff += dp_compare("the DSI PLL", dp_dsi, NULL, kDPPllRestore, DP_N(kDPPllRestore),
		    &dp_phy[DP_N(kDPPhyRestore) + DP_N(kDPPhyAfter)]);
	}
	for (i = DP_N(kDPLink); i-- > 0;) {
		if (!dp_branch_on(kDPLink[i].offset, dp_link_was[i])) {
			dp_why("dispcc %s did not start (%#x)", kDPLink[i].name, rd(dispcc + kDPLink[i].offset));
			ok = false;
		}
	}
	dp_windows(false, DPW_DSI, DPW_DSI);
	wr(dp_dsi + DSI_INT_CTRL, dp_win_saved(DPW_DSI, DSI_INT_CTRL) & DSI_INT_ENABLES);
	wr(dp_dsi + DSI_CTRL, dp_win_saved(DPW_DSI, DSI_CTRL));

	wr(dp_mdss + 0x16100, dp_win_saved(DPW_CTL, 0x160e4));
	wr(dp_mdss + 0x16110, dp_win_saved(DPW_CTL, 0x160f4));
	wr(dp_mdss + 0x16104, dp_win_saved(DPW_CTL, 0x160e8));

	uint32_t flush = (1u << 17) | (1u << 31), fetch = dp_win_saved(DPW_CTL, 0x160fc);
	static const uint8_t kFetchFlush[] = { 0, 1, 2, 18 };
	static const uint8_t kDmaFlush[] = { 11, 12, 24, 25 };
	static const uint8_t kLmFlush[] = { 6, 7, 8, 9, 10, 20 };

	for (i = 0; i < 4; i++) {
		flush |= (fetch & (1u << (16 + i))) ? (1u << kFetchFlush[i]) : 0;
		flush |= (fetch & (1u << i)) ? (1u << kDmaFlush[i]) : 0;
	}
	for (i = 0; i < 6; i++) {
		uint32_t layer = dp_win_saved(DPW_CTL, i == 5 ? 0x16024 : 0x16000 + 4 * i);

		flush |= (layer != 0 && layer != UINT32_MAX) ? (1u << kLmFlush[i]) : 0;
	}
	flush |= dp_win_saved(DPW_CTL, 0x160e8) != 0 ? (1u << 22) : 0;
	flush |= dp_win_saved(DPW_CTL, 0x160e4) != 0 ? (1u << 23) : 0;
	wr(dp_mdss + 0x16018, flush);
	if (dp_replayed > dp_replayed_max) {
		dp_replayed_max = dp_replayed;
	}
	wr(intf + 0x284, dp_win_saved(DPW_INTF, 0x36284));
	if (dp_saved) {
		dp_stat.mdss_diff += dp_compare("the MDSS", dp_mdss, kDPMdssCheck, NULL, DP_N(kDPMdssCheck), dp_mdss_was);
		dp_stat.dsi_diff += dp_compare("the DSI controller", dp_dsi, NULL, kDPDsiCheck, DP_N(kDPDsiCheck), dp_dsi_was);
	}
	if (dp_autorefresh & (1u << 31)) {
		wr(intf + INTF_TEAR_AUTOREFRESH_CONFIG, dp_autorefresh);
	}
	__atomic_store_n(&dp_state, 0, __ATOMIC_RELEASE);
	dp_stat.ups++;
	{
		uint32_t us = (uint32_t)(now_us() - began);
		if (us > dp_stat.up_us_max) {
			dp_stat.up_us_max = us;
		}
		if (!ok) {
			dp_stat.up_failed++;
			dp_broken = true;
			say("display power: on again with FAILURES in %u us; no power-down again until the next boot\n", us);
		} else if (dp_stat.ups <= 3 || (dp_stat.ups % 50) == 0) {
			say("display power: on in %u us (PLL locked in %u us; %u times; differences so far: MDSS %u DSI %u PHY %u)\n",
			    us, dp_stat.last_lock_us, dp_stat.ups, dp_stat.mdss_diff, dp_stat.dsi_diff, dp_stat.phy_diff);
		}
	}
}

static int
dp_stats_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	char text[640];
	int n = snprintf(text, sizeof(text), "display_power_down %u%s%s: off now %u; downs %u ups %u refused %u failed ups %u; "
	    "max down %u us, up %u us, PLL lock %u us (last %u); registers not as before: MDSS %u DSI %u PHY/PLL %u; "
	    "last reason: %s\ncycles %u (frames %u, failed %u): %s\n", dp_enabled() ? 1u : 0u, dp_broken ? " BROKEN" : "",
	    dp_ready ? "" : " (not mapped yet)", dp_off, dp_stat.downs, dp_stat.ups, dp_stat.refused, dp_stat.up_failed,
	    dp_stat.down_us_max, dp_stat.up_us_max, dp_stat.lock_us_max, dp_stat.last_lock_us, dp_stat.mdss_diff,
	    dp_stat.dsi_diff, dp_stat.phy_diff, dp_stat.why[0] ? dp_stat.why : "-", dp_stat.cycles, dp_stat.cycle_frames,
	    dp_stat.cycle_failed, dp_stat.cycle[0] ? dp_stat.cycle : "-");

	return SYSCTL_OUT(req, text, (size_t)MIN(n, (int)sizeof(text) - 1) + 1);
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, display_power_stats, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, NULL, 0,
    dp_stats_sysctl, "A", "the display's hardware off while the panel sleeps: counts, times, the last failure");

#endif /* _HAKIOS_DISPLAY_POWER_H */
