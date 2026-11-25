/* C2: SM8450 PCIe0. No XNU or Linux types cross this boundary. */
#ifndef _SYS_HAKIOS_PCIE_H_
#define _SYS_HAKIOS_PCIE_H_

struct hakios_pcie_link {
	unsigned long long bar0_pa;
	void *bar0_va;
	unsigned long bar0_size;
	unsigned short vendor, device;
	unsigned gen, width;
};

/* BDF encoding: bus << 8 | device << 3 | function. RC0 endpoint is 0x0100. */
#define HAKIOS_PCIE_WIFI_RC 0u
#define HAKIOS_PCIE_WIFI_BDF 0x0100u

/* 0 or positive errno (all C2/C3/C4 functions returning int, except noted).
 * up: zero *link first; on failure undo partial setup and leave it zero.
 * Success owns rails/GPIOs/clocks/PHY/link, BAR0 decode and bus mastering.
 * Caller serializes up/down. Only RC0 supported; others return ENODEV.
 */
int hakios_pcie_up(unsigned rc, struct hakios_pcie_link *link);
/* Idempotent, including after failed up. No allocation or sleeping mutex;
 * callable from panic. Caller stops DMA/MHI before PERST/WLAN_EN/votes go off.
 * Invalidates all borrowed BAR pointers. Does not own the SMMU context.
 */
void hakios_pcie_down(unsigned rc);
int hakios_pcie_down_status(void); /* 0 only after a complete shutdown; EBUSY while up */
int hakios_pcie_pm_start(void);
void hakios_pcie_pm_sample(void);
/* size=1/2/4, naturally aligned off, off+size<=4096; narrow reads zero-extend.
 * No endpoint config/BAR access before link-up. Failure leaves *value alone.
 */
int hakios_pcie_cfg_read(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned *value);
int hakios_pcie_cfg_write(unsigned rc, unsigned bdf, unsigned off, unsigned size, unsigned value);
/* Aligned mapped VA. Recover synchronous external abort as EIO; never a raw
 * unguarded load/store. Failed read leaves *value alone. Writes include barriers.
 */
int hakios_pcie_try_read32(const volatile void *va, unsigned *value);
int hakios_pcie_try_write32(volatile void *va, unsigned value);
/* Link sleep with the radio powered. sleep: MHI already in M3; saves the endpoint's config, D3hot, PME_Turn_Off,
 * PERST, controller off, PCIe's bus floor released. wake: back, config restored, link PM as requested. Caller
 * serializes both with up/down. While asleep config access and the bar accessors fail (ENXIO), never touch it. */
int hakios_pcie_sleep(_Bool fast_mhi);
_Bool hakios_pcie_drv_cx_ready(void);
int hakios_pcie_wake(void);
int hakios_pcie_l1_hold(unsigned *saved);
int hakios_pcie_l1_restore(unsigned saved);
_Bool hakios_pcie_asleep(void);
int hakios_pcie_bar_read32(const volatile void *va, unsigned *value);
int hakios_pcie_bar_write32(volatile void *va, unsigned value);
/* WAKE# (GPIO 96) through the PDC: cb in interrupt context, the line then masked; arm(1) unmasks it. */
int hakios_pcie_wake_irq(void (*cb)(void));
void hakios_pcie_wake_arm(_Bool on);
_Bool hakios_pcie_wake_line(void);
/* How the link sleeps: 1 DRV (trained, L1.2, the audio DSP holding it; PERST# high, the endpoint in D0), 2 off (PERST#). */
unsigned hakios_pcie_sleep_mode(void);

#endif
