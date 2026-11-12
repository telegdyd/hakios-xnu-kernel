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

#endif
