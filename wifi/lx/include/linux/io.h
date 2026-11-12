/* PCI BAR accesses recover an external abort through the C4 boundary. */
#ifndef _LX_PCI_IO_H_
#define _LX_PCI_IO_H_
#include_next <linux/io.h>
#ifdef CONFIG_ATH11K_PCI
u32 lx_pci_read32(const volatile void *);
void lx_pci_write32(u32, volatile void *);
#undef readl
#undef writel
#undef readl_relaxed
#undef writel_relaxed
#undef ioread32
#undef iowrite32
#define readl(a) lx_pci_read32(a)
#define writel(v,a) lx_pci_write32(v,a)
#define readl_relaxed(a) lx_pci_read32(a)
#define writel_relaxed(v,a) lx_pci_write32(v,a)
#define ioread32(a) lx_pci_read32(a)
#define iowrite32(v,a) lx_pci_write32(v,a)
#endif
#endif
