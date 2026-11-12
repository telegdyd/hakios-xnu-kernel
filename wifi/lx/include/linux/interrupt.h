/* PCI IRQ operations are serviced in the one polling thread. */
#ifndef _LX_PCI_INTERRUPT_H_
#define _LX_PCI_INTERRUPT_H_
#include_next <linux/interrupt.h>
#ifdef CONFIG_ATH11K_PCI
#undef irq_get_msi_desc
#define irq_get_msi_desc lx_irq_get_msi_desc
struct msi_desc *lx_irq_get_msi_desc(unsigned);
#define disable_irq lx_disable_irq_sync
void lx_disable_irq_sync(unsigned);
#define irq_set_status_flags lx_irq_set_status_flags
void lx_irq_set_status_flags(unsigned, unsigned long);
#define synchronize_irq lx_synchronize_irq
void lx_synchronize_irq(unsigned);
#define irq_set_affinity_and_hint(irq, mask) 0
#endif
#endif
