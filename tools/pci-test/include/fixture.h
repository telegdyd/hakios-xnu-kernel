#ifndef FIXTURE_H
#define FIXTURE_H
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "hakios.pcie.h"
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t dma_addr_t;
typedef void *device_t;
typedef uint64_t cpumask_t;
#define CPU_ZERO(p) (*(p)=0)
#define CPU_SET(i,p) (*(p)|=1ull<<(i))
#define GFP_KERNEL 0
#define __printflike(a,b)
#define IRQF_SHARED 4
#define IRQ_NOAUTOEN (1u<<12)
#define PCI_IRQ_MSI 1
#define IRQ_HANDLED 1
#define IRQ_WAKE_THREAD 2
typedef int irqreturn_t;
typedef irqreturn_t (*irq_handler_t)(int,void *);
struct device { struct { const char *name; } kobj; void *driver_data; };
struct pci_bus { int domain; };
struct pci_dev { struct device dev; struct pci_bus *bus; unsigned short vendor,device; unsigned msi_cap,irq; bool msi_enabled; };
struct pci_device_id { unsigned vendor,device; };
struct pci_driver { const struct pci_device_id *id_table; int (*probe)(struct pci_dev *,const struct pci_device_id *); void (*remove)(struct pci_dev *); };
struct msi_desc { struct { unsigned address_lo,address_hi,data; } msg; struct { struct { bool is_64; } msi_attrib; } pci; };
#define pci_get_drvdata(p) ((p)->dev.driver_data)
struct tasklet_struct;
typedef void tasklet_callback_t(struct tasklet_struct *);
struct tasklet_struct { tasklet_callback_t *callback; bool use_callback; };
struct net_device { unsigned dummy; };
struct napi_struct { struct net_device *dev; int (*poll)(struct napi_struct *,int); int budget; };
struct device_node { unsigned dummy; };
struct resource { unsigned dummy; };
struct property { unsigned dummy; };
static inline void *kzalloc(size_t n,int flags) { return calloc(1,n); }
#define kfree(p) free(p)
static inline u32 readl(const volatile void *p) { return *(const volatile u32 *)p; }
static inline void writel(u32 v,volatile void *p) { *(volatile u32 *)p=v; }
int lx_os_pcie_up(unsigned,struct hakios_pcie_link *);
void lx_os_pcie_down(unsigned);
int lx_os_pcie_cfg_read(unsigned,unsigned,unsigned,unsigned,unsigned *);
int lx_os_pcie_cfg_write(unsigned,unsigned,unsigned,unsigned,unsigned);
int lx_os_pcie_try_read32(const volatile void *,unsigned *);
int lx_os_pcie_try_write32(volatile void *,unsigned);
unsigned lx_os_option(const char *);
void lx_os_lock(void);
void lx_os_unlock(void);
void lx_os_wakeup(const void *);
int lx_os_thread(const char *,void (*)(void *),void *);
void *lx_os_thread_self(void);
unsigned long long lx_os_nanoseconds(void);
void lx_os_tell(const char *);
_Noreturn void lx_os_panic(const char *);
void *linux_dma_alloc_coherent(struct device *,size_t,dma_addr_t *,int);
void lx_pci_free_irq_vectors(struct pci_dev *);
void lx_pci_pool_reset(void);
void lx_synchronize_irq(unsigned);
void tasklet_schedule(struct tasklet_struct *);
#endif
