/*
 * RC0 endpoint for upstream ath11k PCI (17cb:1103). C4 owns the hardware
 * and the IOVA mapping. No direct access to unlinked or invalid BAR space.
 * MSI writes a coherent RAM mailbox; handlers are polled every millisecond,
 * including when two messages overwrite the mailbox before we observe them.
 * The mailbox is diagnostic, never a condition for servicing an event ring.
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <lx_os.h>
#include "lx.h"
#ifdef CONFIG_ATH11K_PCI
#define IRQ_BASE 512u
#define VECTORS 32u
#define HANDLERS 96u
#define MSI_DATA 0x100u
#define POOL_BYTES (64ul << 20)
static struct {
 struct pci_dev dev;
 struct pci_bus bus;
 struct pci_driver *driver;
 struct hakios_pcie_link link;
 struct msi_desc msi;
 unsigned pcie_cap, vectors, msi_control, msi_mask_off, msi_mask_original;
 unsigned depth[VECTORS], status[VECTORS];
 bool enabled, region, linked, stop, polling;
 void *poll_owner;
 volatile u32 *mailbox;
 dma_addr_t mailbox_dma;
 struct {
  irq_handler_t handler, thread;
  void *arg;
  unsigned irq, active;
  bool retiring;
  unsigned long flags;
 } handlers[HANDLERS];
} endpoint;
static int cfg_read(unsigned off, unsigned size, unsigned *v)
{
 if (!endpoint.linked) return -ENODEV;
 return -lx_os_pcie_cfg_read(HAKIOS_PCIE_WIFI_RC, HAKIOS_PCIE_WIFI_BDF, off, size, v);
}
static int cfg_write(unsigned off, unsigned size, unsigned v)
{
 if (!endpoint.linked) return -ENODEV;
 return -lx_os_pcie_cfg_write(HAKIOS_PCIE_WIFI_RC, HAKIOS_PCIE_WIFI_BDF, off, size, v);
}
#define CONFIG_READ(name,type,size) \
 int lx_pci_read_config_##name(const struct pci_dev *p,int off,type *v) { \
 unsigned value; int ret = cfg_read(off,size,&value); if (!ret) *v=value; return ret; }
#define CONFIG_WRITE(name,type,size) \
 int lx_pci_write_config_##name(const struct pci_dev *p,int off,type v) { return cfg_write(off,size,v); }
CONFIG_READ(byte,u8,1)
CONFIG_READ(word,u16,2)
CONFIG_READ(dword,u32,4)
CONFIG_WRITE(byte,u8,1)
CONFIG_WRITE(word,u16,2)
CONFIG_WRITE(dword,u32,4)
static int change_command(unsigned set, unsigned clear)
{
 unsigned old, check; int ret=cfg_read(4,2,&old);
 if (!ret) ret=cfg_write(4,2,(old & ~clear)|set);
 if (!ret) ret=cfg_read(4,2,&check);
 if (!ret && (check & (set|clear)) != set) ret=-EIO;
 return ret;
}
int lx_pci_enable_device(struct pci_dev *p)
{
 int ret=change_command(2,0); if (!ret) endpoint.enabled=true; return ret;
}
void lx_pci_disable_device(struct pci_dev *p)
{
 (void)change_command(0,6); endpoint.enabled=false;
}
int lx_pci_is_enabled(struct pci_dev *p) { return endpoint.enabled; }
int lx_pci_set_master(struct pci_dev *p) { return change_command(4,0); }
int lx_pci_assign_resource(struct pci_dev *p,int bar)
{
 unsigned address;
 if (bar || cfg_read(0x10,4,&address)) return -ENODEV;
 return (address & ~15u)==endpoint.link.bar0_pa ? 0 : -EIO;
}
int lx_pci_request_region(struct pci_dev *p,int bar,const char *name)
{
 if (bar || endpoint.region) return -EBUSY;
 endpoint.region=true; return 0;
}
void lx_pci_release_region(struct pci_dev *p,int bar) { if (!bar) endpoint.region=false; }
unsigned long lx_pci_resource_len(struct pci_dev *p,int bar) { return bar ? 0 : endpoint.link.bar0_size; }
void *lx_pci_iomap(struct pci_dev *p,int bar,unsigned long maxlen)
{
 return bar || !endpoint.enabled || (maxlen && maxlen>endpoint.link.bar0_size) ? NULL : endpoint.link.bar0_va;
}
void lx_pci_iounmap(struct pci_dev *p,void *va) { /* borrowed C4 mapping */ }
int lx_pcie_read_word(struct pci_dev *p,int off,u16 *v)
{
 return endpoint.pcie_cap ? lx_pci_read_config_word(p,endpoint.pcie_cap+off,v) : -ENODEV;
}
int lx_pcie_change_word(struct pci_dev *p,int off,u16 clear,u16 set)
{
 u16 v; int ret=lx_pcie_read_word(p,off,&v);
 return ret ? ret : lx_pci_write_config_word(p,endpoint.pcie_cap+off,(v & ~clear)|set);
}
static bool in_bar(const volatile void *va)
{
 uintptr_t a=(uintptr_t)va, base=(uintptr_t)endpoint.link.bar0_va;
 return endpoint.linked && a>=base && a-base<=endpoint.link.bar0_size-4;
}
u32 lx_pci_read32(const volatile void *va)
{
 unsigned v=~0u;
 if (in_bar(va)) {
  if (lx_os_pcie_try_read32(va,&v)) lx_tell("pci: BAR read refused at %p\n",va);
  return v;
 }
 /* AHB retains its existing access semantics in a combined-bus build. */
 if (lx_os_option("wifi_soc")!=8450) return (readl)(va);
 lx_tell("pci: out-of-range BAR read %p\n",va); return v;
}
void lx_pci_write32(u32 v,volatile void *va)
{
 if (in_bar(va)) {
  if (lx_os_pcie_try_write32(va,v)) lx_tell("pci: BAR write refused at %p\n",va);
 } else if (lx_os_option("wifi_soc")!=8450) { (writel)(v,va); }
 else lx_tell("pci: out-of-range BAR write %p\n",va);
}
static void deferred_poll(void);
static void deferred_clear(void);
static void poll(void *arg)
{
 endpoint.poll_owner=lx_os_thread_self();
 while (!endpoint.stop) {
  for (unsigned i=0;i<HANDLERS;i++) {
   irq_handler_t handler,thread; void *data; unsigned irq;
   lx_os_lock();
   irq=endpoint.handlers[i].irq;
   handler=endpoint.handlers[i].handler;
   thread=endpoint.handlers[i].thread;
   data=endpoint.handlers[i].arg;
   if (!handler || endpoint.depth[irq-IRQ_BASE]) { lx_os_unlock(); continue; }
   endpoint.handlers[i].active++;
   lx_os_unlock();
   irqreturn_t result=handler(irq,data);
   if (result==IRQ_WAKE_THREAD && thread) thread(irq,data);
   lx_os_lock(); endpoint.handlers[i].active--; lx_os_unlock();
  }
  deferred_poll();
  lx_pause_ms(1);
 }
 endpoint.poll_owner=NULL;
 lx_os_lock(); endpoint.polling=false; lx_os_wakeup(&endpoint); lx_os_unlock();
}
struct msi_desc *lx_irq_get_msi_desc(unsigned irq)
{
 return irq>=IRQ_BASE && irq<IRQ_BASE+endpoint.vectors ? &endpoint.msi : NULL;
}
int lx_pci_irq_vector(struct pci_dev *p,unsigned v) { return v<endpoint.vectors ? IRQ_BASE+v : -EINVAL; }
static void msi_restore_mask(void)
{
 if (endpoint.msi_mask_off) {
  if (cfg_write(endpoint.msi_mask_off,4,endpoint.msi_mask_original))
   lx_tell("msi: cannot restore per-vector mask; MSI disabled, RC teardown required\n");
  endpoint.msi_mask_off=0;
 }
}
int lx_pci_alloc_irq_vectors(struct pci_dev *p,int min,int max,unsigned flags)
{
 unsigned control,check; int ret;
 unsigned wanted=lx_os_option("wifi_msi");
 if (wanted && wanted!=1 && wanted!=32) return -EINVAL;
 unsigned n=wanted==1 ? 1 : 32;
 if (!(flags & PCI_IRQ_MSI) || min>(int)n || max<(int)n) return -ENOSPC;
 if (endpoint.vectors) return -EBUSY;
 ret=cfg_read(p->msi_cap+2,2,&control); if (ret) return ret;
 if ((1u<<((control>>1)&7))<n) return -ENOSPC;
 endpoint.msi_control=control & ~1u;
 /* A failed DMA allocation must not leave an old mailbox target enabled. */
 ret=cfg_write(p->msi_cap+2,2,control & ~1u); if (ret) goto fail;
 if (!endpoint.mailbox) {
  endpoint.mailbox=linux_dma_alloc_coherent(&p->dev,4096,&endpoint.mailbox_dma,GFP_KERNEL);
  if (!endpoint.mailbox || endpoint.mailbox_dma>0xfffff000ull || (endpoint.mailbox_dma & 4095)) { ret=-ENOMEM; goto fail; }
 }
 endpoint.msi.msg.address_lo=(u32)endpoint.mailbox_dma;
 endpoint.msi.msg.address_hi=0;
 endpoint.msi.msg.data=MSI_DATA;
 endpoint.msi.pci.msi_attrib.is_64=!!(control & 0x80);
 ret=cfg_write(p->msi_cap+2,2,control & ~1u); if (ret) goto fail;
 ret=cfg_write(p->msi_cap+4,4,(u32)endpoint.mailbox_dma); if (ret) goto fail;
 if (control & 0x80) { ret=cfg_write(p->msi_cap+8,4,0); if (ret) goto fail; }
 ret=cfg_write(p->msi_cap+((control & 0x80)?12:8),2,MSI_DATA); if (ret) goto fail;
 /* PCI MSI per-vector masking is optional. Old firmware may have left
  * every vector masked even though MSI itself is enabled. Unmask exactly
  * the allocated messages while MSI is disabled, then verify read-back. */
 if (control & 0x100u) {
  unsigned mask_off=p->msi_cap+((control & 0x80u)?16:12);
  unsigned mask, readback, allocated=n==32 ? ~0u : 1u;
  ret=cfg_read(mask_off,4,&mask); if (ret) goto fail;
  endpoint.msi_mask_off=mask_off; endpoint.msi_mask_original=mask;
  ret=cfg_write(mask_off,4,mask & ~allocated); if (ret) goto fail;
  ret=cfg_read(mask_off,4,&readback); if (ret) goto fail;
  if ((readback & allocated) || (readback & ~allocated)!=(mask & ~allocated)) { ret=-EIO; goto fail; }
 }
 unsigned shift=n==32 ? 5 : 0;
 control=(control & ~0x71u)|(shift<<4)|1u;
 ret=cfg_write(p->msi_cap+2,2,control); if (ret) goto fail;
 ret=cfg_read(p->msi_cap+2,2,&check);
 if (ret || (check & 0x71u)!=(control & 0x71u)) {
  ret=ret ? ret : -EIO; goto fail;
 }
 endpoint.vectors=n; p->irq=IRQ_BASE; p->msi_enabled=true;
 endpoint.stop=false; endpoint.polling=true;
 ret=lx_os_thread("wifi-pci-poll",poll,NULL);
 if (ret) { endpoint.polling=false; lx_pci_free_irq_vectors(p); return -ret; }
 lx_tell("msi: %u vectors at %#llx data %#x enabled (control %#x)\n",n,(unsigned long long)endpoint.mailbox_dma,MSI_DATA,check);
 return n;
fail:
 /* Even a failed write may have reached hardware: undo with the saved
  * control word, without depending on vectors having been published. */
 (void)cfg_write(p->msi_cap+2,2,endpoint.msi_control);
 msi_restore_mask();
 memset(&endpoint.msi,0,sizeof(endpoint.msi));
 return ret;
}
void lx_pci_free_irq_vectors(struct pci_dev *p)
{
 unsigned control;
 if (endpoint.linked && p->msi_cap) {
  control=endpoint.msi_control;
  (void)cfg_read(p->msi_cap+2,2,&control);
  (void)cfg_write(p->msi_cap+2,2,control & ~1u);
 }
 msi_restore_mask();
 endpoint.stop=true;
 while (endpoint.polling) lx_pause_ms(1);
 memset(endpoint.handlers,0,sizeof(endpoint.handlers));
 memset(endpoint.depth,0,sizeof(endpoint.depth));
 memset(endpoint.status,0,sizeof(endpoint.status));
 endpoint.vectors=0; p->msi_enabled=false;
 memset(&endpoint.msi,0,sizeof(endpoint.msi));
 deferred_clear();
}
int lkpi_request_irq(struct device *dev,unsigned irq,irq_handler_t handler,irq_handler_t thread,unsigned long flags,const char *name,void *arg)
{
 if (irq<IRQ_BASE || irq>=IRQ_BASE+endpoint.vectors || !handler) return -EINVAL;
 lx_os_lock();
 unsigned slot=HANDLERS;
 for (unsigned i=0;i<HANDLERS;i++) {
  if (endpoint.handlers[i].handler && endpoint.handlers[i].irq==irq &&
      (!(flags & IRQF_SHARED) || !(endpoint.handlers[i].flags & IRQF_SHARED) || endpoint.handlers[i].arg==arg)) {
   lx_os_unlock(); return -EBUSY;
  }
  if (!endpoint.handlers[i].handler && !endpoint.handlers[i].retiring && slot==HANDLERS) slot=i;
 }
 if (slot==HANDLERS) { lx_os_unlock(); return -ENOSPC; }
 endpoint.handlers[slot].irq=irq; endpoint.handlers[slot].arg=arg;
 endpoint.handlers[slot].thread=thread; endpoint.handlers[slot].flags=flags;
 endpoint.handlers[slot].handler=handler;
 lx_os_unlock(); return 0;
}
int lkpi_enable_irq(unsigned irq)
{
 if (irq<IRQ_BASE || irq>=IRQ_BASE+endpoint.vectors) return -EINVAL;
 lx_os_lock(); if (endpoint.depth[irq-IRQ_BASE]) endpoint.depth[irq-IRQ_BASE]--; lx_os_unlock(); return 0;
}
void lkpi_disable_irq(unsigned irq)
{
 if (irq<IRQ_BASE || irq>=IRQ_BASE+endpoint.vectors) return;
 lx_os_lock(); endpoint.depth[irq-IRQ_BASE]++; lx_os_unlock();
}
void lx_synchronize_irq(unsigned irq)
{
 if (endpoint.poll_owner==lx_os_thread_self()) return;
 for (;;) {
  unsigned active=0;
  lx_os_lock();
  for (unsigned i=0;i<HANDLERS;i++) if (endpoint.handlers[i].irq==irq) active+=endpoint.handlers[i].active;
  lx_os_unlock(); if (!active) return; lx_pause_ms(1);
 }
}
void lkpi_free_irq(unsigned irq,void *arg)
{
 for (unsigned i=0;i<HANDLERS;i++) {
  lx_os_lock();
  bool match=endpoint.handlers[i].irq==irq && endpoint.handlers[i].arg==arg;
  if (match) { endpoint.handlers[i].retiring=true; endpoint.handlers[i].handler=NULL; }
  lx_os_unlock();
  if (match) { lx_synchronize_irq(irq); lx_os_lock(); memset(&endpoint.handlers[i],0,sizeof(endpoint.handlers[i])); lx_os_unlock(); }
 }
}
void lkpi_devm_free_irq(struct device *d,unsigned irq,void *arg) { lkpi_free_irq(irq,arg); }
static int capabilities(void)
{
 unsigned off; if (cfg_read(0x34,1,&off)) return -EIO;
 for (unsigned i=0;off && i<48;i++) {
  unsigned cap; if ((off & 3) || off<0x40 || off>0xfc || cfg_read(off,2,&cap)) return -EIO;
  if ((cap & 255)==5) endpoint.dev.msi_cap=off;
  if ((cap & 255)==0x10) endpoint.pcie_cap=off;
  off=(cap>>8)&255;
 }
 return endpoint.dev.msi_cap && endpoint.pcie_cap ? 0 : -ENODEV;
}
int lx_pci_register_driver(struct pci_driver *driver)
{
 if (endpoint.linked) return -EBUSY;
 int ret=lx_os_pcie_up(HAKIOS_PCIE_WIFI_RC,&endpoint.link);
 if (ret) return -ret;
 endpoint.linked=true;
 if (endpoint.link.vendor!=0x17cb || endpoint.link.device!=0x1103 || endpoint.link.bar0_size<0x200000) { ret=-ENODEV; goto fail; }
 endpoint.dev.vendor=endpoint.link.vendor; endpoint.dev.device=endpoint.link.device;
 endpoint.dev.bus=&endpoint.bus; endpoint.dev.dev.kobj.name="ath11k_pci";
 ret=capabilities(); if (ret) goto fail;
 const struct pci_device_id *id=driver->id_table;
 for (;id->vendor;id++) if (id->vendor==endpoint.dev.vendor && id->device==endpoint.dev.device) break;
 if (!id->vendor) { ret=-ENODEV; goto fail; }
 endpoint.driver=driver;
 ret=driver->probe(&endpoint.dev,id);
 if (ret) { endpoint.driver=NULL; goto fail; }
 return 0;
fail:
 lx_pci_free_irq_vectors(&endpoint.dev);
 endpoint.linked=false; lx_os_pcie_down(HAKIOS_PCIE_WIFI_RC); memset(&endpoint.link,0,sizeof(endpoint.link));
 lx_pci_pool_reset();
 memset(&endpoint,0,sizeof(endpoint));
 return ret;
}
void lx_pci_unregister_driver(struct pci_driver *driver)
{
 if (!endpoint.linked) return;
 if (endpoint.driver && endpoint.driver->remove) endpoint.driver->remove(&endpoint.dev);
 endpoint.driver=NULL; lx_pci_free_irq_vectors(&endpoint.dev);
 endpoint.linked=false; lx_os_pcie_down(HAKIOS_PCIE_WIFI_RC); memset(&endpoint.link,0,sizeof(endpoint.link));
 lx_pci_pool_reset();
 memset(&endpoint,0,sizeof(endpoint));
}
void *lx_pci_base(void) { return endpoint.driver ? pci_get_drvdata(&endpoint.dev) : NULL; }
/* Deferred CE tasklets and DP NAPI execute only on the polling thread. */
#define DEFERRED_MAX 64
static struct { struct tasklet_struct *task; bool pending, running; } tasks[DEFERRED_MAX];
static struct { struct napi_struct *napi; bool enabled, pending, running; } napis[DEFERRED_MAX];
void tasklet_setup(struct tasklet_struct *task,tasklet_callback_t *fn)
{
 memset(task,0,sizeof(*task)); task->callback=fn; task->use_callback=true;
 lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (!tasks[i].task || tasks[i].task==task) { tasks[i].task=task; lx_os_unlock(); return; }
 lx_os_unlock(); lx_os_panic("pci: tasklet table full");
}
void tasklet_schedule(struct tasklet_struct *task)
{
 lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (tasks[i].task==task) tasks[i].pending=true;
 lx_os_unlock();
}
void tasklet_kill(struct tasklet_struct *task)
{
 for (unsigned i=0;i<DEFERRED_MAX;i++) {
  for (;;) {
   lx_os_lock();
   if (tasks[i].task!=task) { lx_os_unlock(); break; }
   tasks[i].pending=false;
   if (!tasks[i].running) { lx_os_unlock(); break; }
   lx_os_unlock(); lx_pause_ms(1);
  }
 }
}
struct net_device *linuxkpi_alloc_netdev(size_t size,const char *name,uint32_t flags,void (*setup)(struct net_device *))
{
 struct net_device *dev=kzalloc(sizeof(*dev)+size,GFP_KERNEL);
 if (dev && setup) setup(dev); return dev;
}
void linuxkpi_free_netdev(struct net_device *dev) { kfree(dev); }
void linuxkpi_netif_napi_add(struct net_device *dev,struct napi_struct *n,int (*fn)(struct napi_struct *,int))
{
 memset(n,0,sizeof(*n)); n->dev=dev; n->poll=fn; n->budget=64;
 lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (!napis[i].napi) { napis[i].napi=n; lx_os_unlock(); return; }
 lx_os_unlock(); lx_os_panic("pci: NAPI table full");
}
void linuxkpi_napi_synchronize(struct napi_struct *n)
{
 if (endpoint.poll_owner==lx_os_thread_self()) return;
 for (;;) {
  bool active=false; lx_os_lock();
  for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n) active=napis[i].running;
  lx_os_unlock(); if (!active) return; lx_pause_ms(1);
 }
}
void linuxkpi_napi_disable(struct napi_struct *n)
{
 lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n) { napis[i].enabled=false; napis[i].pending=false; }
 lx_os_unlock(); linuxkpi_napi_synchronize(n);
}
void linuxkpi_napi_enable(struct napi_struct *n)
{
 lx_os_lock(); for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n) napis[i].enabled=true; lx_os_unlock();
}
bool linuxkpi_napi_schedule(struct napi_struct *n)
{
 bool queued=false; lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n && napis[i].enabled) { queued=!napis[i].pending; napis[i].pending=true; }
 lx_os_unlock(); return queued;
}
bool linuxkpi_napi_complete_done(struct napi_struct *n,int count)
{
 lx_os_lock(); for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n) napis[i].pending=false; lx_os_unlock(); return true;
}
void linuxkpi_netif_napi_del(struct napi_struct *n)
{
 linuxkpi_napi_disable(n); lx_os_lock();
 for (unsigned i=0;i<DEFERRED_MAX;i++) if (napis[i].napi==n) memset(&napis[i],0,sizeof(napis[i]));
 lx_os_unlock();
}
static void deferred_poll(void)
{
 for (unsigned i=0;i<DEFERRED_MAX;i++) {
  lx_os_lock(); struct tasklet_struct *t=tasks[i].pending ? tasks[i].task : NULL;
  if (t) { tasks[i].pending=false; tasks[i].running=true; }
  lx_os_unlock();
  if (t) { t->callback(t); lx_os_lock(); tasks[i].running=false; lx_os_unlock(); }
  lx_os_lock(); struct napi_struct *n=napis[i].pending && napis[i].enabled ? napis[i].napi : NULL;
  if (n) napis[i].running=true;
  lx_os_unlock();
  if (n) { n->poll(n,n->budget); lx_os_lock(); napis[i].running=false; lx_os_unlock(); }
 }
}
static void deferred_clear(void)
{
 memset(tasks,0,sizeof(tasks)); memset(napis,0,sizeof(napis));
}
void lx_disable_irq_sync(unsigned irq)
{
 lkpi_disable_irq(irq); lx_synchronize_irq(irq);
}
void lx_irq_set_status_flags(unsigned irq,unsigned long flags)
{
 if (irq<IRQ_BASE || irq>=IRQ_BASE+endpoint.vectors) return;
 lx_os_lock();
 if ((flags & IRQ_NOAUTOEN) && !(endpoint.status[irq-IRQ_BASE] & IRQ_NOAUTOEN)) endpoint.depth[irq-IRQ_BASE]++;
 endpoint.status[irq-IRQ_BASE]|=flags;
 lx_os_unlock();
}
int lx_pci_dma_mask(struct device *d,u64 mask) { return mask>=0xffffffffull ? 0 : -EIO; }
void DELAY(int us)
{
 unsigned long long end=lx_os_nanoseconds()+(unsigned long long)(us>0?us:0)*1000;
 while (lx_os_nanoseconds()<end) {
#ifdef __aarch64__
  __asm__ volatile("yield");
#else
  __asm__ volatile("" ::: "memory");
#endif
 }
}
int device_printf(device_t dev,const char *fmt,...)
{
 char line[320]; va_list ap; va_start(ap,fmt); int len=vsnprintf(line,sizeof(line),fmt,ap); va_end(ap); lx_os_tell(line); return len;
}
cpumask_t *lkpi_get_static_single_cpu_mask(int cpu)
{
 static cpumask_t boot_cpu; CPU_ZERO(&boot_cpu); CPU_SET(0,&boot_cpu); return &boot_cpu;
}
const struct property *of_find_property(const struct device_node *np,const char *name,int *lenp)
{
 if (lenp) *lenp=0; return NULL;
}
struct device_node *of_find_node_by_type(struct device_node *from,const char *type) { return NULL; }
int of_address_to_resource(struct device_node *node,int index,struct resource *res) { return -ENODEV; }
#endif
