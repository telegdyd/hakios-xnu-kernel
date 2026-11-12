/* Tests compile the production shim, with only kernel/PCI hardware replaced. */
#include "fixture.h"
#include "../../wifi/lx/src/lx_pci.c"
static unsigned char config[4096];
static unsigned bar[0x200000/4];
static unsigned char dma[4096];
static unsigned option_msi=32, fault_offset=~0u, cfg_writes, unlinked, handlers_called;
static bool fail_dma, fail_thread, fail_readback, raw_fault, fail_enable_write;
static unsigned stuck_mask_offset=~0u;
static unsigned long long ticks;
void lx_pci_pool_reset(void) {}
void lx_os_lock(void) {}
void lx_os_unlock(void) {}
void lx_os_wakeup(const void *p) {}
void *lx_os_thread_self(void) { return (void *)1; }
unsigned long long lx_os_nanoseconds(void) { return ticks+=1000; }
void lx_os_tell(const char *s) {}
void lx_tell(const char *fmt,...) {}
_Noreturn void lx_os_panic(const char *s) { fprintf(stderr,"%s\n",s); abort(); }
unsigned lx_os_option(const char *name) { return !strcmp(name,"wifi_msi")?option_msi:!strcmp(name,"wifi_soc")?8450:0; }
void lx_pause_ms(unsigned ms) { endpoint.stop=true; }
int lx_os_thread(const char *name,void (*fn)(void *),void *arg) { return fail_thread?EAGAIN:0; }
int lx_os_pcie_up(unsigned rc,struct hakios_pcie_link *link)
{
 *link=(struct hakios_pcie_link){.bar0_va=bar,.bar0_pa=0x60400000,.bar0_size=sizeof(bar),.vendor=0x17cb,.device=0x1103,.gen=2,.width=1}; return 0;
}
void lx_os_pcie_down(unsigned rc) { unlinked++; }
int lx_os_pcie_cfg_read(unsigned rc,unsigned bdf,unsigned off,unsigned size,unsigned *v)
{
 if (off==fault_offset) return EIO;
 *v=0; memcpy(v,config+off,size);
 if (fail_readback && off==0x52) *v &= ~1u;
 return 0;
}
int lx_os_pcie_cfg_write(unsigned rc,unsigned bdf,unsigned off,unsigned size,unsigned v)
{
 cfg_writes++;
 if (off==fault_offset) return EIO;
 if (off==stuck_mask_offset) return 0;
 memcpy(config+off,&v,size);
 if (fail_enable_write && off==0x52 && (v & 1)) { fail_enable_write=false; return EIO; }
 return 0;
}
int lx_os_pcie_try_read32(const volatile void *va,unsigned *v)
{
 if (raw_fault) return EIO;
 *v=*(volatile unsigned *)va; return 0;
}
int lx_os_pcie_try_write32(volatile void *va,unsigned v)
{
 if (raw_fault) return EIO;
 *(volatile unsigned *)va=v; return 0;
}
void *linux_dma_alloc_coherent(struct device *dev,size_t n,dma_addr_t *iova,int flags)
{
 if (fail_dma) return NULL;
 assert(n==4096); *iova=0xa0000000; return dma;
}
static unsigned cfg(unsigned off,unsigned n) { unsigned v=0; memcpy(&v,config+off,n); return v; }
static void setup(void)
{
 memset(&endpoint,0,sizeof(endpoint)); memset(config,0,sizeof(config)); deferred_clear();
 endpoint.linked=true; endpoint.enabled=true; endpoint.link.bar0_va=bar;
 endpoint.link.bar0_size=sizeof(bar); endpoint.link.bar0_pa=0x60400000;
 endpoint.dev.msi_cap=0x50; option_msi=32; fault_offset=~0u; stuck_mask_offset=~0u; fail_dma=fail_thread=fail_readback=raw_fault=fail_enable_write=false;
 unsigned flags=0x80|(5u<<1); memcpy(config+0x52,&flags,2); handlers_called=0; cfg_writes=0;
}
static irqreturn_t handler(int irq,void *data) { handlers_called+=(unsigned)(uintptr_t)data; return IRQ_HANDLED; }
static void run_poll(void) { endpoint.stop=false; endpoint.polling=true; poll(NULL); assert(!endpoint.polling); }
static int probe(struct pci_dev *p,const struct pci_device_id *id)
{
 /* Same observable order as ath11k before its MHI power-up call. */
 assert(lx_pci_alloc_irq_vectors(p,32,32,PCI_IRQ_MSI)==32);
 assert(cfg(p->msi_cap+2,2)&1); return 0;
}
static int probe_fail(struct pci_dev *p,const struct pci_device_id *id)
{
 assert(!probe(p,id)); run_poll(); return -EIO;
}
static void remove_driver(struct pci_dev *p) { assert(cfg(p->msi_cap+2,2)&1); }
static void test_msi(void)
{
 setup(); assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,PCI_IRQ_MSI)==32);
 assert(cfg(0x54,4)==0xa0000000); assert(cfg(0x58,4)==0); assert(cfg(0x5c,2)==0x100);
 assert((cfg(0x52,2)&0x71)==0x51); assert(lx_pci_irq_vector(&endpoint.dev,31)==IRQ_BASE+31);
 assert(lx_irq_get_msi_desc(IRQ_BASE)->msg.data==0x100);
 assert(lx_irq_get_msi_desc(IRQ_BASE)->msg.address_lo==cfg(0x54,4));
 assert(lx_irq_get_msi_desc(IRQ_BASE)->msg.address_hi==cfg(0x58,4));
 assert(lx_pci_irq_vector(&endpoint.dev,32)==-EINVAL);
 run_poll(); lx_pci_free_irq_vectors(&endpoint.dev); assert(!(cfg(0x52,2)&1));
 setup(); option_msi=1; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-ENOSPC);
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,1,1,1)==1); assert((cfg(0x52,2)&0x71)==1);
 run_poll(); lx_pci_free_irq_vectors(&endpoint.dev);
 setup(); option_msi=7; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,1,32,1)==-EINVAL); assert(!cfg_writes);
}
static void test_per_vector_mask(void)
{
 setup(); unsigned flags=0x180|(5u<<1), mask=~0u;
 memcpy(config+0x52,&flags,2); memcpy(config+0x60,&mask,4);
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==32); assert(cfg(0x60,4)==0);
 run_poll(); lx_pci_free_irq_vectors(&endpoint.dev); assert(cfg(0x60,4)==~0u); assert(!(cfg(0x52,2)&1));
 setup(); memcpy(config+0x52,&flags,2); memcpy(config+0x60,&mask,4); stuck_mask_offset=0x60;
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EIO); assert(!(cfg(0x52,2)&1));
 setup(); option_msi=1; memcpy(config+0x52,&flags,2); memcpy(config+0x60,&mask,4);
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,1,1,1)==1); assert(cfg(0x60,4)==0xfffffffeu);
 run_poll(); lx_pci_free_irq_vectors(&endpoint.dev); assert(cfg(0x60,4)==~0u);
 setup(); flags=0x100|(5u<<1); memcpy(config+0x52,&flags,2); memcpy(config+0x5c,&mask,4);
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==32); assert(cfg(0x5c,4)==0);
 assert(cfg(0x58,2)==0x100); run_poll(); lx_pci_free_irq_vectors(&endpoint.dev); assert(cfg(0x5c,4)==~0u);
 setup(); memcpy(config+0x52,&flags,2); memcpy(config+0x5c,&mask,4); fault_offset=0x5c;
 assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EIO); assert(!(cfg(0x52,2)&1));
}
static void test_failed_probe_retry(void)
{
 setup(); endpoint.linked=false; endpoint.enabled=false;
 config[0x34]=0x50; config[0x50]=5; config[0x51]=0x70; config[0x70]=0x10;
 const struct pci_device_id ids[]={{0x17cb,0x1103},{0,0}};
 struct pci_driver d={.id_table=ids,.probe=probe_fail,.remove=remove_driver};
 assert(lx_pci_register_driver(&d)==-EIO);
 assert(!endpoint.linked && !endpoint.enabled && !endpoint.region && !endpoint.pcie_cap && !endpoint.dev.msi_cap);
 assert(!endpoint.vectors && !endpoint.polling && !endpoint.mailbox && !(cfg(0x52,2)&1));
 d.probe=probe; assert(!lx_pci_register_driver(&d)); run_poll(); lx_pci_unregister_driver(&d);
}
static void test_failures(void)
{
 setup(); fail_dma=true; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-ENOMEM); assert(!(cfg(0x52,2)&1));
 setup(); fault_offset=0x54; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EIO); assert(!(cfg(0x52,2)&1));
 setup(); fail_readback=true; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EIO); assert(!(cfg(0x52,2)&1));
 setup(); fail_enable_write=true; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EIO); assert(!(cfg(0x52,2)&1)); assert(!endpoint.vectors);
 setup(); fail_thread=true; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,32,32,1)==-EAGAIN); assert(!(cfg(0x52,2)&1)); assert(!endpoint.vectors);
}
static void test_handlers(void)
{
 setup(); option_msi=1; assert(lx_pci_alloc_irq_vectors(&endpoint.dev,1,1,1)==1);
 assert(!lkpi_request_irq(NULL,IRQ_BASE,handler,NULL,IRQF_SHARED,"a",(void *)1));
 assert(!lkpi_request_irq(NULL,IRQ_BASE,handler,NULL,IRQF_SHARED,"b",(void *)2));
 assert(lkpi_request_irq(NULL,IRQ_BASE,handler,NULL,0,"x",(void *)4)==-EBUSY);
 run_poll(); assert(handlers_called==3);
 lkpi_free_irq(IRQ_BASE,(void *)1); run_poll(); assert(handlers_called==5);
 lkpi_disable_irq(IRQ_BASE); run_poll(); assert(handlers_called==5);
 lkpi_enable_irq(IRQ_BASE); run_poll(); assert(handlers_called==7);
 lkpi_free_irq(IRQ_BASE,(void *)2); run_poll(); assert(handlers_called==7);
 lx_pci_free_irq_vectors(&endpoint.dev); run_poll(); assert(handlers_called==7);
}
static unsigned deferred_calls;
static void task(struct tasklet_struct *t) { deferred_calls++; }
static int napi_poll(struct napi_struct *n,int budget) { deferred_calls++; linuxkpi_napi_complete_done(n,0); return 0; }
static void test_deferred(void)
{
 setup(); struct tasklet_struct t; struct napi_struct n; deferred_calls=0;
 tasklet_setup(&t,task); tasklet_schedule(&t); deferred_poll(); assert(deferred_calls==1);
 tasklet_schedule(&t); tasklet_kill(&t); deferred_poll(); assert(deferred_calls==1);
 tasklet_schedule(&t); deferred_poll(); assert(deferred_calls==2); /* kill permits reuse */
 linuxkpi_netif_napi_add(NULL,&n,napi_poll); assert(!linuxkpi_napi_schedule(&n));
 linuxkpi_napi_enable(&n); assert(linuxkpi_napi_schedule(&n)); deferred_poll(); assert(deferred_calls==3);
 linuxkpi_napi_disable(&n); deferred_poll(); assert(deferred_calls==3); linuxkpi_netif_napi_del(&n);
}
static void test_registration_and_mmio(void)
{
 setup(); endpoint.linked=false;
 config[0x34]=0x50; config[0x50]=5; config[0x51]=0x70; config[0x70]=0x10;
 const struct pci_device_id ids[]={{0x17cb,0x1103},{0,0}};
 struct pci_driver d={.id_table=ids,.probe=probe,.remove=remove_driver};
 assert(!lx_pci_register_driver(&d)); assert(endpoint.linked);
 lx_pci_write32(0x12345678,bar); assert(lx_pci_read32(bar)==0x12345678);
 raw_fault=true; assert(lx_pci_read32(bar)==~0u); lx_pci_write32(0,bar); assert(bar[0]==0x12345678);
 assert(lx_pci_read32(bar+sizeof(bar)/sizeof(*bar))==~0u);
 run_poll(); lx_pci_unregister_driver(&d); assert(!endpoint.linked && !endpoint.vectors && unlinked);
 assert(!endpoint.enabled && !endpoint.region && !endpoint.dev.msi_cap && !endpoint.pcie_cap);
 assert(!endpoint.msi.msg.address_lo && !endpoint.msi.msg.address_hi);
}
int main(void)
{
 test_msi(); test_per_vector_mask(); test_failed_probe_retry(); test_failures(); test_handlers(); test_deferred(); test_registration_and_mmio();
 puts("pci-test: MSI32/single, enable order, allocation/config/readback failures, shared IRQ detach, masking, deferred work and teardown passed");
 return 0;
}
