/* PCI-only backend: retain LinuxKPI types, replace FreeBSD device methods. */
#ifndef _LX_PCI_H_
#define _LX_PCI_H_
#include_next <linux/pci.h>
#ifdef CONFIG_ATH11K_PCI
#include <linux/delay.h>
#ifndef SET_SYSTEM_SLEEP_PM_OPS
#define SET_SYSTEM_SLEEP_PM_OPS(s,r)
#define SET_LATE_SYSTEM_SLEEP_PM_OPS(s,r)
#endif
struct device_node;
struct resource;
struct device_node *of_find_node_by_type(struct device_node *, const char *);
int of_address_to_resource(struct device_node *, int, struct resource *);
#undef pci_enable_device
#define pci_enable_device lx_pci_enable_device
int  lx_pci_enable_device(struct pci_dev *p);
#undef pci_disable_device
#define pci_disable_device lx_pci_disable_device
void  lx_pci_disable_device(struct pci_dev *p);
#undef pci_is_enabled
#define pci_is_enabled lx_pci_is_enabled
int  lx_pci_is_enabled(struct pci_dev *p);
#undef pci_set_master
#define pci_set_master lx_pci_set_master
int  lx_pci_set_master(struct pci_dev *p);
#undef pci_assign_resource
#define pci_assign_resource lx_pci_assign_resource
int  lx_pci_assign_resource(struct pci_dev *p, int bar);
#undef pci_request_region
#define pci_request_region lx_pci_request_region
int  lx_pci_request_region(struct pci_dev *p, int bar, const char *name);
#undef pci_release_region
#define pci_release_region lx_pci_release_region
void  lx_pci_release_region(struct pci_dev *p, int bar);
#undef pci_resource_len
#define pci_resource_len lx_pci_resource_len
unsigned long  lx_pci_resource_len(struct pci_dev *p, int bar);
#undef pci_iomap
#define pci_iomap lx_pci_iomap
void * lx_pci_iomap(struct pci_dev *p, int bar, unsigned long maxlen);
#undef pci_iounmap
#define pci_iounmap lx_pci_iounmap
void  lx_pci_iounmap(struct pci_dev *p, void *va);
#undef pci_free_irq_vectors
#define pci_free_irq_vectors lx_pci_free_irq_vectors
void  lx_pci_free_irq_vectors(struct pci_dev *p);
#undef pci_irq_vector
#define pci_irq_vector lx_pci_irq_vector
int  lx_pci_irq_vector(struct pci_dev *p, unsigned vector);
#define pci_read_config_byte lx_pci_read_config_byte
int lx_pci_read_config_byte(const struct pci_dev *, int, u8 *);
#define pci_write_config_byte lx_pci_write_config_byte
int lx_pci_write_config_byte(const struct pci_dev *, int, u8);
#define pci_read_config_word lx_pci_read_config_word
int lx_pci_read_config_word(const struct pci_dev *, int, u16 *);
#define pci_write_config_word lx_pci_write_config_word
int lx_pci_write_config_word(const struct pci_dev *, int, u16);
#define pci_read_config_dword lx_pci_read_config_dword
int lx_pci_read_config_dword(const struct pci_dev *, int, u32 *);
#define pci_write_config_dword lx_pci_write_config_dword
int lx_pci_write_config_dword(const struct pci_dev *, int, u32);
#undef pcie_capability_read_word
#define pcie_capability_read_word lx_pcie_read_word
int lx_pcie_read_word(struct pci_dev *, int, u16 *);
#undef pcie_capability_clear_word
#define pcie_capability_clear_word(p,o,c) lx_pcie_change_word(p,o,c,0)
#undef pcie_capability_clear_and_set_word
#define pcie_capability_clear_and_set_word lx_pcie_change_word
int lx_pcie_change_word(struct pci_dev *, int, u16, u16);
#undef pci_register_driver
#define pci_register_driver lx_pci_register_driver
int lx_pci_register_driver(struct pci_driver *);
#undef pci_unregister_driver
#define pci_unregister_driver lx_pci_unregister_driver
void lx_pci_unregister_driver(struct pci_driver *);
#undef pci_alloc_irq_vectors
#define pci_alloc_irq_vectors lx_pci_alloc_irq_vectors
int lx_pci_alloc_irq_vectors(struct pci_dev *, int, int, unsigned);
#define dma_set_mask(d,m) lx_pci_dma_mask(d,m)
#define dma_set_coherent_mask(d,m) lx_pci_dma_mask(d,m)
int lx_pci_dma_mask(struct device *, u64);
int lx_pci_driver_init(void);
void lx_pci_driver_exit(void);
void *lx_pci_base(void);
void lx_pci_pool_reset(void);
u32 lx_pci_read32(const volatile void *);
void lx_pci_write32(u32, volatile void *);
#endif
#endif
