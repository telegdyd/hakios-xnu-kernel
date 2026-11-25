/* Host doubles for the Linux and kernel types lx_mhi.c uses. Only what the file touches. */
#ifndef MHI_TEST_FIXTURE_H
#define MHI_TEST_FIXTURE_H
#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t dma_addr_t;
typedef int gfp_t;

#define BIT(n) (1u << (n))
#define __iomem
#define __printflike(a, b) __attribute__((format(printf, a, b)))
#define GFP_KERNEL 0
#define THIS_MODULE ((struct module *)0)
#define module_driver(drv, reg, unreg)

enum dma_data_direction { DMA_BIDIRECTIONAL = 0, DMA_TO_DEVICE = 1, DMA_FROM_DEVICE = 2, DMA_NONE = 3 };

struct device {
	void *driver_data;
};
struct device_driver {
	const char *name;
};
struct sk_buff {
	unsigned char *data;
	unsigned len;
};

#define IRQF_SHARED 0x80ul
typedef enum { IRQ_NONE = 0, IRQ_HANDLED = 1, IRQ_WAKE_THREAD = 2 } irqreturn_t;
typedef irqreturn_t (*irq_handler_t)(int, void *);
int request_irq(unsigned irq, irq_handler_t handler, unsigned long flags, const char *name, void *arg);
void free_irq(unsigned irq, void *arg);
void enable_irq(unsigned irq);
void disable_irq(unsigned irq);

void *test_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma);
void test_dma_free(struct device *dev, size_t size, void *va, dma_addr_t dma);
#define dma_alloc_coherent(dev, size, dma, gfp) test_dma_alloc(dev, size, dma)
#define dma_free_coherent(dev, size, va, dma) test_dma_free(dev, size, va, dma)

static inline void *kzalloc(size_t n, int flags) { return calloc(1, n ? n : 1); }
static inline void kfree(const void *p) { free((void *)p); }
#endif
