/*
 * The phone's USB cable as a network: en2 over a CDC-NCM function, and the kernel's log on TCP 5140.
 *
 * Boot-arg usb_net=1 (the loader leaves USB alone while the kernel owns its core: the UEFI USB stack
 * only runs on that core). The kernel takes the DWC3 controller at 0xa600000 as the loader's UEFI left it
 * (clocks, PHY and power are already on), becomes a high-speed USB device and shows the host a network
 * adapter -- Windows has a driver for NCM in the box, Linux's cdc_ncm and macOS's too. The frames go to the
 * kernel's own interface (the "link" of sys/hakios.net.h) as en2; it is given 192.168.7.2/24 here, the host
 * 192.168.7.1/24 by hand. TCP 5140 then streams the console log: all of it so far, then new lines as they
 * come (devices/negroni/tools/usb-netlog.py).
 *
 * Ported from the POCO X3 Pro's bsd/dev/usbn.c and usbn_ecm.c, the same controller core.
 * Kept: the DWC3 bring-up (Linux's drivers/usb/dwc3 in order, polled from one thread: no interrupt is
 * attached), the control endpoint, the NCM function with its bulk rings, the log port. Not kept: the
 * read-only survey, the handover handshake, the DHCP server, the command port.
 */
#include <machine/machine_routines.h>
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/route.h>
#include <netinet/in.h>
#include <pexpert/pexpert.h>
#include <stdbool.h>
#include <sys/hakios.net.h>
#include <sys/kpi_mbuf.h>
#include <sys/kpi_socket.h>
#include <sys/sockio.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

#ifndef VM_WIMG_WCOMB
#define VM_WIMG_WCOMB           0x6     /* osfmk/arm/memory_types.h: normal memory, not cached */
#endif

#define USBNET_UNIT             2       /* en2, as on the X3: en0 is the radio, en1 the modem */

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern void IOSleep(unsigned int milliseconds);
extern void loader_log_putc(char c);
extern uint32_t loader_log_copy(uint64_t *pos, char *out, uint32_t max);
extern uint64_t kvtophys(vm_offset_t va);
extern void pmap_set_cache_attributes(ppnum_t pn, unsigned int cacheattr);

static int tee_printf(const char *format, ...) __printflike(1, 2);

static int
tee_printf(const char *format, ...)
{
	char line[320];
	va_list list;
	int length;

	va_start(list, format);
	length = vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
	if (ml_get_interrupts_enabled()) {        /* (not on the way out: loader_return runs with interrupts off) */
		(printf)("%s", line);
	}
	return length;
}
#define printf(...) tee_printf(__VA_ARGS__)

#define DWC3_PHYS               0x0a600000UL    /* dwc3@a600000 in the stock vendor_boot DTB */
#define DWC3_SIZE               0x100000UL

#define GSBUSCFG0               0xc100
#define GSBUSCFG1               0xc104
#define GCTL                    0xc110
#define GSTS                    0xc118
#define GUCTL1                  0xc11c
#define GSNPSID                 0xc120
#define GUID                    0xc128
#define GUCTL                   0xc12c
#define GDBGLTSSM               0xc164
#define GUCTL2                  0xc19c
#define GUSB2PHYCFG0            0xc200
#define GUSB3PIPECTL0           0xc2c0
#define GHWPARAMS0              0xc140
#define GHWPARAMS8              0xc600
#define GFLADJ                  0xc630
#define GTXFIFOSIZ(n)           (0xc300 + 4 * (n))
#define GRXFIFOSIZ(n)           (0xc380 + 4 * (n))
#define GEVNTADRLO(n)           (0xc400 + 16 * (n))
#define GEVNTADRHI(n)           (0xc404 + 16 * (n))
#define GEVNTSIZ(n)             (0xc408 + 16 * (n))
#define GEVNTCOUNT(n)           (0xc40c + 16 * (n))
#define DCFG                    0xc700
#define DCTL                    0xc704
#define DEVTEN                  0xc708
#define DSTS                    0xc70c
#define DGCMDPAR                0xc710
#define DGCMD                   0xc714
#define DALEPENA                0xc720
#define DEP(n, o)               (0xc800 + 16 * (n) + (o))

static vm_offset_t dwc3;

static inline uint32_t
rd(vm_offset_t base, vm_offset_t off)
{
	return *(volatile uint32_t *)(base + off);
}

uint64_t usbn_events(void);      /* events processed, for the half-minute line */

uint32_t usbn_rd(vm_offset_t offset);
void usbn_wr(vm_offset_t offset, uint32_t value);

/* One endpoint command, polled to completion (gadget.c:397 dwc3_send_gadget_ep_cmd).
 * Returns 0, or the command's status (bits 12-15) / 0x10 for "no completion in 10 ms". */
int usbn_ep_cmd(unsigned int phys, uint32_t cmd, uint32_t param0, uint32_t param1, uint32_t param2);
/* ... and with the transfer resource index a STARTTRANSFER is answered with (core.h:477). */
int usbn_ep_cmd_rsc(unsigned int phys, uint32_t cmd, uint32_t param0, uint32_t param1, uint32_t param2,
    uint32_t *rsc);
/* SETEPCONFIG (gadget.c:670). action: 0 = INIT, 2 = MODIFY (core.h's DEPCFG_ACTION, bits 30-31). */
int usbn_ep_config(unsigned int phys, uint32_t type, uint32_t maxpacket, uint32_t fifo, uint32_t binterval,
    bool xfer_in_progress, uint32_t action);

vm_offset_t usbn_dma_va(vm_offset_t offset);
uint64_t usbn_dma_pa(vm_offset_t offset);
void usbn_clean(vm_offset_t va, vm_size_t len);  /* dc cvac: ours written before the device reads */
void usbn_inval(vm_offset_t va, vm_size_t len);  /* dc civac: the device's written before we read */

/* One TRB: address, size, control (HWO last, as gadget.c:1173 requires), then cleaned. */
void usbn_trb(vm_offset_t trb, uint64_t buffer_pa, uint32_t size, uint32_t ctrl);
/* The SETUP TRB on ep0 OUT, armed again (ep0.c: dwc3_ep0_out_start) -- no state of the ECM side's. */
void usbn_setup_arm(void);

/* ---- the ECM side, for the event loop -------------------------------------------------------- */

void usbn_ecm_init(void);        /* level 3: en2 and its address, once the controller runs */
void usbn_ecm_ep_event(unsigned int phys, uint32_t event, uint32_t status, uint32_t param);
void usbn_ecm_disconnect(void);
void usbn_ecm_reset(void);
void usbn_ecm_connect(void);
void usbn_ecm_counters(void);

/* ---- registers the ECM side touches (the rest is usbn.c's) ----------------------------- */

#define USB_DCFG                0xc700  /* DCFG: SET_ADDRESS writes DEVADDR here (ep0.c:649) */
#define USB_DALEPENA            0xc720  /* DALEPENA: one bit per physical endpoint */
#define USB_DCFG_DEVADDR        (0x7fu << 3)

/* ---- TRB control, core.h:730-747 ------------------------------------------------------------- */

#define USB_TRB_HWO             (1u << 0)
#define USB_TRB_LST             (1u << 1)
#define USB_TRB_CHN             (1u << 2)
#define USB_TRB_CSP             (1u << 3)
#define USB_TRB_TYPE(n)         (((n) & 0x3f) << 4)
#define USB_TRB_ISP_IMI         (1u << 10)
#define USB_TRB_IOC             (1u << 11)

#define USB_TRBCTL_NORMAL       1
#define USB_TRBCTL_CONTROL_SETUP 2
#define USB_TRBCTL_CONTROL_STATUS2 3
#define USB_TRBCTL_CONTROL_STATUS3 4
#define USB_TRBCTL_CONTROL_DATA 5
#define USB_TRBCTL_LINK         8

struct usbn_trb {
	uint32_t bpl;
	uint32_t bph;
	uint32_t size;
	uint32_t ctrl;
};

/* ---- endpoint commands and their parameters, core.h:475-505, gadget.h:31-54 ------------------- */

#define USB_DEPCMD_DEPSTARTCFG      0x09
#define USB_DEPCMD_ENDTRANSFER      0x08
#define USB_DEPCMD_UPDATETRANSFER   0x07
#define USB_DEPCMD_STARTTRANSFER    0x06
#define USB_DEPCMD_CLEARSTALL       0x05
#define USB_DEPCMD_SETSTALL         0x04
#define USB_DEPCMD_SETTRANSFRESOURCE 0x02
#define USB_DEPCMD_SETEPCONFIG      0x01

#define USB_DEPCMD_CMDIOC       (1u << 8)
#define USB_DEPCMD_CMDACT       (1u << 10)
#define USB_DEPCMD_FORCERM      (1u << 11)      /* HIPRI_FORCERM, for ENDTRANSFER */
#define USB_DEPCMD_PARAM(n)     ((uint32_t)(n) << 16)
#define USB_DEPCMD_GET_RSC(n)   (((n) >> 16) & 0x7f)

#define USB_DEPCFG_TYPE_CONTROL 0
#define USB_DEPCFG_TYPE_BULK    2
#define USB_DEPCFG_TYPE_INTR    3

#define USB_DEPCFG_XFER_COMPLETE_EN     (1u << 8)
#define USB_DEPCFG_XFER_IN_PROGRESS_EN  (1u << 9)
#define USB_DEPCFG_XFER_NOT_READY_EN    (1u << 10)

/* Endpoint events, core.h:1257-1264. */
#define USB_DEPEVT_XFERCOMPLETE     1
#define USB_DEPEVT_XFERINPROGRESS   2
#define USB_DEPEVT_XFERNOTREADY     3
#define USB_DEPEVT_EPCMDCMPLT       7

/* ---- the DMA area (usbn.c allocates it; the arithmetic is there) ----------------------- */

#define USB_EVENT_SIZE          4096
#define USB_TRB_SLOTS           32      /* TRBs per bulk ring: 0-30 data, 31 the link TRB back to 0 */
#define USB_TRB_DATA            31      /* the data slots of each ring */
#define USB_FRAME_MAX           2048
#define USB_MAXPACKET           512

/*
 * TRBs 16 bytes apart, as the controller walks them (a ring or a chain is contiguous memory). The
 * whole area is made uncacheable for the CPU once (usbn.c dma_area(): pmap_set_cache_attributes,
 * VM_WIMG_WCOMB), so the CPU and the controller never disagree about a line holding several TRBs;
 * usbn_clean/inval are only barriers then.
 */
#define USB_TRB_STRIDE          16

#define USB_DMA_EVENTS          0x0000
#define USB_DMA_EP0_TRB         0x1000
#define USB_DMA_EP0_SETUP       0x1040
#define USB_DMA_EP0_DATA        0x1080
#define USB_DMA_OUT_TRB         0x1280      /* 32 TRBs */
#define USB_DMA_IN_TRB          0x1480      /* 32 TRBs */
#define USB_DMA_NOTIFY_TRB      0x1680
#define USB_DMA_NOTIFY_DATA     0x16c0
#define USB_DMA_OUT_DATA        0x2000      /* 32 x 2048 */
#define USB_DMA_IN_DATA         0x12000     /* 32 x 2048 */
#define USB_DMA_SIZE            0x22000

/* ---- the physical endpoints: phys = 2 * epnum + direction (IN = 1), gadget.c:700 --------------- */

#define USB_EP_EP0OUT           0
#define USB_EP_EP0IN            1
#define USB_EP_NOTIFY           3       /* EP1 IN: the CDC notifications, 16 bytes */
#define USB_EP_OUT              4       /* EP2 OUT: the host's frames */
#define USB_EP_IN               5       /* EP2 IN: ours */
/* The bits of the registers the sequence below reads and writes (drivers/usb/dwc3/core.h). */
#define DCTL_RUN_STOP           (1u << 31)
#define DCTL_CSFTRST            (1u << 30)
#define DSTS_DEVCTRLHLT         (1u << 22)
#define GCTL_PRTCAP_MASK        (3u << 12)
#define GCTL_PRTCAP_DEVICE      (2u << 12)
#define GUSB2PHYCFG_SUSPHY      (1u << 6)
#define GUSB2PHYCFG_ENBLSLPM    (1u << 8)
#define DCFG_SPEED_MASK         0x7u
#define DCFG_DEVADDR_MASK       USB_DCFG_DEVADDR
#define DEVTEN_DISCONNECT       (1u << 0)
#define DEVTEN_USBRST           (1u << 1)
#define DEVTEN_CONNECTDONE      (1u << 2)
#define DEVTEN_ULSTCNG          (1u << 3)
#define GEVNTSIZ_INTMASK        (1u << 31)

/* A device event's type and an endpoint event's fields (core.h:1270-1334). */
#define DEVICE_EVENT_TYPE(raw)  (((raw) >> 8) & 0xf)
#define DEPEVT_PHYS(raw)        (((raw) >> 1) & 0x1f)
#define DEPEVT_EVENT(raw)       (((raw) >> 6) & 0xf)
#define DEPEVT_STATUS(raw)      (((raw) >> 12) & 0xf)
#define DEPEVT_PARAM(raw)       ((raw) >> 16)


static uint32_t usb_ip = 0xc0a80702;    /* usb_net_ip=: 192.168.7.2, one uint32 */
static vm_offset_t usb_dma;             /* the rings: physically contiguous, uncached */
static uint64_t usb_dma_pa;
static uint32_t usb_event_at;           /* the running offset in event buffer 0 */
static uint64_t usb_events;             /* how many events have been processed */
static uint32_t usb_link_state = 0xffffffffu;  /* the last USBLNKST said */
static volatile bool usb_taken;         /* this kernel has the controller: quiesce has work */
static volatile bool usb_quiesced;      /* ... and must not be touched any more */
uint64_t
usbn_events(void)
{
	return usb_events;
}

uint32_t
usbn_rd(vm_offset_t offset)
{
	return rd(dwc3, offset);
}

void
usbn_wr(vm_offset_t offset, uint32_t value)
{
	*(volatile uint32_t *)(dwc3 + offset) = value;
}

/* Every write of the bring-up, in the log: what the loader's UEFI had set can be read from it. */
static void
reg_write_logged(const char *name, vm_offset_t offset, uint32_t value)
{
	uint32_t old = rd(dwc3, offset);

	*(volatile uint32_t *)(dwc3 + offset) = value;
	printf("usb_net: w %s %08x -> %08x\n", name, old, value);
}

/*
 * The rings. The task's sketch said 64 KB for the whole area, but 32 x 2048-byte buffers in each
 * direction is 128 KB by itself, so the block is what the layout in usbn.h adds up to
 * (USB_DMA_SIZE): one physically contiguous kmem_alloc_contig() piece, because the
 * controller is given physical addresses.
 */
static bool
dma_area(void)
{
	vm_size_t size = round_page(USB_DMA_SIZE);
	uint64_t pa;

	if (kmem_alloc_contig(kernel_map, &usb_dma, size, PAGE_MASK, 0, 0,
	    KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("usb_net: no %lu contiguous bytes for the rings\n", (unsigned long)size);
		return false;
	}
	pa = kvtophys(usb_dma);
	if (pa == 0) {
		printf("usb_net: the rings at %#lx have no physical address\n", (unsigned long)usb_dma);
		return false;
	}
	for (vm_offset_t offset = 0; offset < size; offset += PAGE_SIZE) {
		if (kvtophys(usb_dma + offset) != pa + offset) {
			printf("usb_net: the rings are not contiguous: page %lu at %#llx, expected %#llx\n",
			    (unsigned long)(offset >> PAGE_SHIFT), kvtophys(usb_dma + offset), pa + offset);
			return false;
		}
	}
	usb_dma_pa = pa;
	/*
	 * Uncacheable for the CPU from here on (VM_WIMG_WCOMB: normal memory, not cached), every mapping
	 * of these pages, the physical aperture's included (pmap.c pmap_update_cache_attributes_locked; the
	 * aperture is page-granular for exactly this, arm_vm_init.c arm_vm_physmap_init): TRBs 16 bytes
	 * apart share cache lines with each other, and a write-back of one would undo what the controller
	 * wrote into its neighbour. The zeroing's dirty lines go to memory first.
	 */
	for (vm_offset_t line = usb_dma; line < usb_dma + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	for (vm_offset_t offset = 0; offset < size; offset += PAGE_SIZE) {
		pmap_set_cache_attributes((ppnum_t)atop(pa + offset), VM_WIMG_WCOMB);
	}
	printf("usb_net: the rings: %lu bytes at %#lx, physical %#llx, uncached\n", (unsigned long)size,
	    (unsigned long)usb_dma, pa);
	return true;
}

vm_offset_t
usbn_dma_va(vm_offset_t offset)
{
	return usb_dma + offset;
}

uint64_t
usbn_dma_pa(vm_offset_t offset)
{
	return usb_dma_pa + offset;
}

/*
 * The area is uncached (dma_area()), so what is left of "clean" and "invalidate" is the ordering:
 * our stores out before the controller is told (a doorbell register write), the controller's
 * writes seen before we read on.
 */
void
usbn_clean(__unused vm_offset_t va, __unused vm_size_t len)
{
	__asm__ volatile ("dsb sy" ::: "memory");
}

void
usbn_inval(__unused vm_offset_t va, __unused vm_size_t len)
{
	__asm__ volatile ("dsb sy" ::: "memory");
}

void
usbn_trb(vm_offset_t trb, uint64_t buffer_pa, uint32_t size, uint32_t ctrl)
{
	struct usbn_trb *entry = (struct usbn_trb *)usbn_dma_va(trb);

	entry->bpl = (uint32_t)buffer_pa;
	entry->bph = (uint32_t)(buffer_pa >> 32);
	entry->size = size;
	/* HWO last: the core must not fetch half a TRB (gadget.c:1173); uncached normal memory may
	 * reorder stores, hence the barrier */
	__asm__ volatile ("dmb oshst" ::: "memory");
	entry->ctrl = ctrl;
	usbn_clean((vm_offset_t)entry, sizeof(*entry));
}

/*
 * One endpoint command, polled to completion: gadget.c:397 dwc3_send_gadget_ep_cmd. In high speed
 * SUSPHY and ENBLSLPM hold endpoint commands off, so they are lifted for the command and put back
 * after it (gadget.c:411-436). The resource index a STARTTRANSFER is answered with is what
 * UPDATETRANSFER and ENDTRANSFER want back (core.h:475-505).
 */
int
usbn_ep_cmd(unsigned int phys, uint32_t cmd, uint32_t param0, uint32_t param1, uint32_t param2)
{
	return usbn_ep_cmd_rsc(phys, cmd, param0, param1, param2, NULL);
}

int
usbn_ep_cmd_rsc(unsigned int phys, uint32_t cmd, uint32_t param0, uint32_t param1, uint32_t param2,
    uint32_t *rsc)
{
	uint32_t cfg = rd(dwc3, GUSB2PHYCFG0), saved = 0, reg = 0;
	bool done = false;

	if ((cfg & GUSB2PHYCFG_SUSPHY) != 0) {
		saved |= GUSB2PHYCFG_SUSPHY;
		cfg &= ~GUSB2PHYCFG_SUSPHY;
	}
	if ((cfg & GUSB2PHYCFG_ENBLSLPM) != 0) {
		saved |= GUSB2PHYCFG_ENBLSLPM;
		cfg &= ~GUSB2PHYCFG_ENBLSLPM;
	}
	if (saved != 0) {
		usbn_wr(GUSB2PHYCFG0, cfg);
	}
	usbn_wr(DEP(phys, 8), param0);
	usbn_wr(DEP(phys, 4), param1);
	usbn_wr(DEP(phys, 0), param2);
	if ((cmd & 0xf) == USB_DEPCMD_UPDATETRANSFER) {
		/* the no-response form (gadget.c:452): the TRB cache is updated, nothing is waited for */
		usbn_wr(DEP(phys, 0xc), cmd & ~USB_DEPCMD_CMDIOC);
		if (saved != 0) {
			usbn_wr(GUSB2PHYCFG0, cfg | saved);
		}
		if (rsc != NULL) {
			*rsc = 0;
		}
		return 0;
	}
	usbn_wr(DEP(phys, 0xc), cmd | USB_DEPCMD_CMDACT);
	for (unsigned int i = 0; i < 100; i++) {        /* 100 x 100 us: 10 ms */
		reg = rd(dwc3, DEP(phys, 0xc));
		if ((reg & USB_DEPCMD_CMDACT) == 0) {
			done = true;
			break;
		}
		delay(100);
	}
	if (saved != 0) {
		usbn_wr(GUSB2PHYCFG0, cfg | saved);
	}
	if (!done) {
		printf("usb_net: ep%u: command %#x does not complete in 10 ms\n", phys, cmd & 0xf);
		return 0x10;
	}
	if (rsc != NULL) {
		*rsc = USB_DEPCMD_GET_RSC(reg);
	}
	if (((reg >> 12) & 0xf) != 0) {
		printf("usb_net: ep%u: command %#x: status %u\n", phys, cmd & 0xf, (reg >> 12) & 0xf);
		return (int)((reg >> 12) & 0xf);
	}
	return 0;
}

int
usbn_ep_config(unsigned int phys, uint32_t type, uint32_t maxpacket, uint32_t fifo, uint32_t binterval,
    bool xfer_in_progress, uint32_t action)
{
	uint32_t param0, param1;

	param0 = ((type & 3) << 1) | ((maxpacket & 0x7ff) << 3) | ((fifo & 0x1f) << 17) | ((action & 3) << 30);
	param1 = USB_DEPCFG_XFER_COMPLETE_EN | USB_DEPCFG_XFER_NOT_READY_EN | ((phys & 0x1f) << 25);
	if (xfer_in_progress) {
		param1 |= USB_DEPCFG_XFER_IN_PROGRESS_EN;
	}
	if (binterval != 0) {
		param1 |= ((binterval - 1) & 0xff) << 16;
	}
	return usbn_ep_cmd(phys, USB_DEPCMD_SETEPCONFIG, param0, param1, 0);
}

void
usbn_setup_arm(void)
{
	uint64_t trb = usbn_dma_pa(USB_DMA_EP0_TRB);

	usbn_trb(USB_DMA_EP0_TRB, usbn_dma_pa(USB_DMA_EP0_SETUP), 8,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_ISP_IMI | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_CONTROL_SETUP));
	if (usbn_ep_cmd(USB_EP_EP0OUT, USB_DEPCMD_STARTTRANSFER, (uint32_t)(trb >> 32), (uint32_t)trb, 0) != 0) {
		printf("usb_net: ep0: the SETUP transfer cannot be started\n");
	}
}

static bool
wait_bits(vm_offset_t offset, uint32_t mask, bool set, unsigned int ms)
{
	for (unsigned int i = 0; i < ms; i++) {
		if (((rd(dwc3, offset) & mask) != 0) == set) {
			return true;
		}
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	return false;
}
static bool
dwc3_endpoints(void)
{
	/* (gadget.c:639 dwc3_gadget_start_config: XferRscIdx 0, then a resource for every ep) */
	if (usbn_ep_cmd(USB_EP_EP0OUT, USB_DEPCMD_DEPSTARTCFG, 0, 0, 0) != 0) {
		printf("usb_net: DEPSTARTCFG failed\n");
		return false;
	}
	/* EP0 OUT and IN: control, 64-byte packets (dwc3_gadget_set_ep_config, gadget.c:670) */
	if (usbn_ep_config(USB_EP_EP0OUT, USB_DEPCFG_TYPE_CONTROL, 64, 0, 0, false, 0) != 0 ||
	    usbn_ep_config(USB_EP_EP0IN, USB_DEPCFG_TYPE_CONTROL, 64, 0, 0, false, 0) != 0) {
		printf("usb_net: ep0 cannot be configured\n");
		return false;
	}
	/* one transfer resource per endpoint, once until the next core reset (gadget.c:742-752): ep0 and
	 * the ECM function's 3, 4, 5 (and 2, unused, so that the indices stay in order) */
	for (unsigned int phys = USB_EP_EP0OUT; phys <= USB_EP_IN; phys++) {
		if (usbn_ep_cmd(phys, USB_DEPCMD_SETTRANSFRESOURCE, 1, 0, 0) != 0) {
			printf("usb_net: ep%u has no transfer resource\n", phys);
			return false;
		}
	}
	reg_write_logged("DALEPENA", USB_DALEPENA,
	    usbn_rd(USB_DALEPENA) | (1u << USB_EP_EP0OUT) | (1u << USB_EP_EP0IN));
	return true;
}

/*
 * The sequence: stop, reset, device mode, event buffer, high speed, endpoints, SETUP, events, run.
 * Linux's drivers/usb/dwc3 in the same order (the line numbers are in each step).
 */
static bool
dwc3_take(void)
{
	uint32_t gctl, dcfg;

	/*
	 * (1) gadget.c:2150 dwc3_gadget_run_stop(is_on = 0): UEFI's transfers ended first (the data book
	 * wants no active transfer when RUN_STOP drops, gadget.c:2200), then RUN_STOP, then UEFI's events
	 * acknowledged until the core halts: "Controller is not halted until the events are acknowledged"
	 * (gadget.c:2216). Run 051220 (usb2): without that, DEVCTRLHLT never came.
	 */
	reg_write_logged("DEVTEN", DEVTEN, 0);
	for (unsigned int phys = 0; phys < 16; phys++) {
		uint32_t rsc = (rd(dwc3, DEP(phys, 0xc)) >> 16) & 0x7f;     /* the last STARTTRANSFER's index */

		if ((rd(dwc3, DALEPENA) & (1u << phys)) == 0) {
			continue;
		}
		printf("usb_net: ep%u (UEFI's): ENDTRANSFER rsc %u: %d\n", phys, rsc, usbn_ep_cmd(phys,
		    USB_DEPCMD_ENDTRANSFER | USB_DEPCMD_FORCERM | USB_DEPCMD_CMDIOC | USB_DEPCMD_PARAM(rsc), 0, 0, 0));
		delay(100);
	}
	reg_write_logged("DCTL", DCTL, rd(dwc3, DCTL) & ~DCTL_RUN_STOP);
	for (unsigned int ms = 0; ms < 500 && (rd(dwc3, DSTS) & DSTS_DEVCTRLHLT) == 0; ms++) {
		uint32_t pending = rd(dwc3, GEVNTCOUNT(0)) & 0xfffc;

		if (pending != 0) {
			usbn_wr(GEVNTCOUNT(0), pending);      /* (UEFI's events, unread: acknowledged) */
		}
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	printf("usb_net: DSTS %08x, GEVNTCOUNT0 %08x after the stop\n", rd(dwc3, DSTS), rd(dwc3, GEVNTCOUNT(0)));
	if ((rd(dwc3, DSTS) & DSTS_DEVCTRLHLT) == 0) {
		/* (UEFI's transfers may still be active; the core soft reset below stops the device anyway) */
		printf("usb_net: DSTS.DEVCTRLHLT does not come up in 500 ms; soft reset anyway\n");
	}

	/* (2) core.c:206 dwc3_core_soft_reset */
	reg_write_logged("DCTL", DCTL, rd(dwc3, DCTL) | DCTL_CSFTRST);
	if (!wait_bits(DCTL, DCTL_CSFTRST, false, 100)) {
		printf("usb_net: DCTL.CSFTRST does not clear\n");
		return false;
	}
	/* the data book's PHY sync delay after a soft reset (core.c:206, then msleep(50)) */
	delay_for_interval(50, NSEC_PER_MSEC);

	/* (3) device mode; GCTL, GUSB2PHYCFG0 and GUSB3PIPECTL0 otherwise as UEFI left them */
	gctl = rd(dwc3, GCTL);
	if ((gctl & GCTL_PRTCAP_MASK) != GCTL_PRTCAP_DEVICE) {
		reg_write_logged("GCTL", GCTL, (gctl & ~GCTL_PRTCAP_MASK) | GCTL_PRTCAP_DEVICE);
	}
	printf("usb_net: GCTL %08x, GUSB2PHYCFG0 %08x\n", rd(dwc3, GCTL), rd(dwc3, GUSB2PHYCFG0));

	/* (4) event buffer 0 (core.c: dwc3_event_buffers_setup); 1..3 only masked away */
	reg_write_logged("GEVNTADRLO0", GEVNTADRLO(0), (uint32_t)usbn_dma_pa(USB_DMA_EVENTS));
	reg_write_logged("GEVNTADRHI0", GEVNTADRHI(0), (uint32_t)(usbn_dma_pa(USB_DMA_EVENTS) >> 32));
	/* (INTMASK: no interrupt is routed; the controller writes events all the same) */
	reg_write_logged("GEVNTSIZ0", GEVNTSIZ(0), USB_EVENT_SIZE | GEVNTSIZ_INTMASK);
	reg_write_logged("GEVNTCOUNT0", GEVNTCOUNT(0), 0);
	for (unsigned int n = 1; n < 4; n++) {
		char name[16];

		if (rd(dwc3, GEVNTSIZ(n)) == 0) {
			continue;
		}
		snprintf(name, sizeof(name), "GEVNTSIZ%u", n);
		reg_write_logged(name, GEVNTSIZ(n), rd(dwc3, GEVNTSIZ(n)) | GEVNTSIZ_INTMASK);
	}

	/* (5) high speed, address 0 (gadget.c:2165, the run_stop's DCFG) */
	dcfg = rd(dwc3, DCFG) & ~(DCFG_SPEED_MASK | DCFG_DEVADDR_MASK);
	reg_write_logged("DCFG", DCFG, dcfg);

	/* (6) the endpoints */
	if (!dwc3_endpoints()) {
		return false;
	}
	/* the loader's FIFOs (GTXFIFOSIZ(0) is ep0's): used as they are, not resized */
	printf("usb_net: GTXFIFOSIZ 1 %08x, 2 %08x%s\n", rd(dwc3, GTXFIFOSIZ(1)), rd(dwc3, GTXFIFOSIZ(2)),
	    rd(dwc3, GTXFIFOSIZ(1)) != 0 && rd(dwc3, GTXFIFOSIZ(2)) != 0 ? "" :
	    ": NOT ALLOCATED, the notify and bulk IN endpoints will not move");

	/* (7) one SETUP TRB waiting (ep0.c: dwc3_ep0_out_start) */
	usbn_setup_arm();

	/* (8) the events the loop wants (core.h:405-416) */
	reg_write_logged("DEVTEN", DEVTEN, DEVTEN_DISCONNECT | DEVTEN_USBRST | DEVTEN_CONNECTDONE | DEVTEN_ULSTCNG);

	/* (9) and run (gadget.c:2185) */
	reg_write_logged("DCTL", DCTL, rd(dwc3, DCTL) | DCTL_RUN_STOP);
	if (!wait_bits(DSTS, DSTS_DEVCTRLHLT, false, 100)) {
		printf("usb_net: DSTS.DEVCTRLHLT does not clear: the controller does not run\n");
		return false;
	}
	printf("usb_net: running as a USB device (ECM), waiting for the host\n");
	return true;
}
/* ---- the event buffer, one event at a time --------------------------------------------------- */

static void
event_one(uint32_t raw)
{
	if ((raw & 1) != 0) {                                   /* a device event (core.h:1327) */
		switch (DEVICE_EVENT_TYPE(raw)) {
		case 0:
			printf("usb_net: the host is gone (disconnect)\n");
			usbn_ecm_disconnect();
			break;
		case 1:
			printf("usb_net: USB reset\n");
			usbn_wr(USB_DCFG, usbn_rd(USB_DCFG) & ~DCFG_DEVADDR_MASK);
			usbn_ecm_reset();
			break;
		case 2: {
			static const char *const speed[] = { "high", "full", "low", "?", "super" };
			uint32_t dsts = rd(dwc3, DSTS);

			printf("usb_net: connect done: %s speed (DSTS %08x)\n",
			    speed[(dsts & 7) < 5 ? (dsts & 7) : 3], dsts);
			usbn_ecm_connect();
			break;
		}
		case 3: {
			uint32_t state = (rd(dwc3, DSTS) >> 18) & 0xf;

			/* (once per change, as gadget.c:3660 keeps dwc->link_state) */
			if (state != usb_link_state) {
				usb_link_state = state;
				printf("usb_net: link state %u\n", state);
			}
			break;
		}
		default:
			printf("usb_net: device event %u\n", DEVICE_EVENT_TYPE(raw));
			break;
		}
		return;
	}
	switch (DEPEVT_EVENT(raw)) {
	case USB_DEPEVT_XFERCOMPLETE:
	case USB_DEPEVT_XFERINPROGRESS:
	case USB_DEPEVT_XFERNOTREADY:
		usbn_ecm_ep_event(DEPEVT_PHYS(raw), DEPEVT_EVENT(raw), DEPEVT_STATUS(raw), DEPEVT_PARAM(raw));
		break;
	case USB_DEPEVT_EPCMDCMPLT:
		break;
	default:
		printf("usb_net: endpoint %u event %u\n", DEPEVT_PHYS(raw), DEPEVT_EVENT(raw));
		break;
	}
}

/*
 * The loop: every millisecond while anything has happened in the last second, every five when it
 * has been quiet (a quiet controller costs one read of GEVNTCOUNT). No interrupt is attached, as in
 * hakios.ufshc.c and hakios.net.c.
 */
static void
event_loop(void)
{
	uint32_t quiet = 0, said = 0;

	for (;;) {
		uint32_t count = rd(dwc3, GEVNTCOUNT(0)) & 0xfffc;
		uint32_t interval;

		if (usb_quiesced) {
			thread_terminate(current_thread());
		}
		if (count != 0) {
			uint32_t done = 0;

			while (done < count) {
				uint32_t *event = (uint32_t *)(usb_dma + USB_DMA_EVENTS + usb_event_at);

				usbn_inval((vm_offset_t)event, sizeof(*event));
				event_one(*event);
				usb_event_at = (usb_event_at + 4) % USB_EVENT_SIZE;
				done += 4;
				usb_events++;
			}
			usbn_wr(GEVNTCOUNT(0), done);
			quiet = 0;
		} else if (quiet < 1000) {
			quiet++;
		}
		interval = (count != 0 || quiet < 1000) ? 1 : 5;
		said += interval;
		if (said >= 30000) {
			said = 0;
			usbn_ecm_counters();
		}
		delay_for_interval(interval, NSEC_PER_MSEC);
	}
}
/* Into the loader's log only: quiesce runs with interrupts off, where printf panics ("Console I/O from
 * interrupt-disabled context", run 051220). */
static void
quiet_log(const char *format, ...)
{
	char line[200];
	va_list list;

	va_start(list, format);
	vsnprintf(line, sizeof(line), format, list);
	va_end(list);
	for (const char *c = line; *c != 0; c++) {
		loader_log_putc(*c);
	}
}

/*
 * On the way back to the loader (loader_return.c), interrupts off: no sleeping, no printf. Stopped as
 * in dwc3_take() (events acknowledged until the halt); the loader restarts its own USB stack after the
 * run anyway (stop + start_ex).
 */
void
hakios_usb_net_quiesce(void)
{
	unsigned int ms;

	if (!usb_taken || dwc3 == 0) {
		return;
	}
	usb_quiesced = true;
	usbn_wr(DEVTEN, 0);
	usbn_wr(DCTL, rd(dwc3, DCTL) & ~DCTL_RUN_STOP);
	for (ms = 0; ms < 100 && (rd(dwc3, DSTS) & DSTS_DEVCTRLHLT) == 0; ms++) {
		uint32_t pending = rd(dwc3, GEVNTCOUNT(0)) & 0xfffc;

		if (pending != 0) {
			usbn_wr(GEVNTCOUNT(0), pending);
		}
		delay(1000);
	}
	usbn_wr(GEVNTSIZ(0), rd(dwc3, GEVNTSIZ(0)) | GEVNTSIZ_INTMASK);
	quiet_log("usb_net: quiesce: RUN_STOP 0, %s after %u ms (usb_take stays 1)\n",
	    (rd(dwc3, DSTS) & DSTS_DEVCTRLHLT) != 0 ? "halted" : "STILL RUNNING", ms);
}

/* ---- the NCM function ------------------------------------------------------------------------ */


/* ---- the descriptors: high speed, CDC-ECM, as Linux's f_ecm.c builds them --------------------- */

#define USB_DT_DEVICE           1
#define USB_DT_CONFIG           2
#define USB_DT_STRING           3
#define USB_DT_INTERFACE        4
#define USB_DT_ENDPOINT         5
#define USB_DT_CS_INTERFACE     0x24

#define USB_CLASS_CDC           2
#define USB_CLASS_CDC_DATA      0x0a
#define USB_CDC_SUBCLASS_ECM    6
#define USB_CDC_SUBCLASS_NCM    0x0d
#define USB_DT_IAD              0x0b

#define USB_ENDPOINT_XFER_BULK  2
#define USB_ENDPOINT_XFER_INT   3

/*
 * bLength 18, DEVICE; bcdUSB 2.00; class 2 (CDC) subclass 0 protocol 0; 64-byte packets; the
 * loader's vendor 0x18d1 with a product of our own (0xd00e, not its 0xd00d); 1.00; three strings,
 * one configuration.
 */
static const uint8_t usbnet_device[] = {
	18, USB_DT_DEVICE,
	0x00, 0x02,
	USB_CLASS_CDC, 0, 0,
	64,
	0xd1, 0x18,
	0x0e, 0xd0,
	0x00, 0x01,
	1, 2, 3,
	1,
};

/*
 * One configuration, two interfaces: 0 is the CDC control interface (header, union, Ethernet
 * networking, one interrupt IN endpoint), 1 has alternate 0 with no endpoints and alternate 1 with
 * the two bulk ones. wTotalLength (bytes 2-3) is filled in from sizeof at the copy below.
 */
static const uint8_t usbnet_config[] = {
	/* configuration: 80 bytes, 2 interfaces, value 1, 0xc0 (self-powered), 250 = 500 mA */
	9, USB_DT_CONFIG, 0, 0, 2, 1, 0, 0xc0, 250,
	/* interface 0: CDC ECM, one endpoint */
	9, USB_DT_INTERFACE, 0, 0, 1, USB_CLASS_CDC, USB_CDC_SUBCLASS_ECM, 0, 0,
	/* CDC header: bcdCDC 1.10 */
	5, USB_DT_CS_INTERFACE, 0x00, 0x10, 0x01,
	/* CDC union: interface 0 controls interface 1 */
	5, USB_DT_CS_INTERFACE, 0x06, 0, 1,
	/* Ethernet networking: MAC at string 4, no statistics, 1514-byte frames, no filters */
	13, USB_DT_CS_INTERFACE, 0x0f, 4, 0, 0, 0, 0, 0xea, 0x05, 0, 0, 0,
	/* EP 1 IN: the notifications, interrupt, 16 bytes, every 32 ms in high speed */
	7, USB_DT_ENDPOINT, 0x81, USB_ENDPOINT_XFER_INT, 16, 0, 9,
	/* interface 1 alternate 0: no endpoints */
	9, USB_DT_INTERFACE, 1, 0, 0, USB_CLASS_CDC_DATA, 0, 0, 0,
	/* interface 1 alternate 1: the frames, 512-byte packets both ways */
	9, USB_DT_INTERFACE, 1, 1, 2, USB_CLASS_CDC_DATA, 0, 0, 0,
	7, USB_DT_ENDPOINT, 0x82, USB_ENDPOINT_XFER_BULK, 0, 2, 0,
	7, USB_DT_ENDPOINT, 0x02, USB_ENDPOINT_XFER_BULK, 0, 2, 0,
};

/*
 * NCM: the same function behind an interface association (device class 0xef/2/1, which Windows wants
 * before it gives a function of two interfaces to one driver), its own product number (0xd00f: so that
 * a host that met the ECM one does not reuse what it decided then), subclass 0x0d, the NCM functional
 * descriptor (1.00, no optional requests) and protocol 1 on the data interface. The layout of Linux's
 * f_ncm.c and of ios-jasmine's sweet_usb; the endpoints are the ECM ones.
 */
static const uint8_t usbnet_device_ncm[] = {
	18, USB_DT_DEVICE,
	0x00, 0x02,
	0xef, 0x02, 0x01,
	64,
	0xd1, 0x18,
	0x0f, 0xd0,
	0x00, 0x01,
	1, 2, 3,
	1,
};

static const uint8_t usbnet_config_ncm[] = {
	9, USB_DT_CONFIG, 0, 0, 2, 1, 0, 0xc0, 250,
	/* interface association: interfaces 0-1, CDC NCM */
	8, USB_DT_IAD, 0, 2, USB_CLASS_CDC, USB_CDC_SUBCLASS_NCM, 0, 0,
	9, USB_DT_INTERFACE, 0, 0, 1, USB_CLASS_CDC, USB_CDC_SUBCLASS_NCM, 0, 0,
	5, USB_DT_CS_INTERFACE, 0x00, 0x10, 0x01,
	5, USB_DT_CS_INTERFACE, 0x06, 0, 1,
	13, USB_DT_CS_INTERFACE, 0x0f, 4, 0, 0, 0, 0, 0xea, 0x05, 0, 0, 0,
	/* NCM functional descriptor: bcdNcmVersion 1.00, bmNetworkCapabilities 0 */
	6, USB_DT_CS_INTERFACE, 0x1a, 0x00, 0x01, 0x00,
	7, USB_DT_ENDPOINT, 0x81, USB_ENDPOINT_XFER_INT, 16, 0, 9,
	9, USB_DT_INTERFACE, 1, 0, 0, USB_CLASS_CDC_DATA, 0, 0x01, 0,
	9, USB_DT_INTERFACE, 1, 1, 2, USB_CLASS_CDC_DATA, 0, 0x01, 0,
	7, USB_DT_ENDPOINT, 0x82, USB_ENDPOINT_XFER_BULK, 0, 2, 0,
	7, USB_DT_ENDPOINT, 0x02, USB_ENDPOINT_XFER_BULK, 0, 2, 0,
};

static bool usbnet_ncm = true;          /* usbnet_ecm=1: false */

/* NTB-16 (CDC NCM 1.0, 3.2-3.3): little-endian fields */
#define NCM_NTB_MAX             2048    /* = USB_FRAME_MAX: one block, one TRB */
#define NCM_NTH16_SIG           0x484d434eu     /* "NCMH" */
#define NCM_NDP16_SIG           0x304d434eu     /* "NCM0": no CRC */
#define NCM_NDP16_SIG_CRC       0x314d434eu     /* "NCM1": each frame followed by its CRC */
#define NCM_TX_HDR              28      /* NTH16 12 + NDP16 8 + one entry 4 + the terminating entry 4 */

static inline uint32_t ld16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static inline uint32_t ld32(const uint8_t *p) { return ld16(p) | ld16(p + 2) << 16; }
static inline void st16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void st32(uint8_t *p, uint32_t v) { st16(p, v); st16(p + 2, v >> 16); }

/* Strings 1..4, and 0 the language: UTF-16LE in the wire format, built into the caller's buffer. */
static uint32_t
usbnet_string(uint32_t index, uint8_t *out)
{
	static const char *const strings[] = { "XNU", "vayu usbnet", "vayu0", "02A0B0C0D001" };
	const char *text;
	uint32_t length;

	if (index == 0) {
		out[0] = 4;
		out[1] = USB_DT_STRING;
		out[2] = 0x09;                  /* LANGID 0x0409: English, United States */
		out[3] = 0x04;
		return 4;
	}
	text = strings[index - 1];
	length = (uint32_t)strlen(text);
	out[0] = (uint8_t)(2 + 2 * length);
	out[1] = USB_DT_STRING;
	for (uint32_t i = 0; i < length; i++) {
		out[2 + 2 * i] = (uint8_t)text[i];
		out[3 + 2 * i] = 0;
	}
	return 2 + 2 * length;
}

/* ---- the control endpoint -------------------------------------------------------------------- */

struct usbnet_setup {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

/* EP0_ARMED first: a zeroed state is "the SETUP TRB is waiting", which is where a reset leaves it. */
enum {
	EP0_ARMED,
	EP0_DATA_IN,            /* the answer is in the buffer, waiting for XferNotReady(DATA) */
	EP0_DATA_SENT,          /* the data stage is the controller's, waiting for its completion */
	EP0_STATUS_IN,          /* a request with no data: waiting for XferNotReady(STATUS) on ep0 IN */
	EP0_STATUS_IN_SENT,
	EP0_STATUS_OUT,         /* after a data stage: waiting for XferNotReady(STATUS) on ep0 OUT */
	EP0_STATUS_OUT_SENT,
	EP0_DATA_OUT_SENT,      /* a control write: the host's data is the controller's to receive */
	EP0_STATUS_IN3,         /* ... then the status, an IN, three-stage: waiting for XferNotReady(STATUS) */
};

static uint32_t ep0_state;
static uint32_t ep0_length;             /* bytes the data stage carries */
static uint32_t ep0_wlength;            /* the request's wLength */
static uint32_t ep0_alt;                /* interface 1's alternate setting */
static bool ep0_configured;
static int ep0_active = -1;             /* the ep0 direction with a transfer under way (phys), or -1 */
static uint32_t ep0_rsc;

/* STARTTRANSFER of the one ep0 TRB on phys, noted so that a reset can end it (gadget.c:3360). */
static int
ep0_start(unsigned int phys)
{
	uint64_t trb = usbn_dma_pa(USB_DMA_EP0_TRB);
	int error;

	error = usbn_ep_cmd_rsc(phys, USB_DEPCMD_STARTTRANSFER, (uint32_t)(trb >> 32), (uint32_t)trb, 0, &ep0_rsc);
	ep0_active = error == 0 ? (int)phys : -1;
	return error;
}

/* A control transfer cut short by a reset or a disconnect: end it before ep0 is used again. */
static void
ep0_end(void)
{
	if (ep0_active < 0) {
		return;
	}
	usbn_ep_cmd((unsigned int)ep0_active, USB_DEPCMD_ENDTRANSFER | USB_DEPCMD_FORCERM | USB_DEPCMD_CMDIOC |
	    USB_DEPCMD_PARAM(ep0_rsc), 0, 0, 0);
	delay(100);                     /* (gadget.c:3247: the core needs a moment after CMDACT clears) */
	ep0_active = -1;
}

static void
ep0_restart(void)
{
	ep0_state = EP0_ARMED;
	ep0_active = -1;                /* (the SETUP TRB: a reset leaves it armed, ep0.c) */
	usbn_setup_arm();
}

static void ep0_data_stage(void);

/* No data stage: the status stage is an IN on ep0 (ep0.c: dwc3_ep0_start_control_status). */
static void
ep0_status_only(void)
{
	ep0_state = EP0_STATUS_IN;
}

/* The IN data stage: length bytes from src (or, with src == NULL, what is already in the buffer). */
static void
ep0_data(const void *src, uint32_t length)
{
	uint32_t len = length < ep0_wlength ? length : ep0_wlength;

	if (len == 0) {
		ep0_status_only();      /* (wLength 0: the host wants no data, only the status) */
		return;
	}
	if (src != NULL) {
		memcpy((void *)usbn_dma_va(USB_DMA_EP0_DATA), src, len);
	}
	usbn_clean(usbn_dma_va(USB_DMA_EP0_DATA), len);
	ep0_length = len;
	ep0_state = EP0_DATA_IN;
	/* started at once: the core starts on demand only the status stage (ep0.c __dwc3_gadget_ep0_queue) */
	ep0_data_stage();
}

static void
ep0_data_stage(void)
{
	usbn_trb(USB_DMA_EP0_TRB, usbn_dma_pa(USB_DMA_EP0_DATA), ep0_length,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_ISP_IMI | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_CONTROL_DATA));
	if (ep0_start(USB_EP_EP0IN) != 0) {
		printf("usb_net: ep0: the data stage cannot be started\n");
		ep0_restart();
		return;
	}
	ep0_state = EP0_DATA_SENT;
}

/*
 * The OUT data stage of a control write (NCM's SET_NTB_INPUT_SIZE, SET_MAX_DATAGRAM_SIZE): one 64-byte
 * packet's room, a multiple of the packet size as DWC3 wants for ep0 OUT (ep0.c bounces what is not).
 * What arrives is not used: the sizes the host may set are no larger than what this side sends anyway.
 */
static void
ep0_data_out(void)
{
	usbn_trb(USB_DMA_EP0_TRB, usbn_dma_pa(USB_DMA_EP0_DATA), 64,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_ISP_IMI | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_CONTROL_DATA));
	if (ep0_start(USB_EP_EP0OUT) != 0) {
		printf("usb_net: ep0: the OUT data stage cannot be started\n");
		ep0_restart();
		return;
	}
	ep0_state = EP0_DATA_OUT_SENT;
}

static void
ep0_status_in(void)                     /* the status is an IN on ep0: two-stage, or three after OUT data */
{
	uint32_t kind = ep0_state == EP0_STATUS_IN3 ? USB_TRBCTL_CONTROL_STATUS3 : USB_TRBCTL_CONTROL_STATUS2;

	usbn_trb(USB_DMA_EP0_TRB, usbn_dma_pa(USB_DMA_EP0_SETUP), 0,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_ISP_IMI | USB_TRB_IOC | USB_TRB_TYPE(kind));
	if (ep0_start(USB_EP_EP0IN) != 0) {
		printf("usb_net: ep0: the status stage cannot be started\n");
		ep0_restart();
		return;
	}
	ep0_state = EP0_STATUS_IN_SENT;
}

static void
ep0_status_out(void)                    /* three-stage after the data: an OUT on ep0 */
{
	usbn_trb(USB_DMA_EP0_TRB, usbn_dma_pa(USB_DMA_EP0_SETUP), 0,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_ISP_IMI | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_CONTROL_STATUS3));
	if (ep0_start(USB_EP_EP0OUT) != 0) {
		printf("usb_net: ep0: the status stage cannot be started\n");
		ep0_restart();
		return;
	}
	ep0_state = EP0_STATUS_OUT_SENT;
}

static void
ep0_stall(void)
{
	/* (ep0.c: dwc3_ep0_stall_and_restart: halt ep0, then a SETUP TRB again) */
	printf("usb_net: stall\n");
	usbn_ep_cmd(USB_EP_EP0OUT, USB_DEPCMD_SETSTALL, 0, 0, 0);
	ep0_restart();
}

static void
usbnet_data_stop(void);
static void usbnet_alt(uint32_t alt);
static void usbnet_configured(void);
static void usbnet_link(bool up);

/* One SETUP has arrived and its eight bytes are in the buffer: answer it (ep0.c: dwc3_ep0_inspect_setup). */
static void
ep0_setup(void)
{
	struct usbnet_setup *setup = (struct usbnet_setup *)usbn_dma_va(USB_DMA_EP0_SETUP);
	uint8_t type, request, answer[8];
	uint16_t value, index;
	uint32_t kind, number;

	usbn_inval((vm_offset_t)setup, sizeof(*setup));
	type = setup->bmRequestType;
	request = setup->bRequest;
	value = setup->wValue;
	index = setup->wIndex;
	ep0_wlength = setup->wLength;
	printf("usb_net: setup %02x %02x %04x %04x %04x\n", type, request, value, index, ep0_wlength);

	if (type == 0x80 && request == 6) {                     /* GET_DESCRIPTOR */
		kind = (uint32_t)value >> 8;
		number = value & 0xff;

		if (kind == USB_DT_DEVICE) {
			if (usbnet_ncm) {
				ep0_data(usbnet_device_ncm, sizeof(usbnet_device_ncm));
			} else {
				ep0_data(usbnet_device, sizeof(usbnet_device));
			}
			return;
		}
		if (kind == USB_DT_CONFIG) {
			const uint8_t *config = usbnet_ncm ? usbnet_config_ncm : usbnet_config;
			uint32_t size = usbnet_ncm ? sizeof(usbnet_config_ncm) : sizeof(usbnet_config);

			/* (the one field the table cannot carry: its own length) */
			memcpy((void *)usbn_dma_va(USB_DMA_EP0_DATA), config, size);
			((uint8_t *)usbn_dma_va(USB_DMA_EP0_DATA))[2] = (uint8_t)size;
			((uint8_t *)usbn_dma_va(USB_DMA_EP0_DATA))[3] = (uint8_t)(size >> 8);
			ep0_data(NULL, size);
			return;
		}
		if (kind == USB_DT_STRING && number <= 4) {
			ep0_data(NULL, usbnet_string(number, (uint8_t *)usbn_dma_va(USB_DMA_EP0_DATA)));
			return;
		}
		/* (the device qualifier and the other-speed configuration are for a two-speed device) */
		printf("usb_net: descriptor %u index %u: not for a high-speed-only device\n", kind, number);
		ep0_stall();
		return;
	}

	if (type == 0x00 && request == 5) {                     /* SET_ADDRESS */
		if (value > 127 || ep0_configured) {
			ep0_stall();
			return;
		}
		/* (at setup time, before the status stage: ep0.c:649 dwc3_ep0_set_address) */
		usbn_wr(USB_DCFG, (usbn_rd(USB_DCFG) & ~USB_DCFG_DEVADDR) | ((uint32_t)value << 3));
		printf("usb_net: address %u\n", value);
		ep0_status_only();
		return;
	}

	if (type == 0x00 && request == 9) {                     /* SET_CONFIGURATION */
		if (value == 1) {
			usbnet_data_stop();     /* (a second SET_CONFIGURATION: nothing may still run) */
			ep0_alt = 0;
			usbnet_configured();
			ep0_configured = true;
			printf("usb_net: configured: the endpoints are on\n");
		} else if (value == 0) {
			usbnet_data_stop();
			ep0_configured = false;
			printf("usb_net: unconfigured\n");
		} else {
			printf("usb_net: configuration %u: not here\n", value);
			ep0_stall();
			return;
		}
		ep0_status_only();
		return;
	}

	if (type == 0x80 && request == 8) {                     /* GET_CONFIGURATION */
		answer[0] = ep0_configured ? 1 : 0;
		ep0_data(answer, 1);
		return;
	}

	if ((type == 0x80 || type == 0x81 || type == 0x82) && request == 0) {   /* GET_STATUS */
		answer[0] = 0;
		answer[1] = 0;
		ep0_data(answer, 2);
		return;
	}

	if (type == 0x01 && request == 11) {                    /* SET_INTERFACE */
		if (index == 0 && value == 0) {
			/* (the control interface has one setting) */
		} else if (index == 1 && value <= 1) {
			usbnet_alt(value);
		} else {
			printf("usb_net: interface %u setting %u: not here\n", index, value);
			ep0_stall();
			return;
		}
		ep0_status_only();
		return;
	}

	if (type == 0x81 && request == 10) {                    /* GET_INTERFACE */
		answer[0] = index == 1 ? (uint8_t)ep0_alt : 0;
		ep0_data(answer, 1);
		return;
	}

	if (usbnet_ncm && type == 0xa1) {                       /* NCM's class requests in (NCM 1.0, 6.2) */
		uint8_t *data = (uint8_t *)usbn_dma_va(USB_DMA_EP0_DATA);

		switch (request) {
		case 0x80:                      /* GET_NTB_PARAMETERS, 28 bytes */
			bzero(data, 28);
			st16(data + 0, 28);             /* wLength */
			st16(data + 2, 1);              /* bmNtbFormatsSupported: NTB-16 only */
			st32(data + 4, NCM_NTB_MAX);    /* dwNtbInMaxSize: ours to the host */
			st16(data + 8, 4);              /* wNdpInDivisor */
			st16(data + 10, 0);             /* wNdpInPayloadRemainder */
			st16(data + 12, 4);             /* wNdpInAlignment */
			st32(data + 16, NCM_NTB_MAX);   /* dwNtbOutMaxSize: the host's to us */
			st16(data + 20, 4);             /* wNdpOutDivisor */
			st16(data + 22, 0);             /* wNdpOutPayloadRemainder */
			st16(data + 24, 4);             /* wNdpOutAlignment */
			st16(data + 26, 0);             /* wNtbOutMaxDatagrams: no limit */
			ep0_data(NULL, 28);
			return;
		case 0x83:                      /* GET_NTB_FORMAT: 0, NTB-16 */
		case 0x89:                      /* GET_CRC_MODE: 0, none */
			st16(data, 0);
			ep0_data(NULL, 2);
			return;
		case 0x85:                      /* GET_NTB_INPUT_SIZE */
			st32(data, NCM_NTB_MAX);
			ep0_data(NULL, 4);
			return;
		case 0x87:                      /* GET_MAX_DATAGRAM_SIZE */
			st16(data, ETHER_MAX_LEN - ETHER_CRC_LEN);
			ep0_data(NULL, 2);
			return;
		default:
			break;
		}
	}

	if (usbnet_ncm && type == 0x21 && request != 0x43) {    /* NCM's class requests out */
		switch (request) {
		case 0x84:                      /* SET_NTB_FORMAT: only NTB-16 is offered */
			if (value != 0) {
				break;
			}
			/* FALLTHROUGH */
		case 0x8a:                      /* SET_CRC_MODE (frames with a CRC are taken either way) */
		case 0x44:                      /* SET_ETHERNET_MULTICAST_FILTERS (none: all come in) */
			if (ep0_wlength == 0) {
				ep0_status_only();
				return;
			}
			break;
		case 0x86:                      /* SET_NTB_INPUT_SIZE: 4 or 8 bytes */
		case 0x88:                      /* SET_MAX_DATAGRAM_SIZE: 2 bytes */
			if (ep0_wlength != 0 && ep0_wlength <= 64) {
				printf("usb_net: NCM request %02x with %u bytes (taken, not used)\n", request, ep0_wlength);
				ep0_data_out();
				return;
			}
			if (ep0_wlength == 0) {
				ep0_status_only();
				return;
			}
			break;
		default:
			break;
		}
	}

	if (type == 0x21 && request == 0x43) {                  /* CDC SET_ETHERNET_PACKET_FILTER */
		printf("usb_net: packet filter %04x\n", value);
		ep0_status_only();
		return;
	}

	if (type == 0x02 && (request == 1 || request == 3)) {   /* CLEAR_FEATURE / SET_FEATURE */
		uint32_t epnum = index & 0x0f;

		/* (ENDPOINT_HALT; anything else is accepted and does nothing) */
		if (value == 0 && epnum != 0) {
			unsigned int phys = 2 * epnum + ((index & 0x80) != 0 ? 1 : 0);

			usbn_ep_cmd(phys, request == 1 ? USB_DEPCMD_CLEARSTALL : USB_DEPCMD_SETSTALL, 0, 0, 0);
		}
		ep0_status_only();
		return;
	}

	/* (a data stage out of the host is not expected: ECM only sends on the bulk endpoints) */
	printf("usb_net: request %02x %02x %04x %04x %04x: not here\n", type, request, value, index, ep0_wlength);
	ep0_stall();
}

/* XferComplete on ep0: the previous stage is done (ep0.c: dwc3_ep0_xfer_complete). */
static void
ep0_complete(unsigned int phys)
{
	if (ep0_active == (int)phys) {
		ep0_active = -1;
	}
	if (phys == USB_EP_EP0OUT) {
		if (ep0_state == EP0_STATUS_OUT_SENT) {
			ep0_restart();
			return;
		}
		if (ep0_state == EP0_DATA_OUT_SENT) {
			ep0_state = EP0_STATUS_IN3;     /* (the status of a control-OUT is an IN) */
			return;
		}
		if (ep0_state != EP0_ARMED) {
			/* (the host gave up on the request and sent a new SETUP: start over) */
			printf("usb_net: ep0: a SETUP while state %u was pending\n", ep0_state);
		}
		ep0_setup();
		return;
	}
	if (ep0_state == EP0_DATA_SENT) {
		ep0_state = EP0_STATUS_OUT;     /* (the status of a control-IN is an OUT) */
		return;
	}
	if (ep0_state == EP0_STATUS_IN_SENT) {
		ep0_restart();
		return;
	}
	printf("usb_net: ep0: a completion in state %u\n", ep0_state);
}

/* XferNotReady on ep0: the stage the controller is waiting for can be started now. */
static void
ep0_notready(unsigned int phys, uint32_t status)
{
	if ((status & 3) == 1) {
		return;                 /* DATA: already started with the request (ep0_data) */
	}
	if (phys == USB_EP_EP0IN && (status & 3) == 2 && (ep0_state == EP0_STATUS_IN || ep0_state == EP0_STATUS_IN3)) {
		ep0_status_in();
		return;
	}
	if (phys == USB_EP_EP0OUT && (status & 3) == 2 && ep0_state == EP0_STATUS_OUT) {
		ep0_status_out();
		return;
	}
	printf("usb_net: ep0: not ready %u in state %u\n", status, ep0_state);
}

/* ---- the data endpoints ---------------------------------------------------------------------- */

#define USB_EP_NUMBER_IN        2       /* EP2 IN: ours, phys 5 (EP2 OUT is 4, USB_EP_OUT) */
#define USB_MAXPACKET_NOTIFY    16

static LCK_GRP_DECLARE(usbnet_lck_grp, "vayu-usbnet");
static LCK_MTX_DECLARE(usbnet_lock, &usbnet_lck_grp);

/*
 * The OUT ring: 31 data TRBs and a link TRB at 31 pointing back at 0 with HWO held forever
 * (gadget.c:818) -- Linux's scheme, so the controller circles on its own and a frame is put back
 * where it was taken from. out_next is the oldest TRB the controller has not finished with, which
 * is also the next one to refill and the one to start at again after the ring ran dry.
 */
static uint32_t out_next;
static uint32_t out_rsc;
static bool out_running;            /* a transfer runs on the OUT ring */

/*
 * The IN ring: the same shape as the OUT ring, one TRB per frame (two for a frame whose length is a
 * multiple of 512: a zero-length packet must follow it, chained). in_head is the oldest TRB still the
 * controller's; a TRB is ours again once its HWO is clear (in_reclaim). One transfer, started with the
 * first frame and kept going with UPDATETRANSFER (gadget.c:1415).
 */
static uint32_t in_head;                /* the oldest TRB not yet given back */
static uint32_t in_tail;                /* where the next frame goes */
static uint32_t in_used;                /* data TRBs the controller has */
static uint32_t in_rsc;
static bool in_running;

/* The notification endpoint: NETWORK_CONNECTION first, then CONNECTION_SPEED_CHANGE (f_ecm.c). */
static uint32_t notify_phase;           /* 0 idle, 1 the first, 2 the second (a transfer runs) */
static uint32_t notify_rsc;

static bool link_up;
static bool data_up;

static bool usbnet_attached;
static const uint8_t usbnet_address[ETHER_ADDR_LEN] = { 0x02, 0xa0, 0xb0, 0xc0, 0xd0, 0x02 };

static struct {
	uint64_t rx_frames, rx_bytes, tx_frames, tx_bytes, drops;
} usbnet_count;

static void
out_refill(uint32_t slot)
{
	usbn_trb(USB_DMA_OUT_TRB + USB_TRB_STRIDE * slot,
	    usbn_dma_pa(USB_DMA_OUT_DATA + USB_FRAME_MAX * slot), USB_FRAME_MAX,
	    USB_TRB_HWO | USB_TRB_IOC | USB_TRB_CSP | USB_TRB_TYPE(USB_TRBCTL_NORMAL));
}

static void
usbnet_input(const uint8_t *frame, size_t length);

/*
 * One NTB-16 from the host: every frame its tables list (NCM 1.0, 3.3). Whatever does not add up --
 * a signature, an index past the block -- ends the block there; the frames before it have gone up.
 */
static void
ncm_input(const uint8_t *block, uint32_t length)
{
	uint32_t ndp, guard;

	if (length < 12 || ld32(block) != NCM_NTH16_SIG || ld16(block + 4) != 12) {
		usbnet_count.drops++;
		return;
	}
	if (ld16(block + 8) != 0 && ld16(block + 8) < length) {
		length = ld16(block + 8);       /* wBlockLength */
	}
	ndp = ld16(block + 10);
	for (guard = 0; ndp != 0 && guard < 8; guard++) {
		uint32_t signature, end, entry;

		if ((ndp & 3) != 0 || ndp + 16 > length) {
			break;
		}
		signature = ld32(block + ndp);
		if (signature != NCM_NDP16_SIG && signature != NCM_NDP16_SIG_CRC) {
			break;
		}
		end = ndp + ld16(block + ndp + 4);
		if (end > length) {
			end = length;
		}
		for (entry = ndp + 8; entry + 4 <= end; entry += 4) {
			uint32_t index = ld16(block + entry), size = ld16(block + entry + 2);

			if (index == 0 || size == 0) {
				break;
			}
			if (signature == NCM_NDP16_SIG_CRC) {
				size = size > 4 ? size - 4 : 0;
			}
			if (index + size > length || size == 0) {
				usbnet_count.drops++;
				break;
			}
			usbnet_input(block + index, size);
		}
		ndp = ld16(block + ndp + 6);    /* wNextNdpIndex */
	}
}

/* The frames the controller has finished with, oldest first: hand them up and refill their TRBs. */
static void
out_drain(void)
{
	uint32_t refilled = 0;

	if (!out_running) {
		return;                         /* (an event queued before the ring was stopped) */
	}
	for (uint32_t n = 0; n < USB_TRB_DATA; n++) {
		struct usbn_trb *trb =
		    (struct usbn_trb *)usbn_dma_va(USB_DMA_OUT_TRB + USB_TRB_STRIDE * out_next);
		uint32_t size, length;

		usbn_inval((vm_offset_t)trb, sizeof(*trb));
		if ((trb->ctrl & USB_TRB_HWO) != 0) {
			break;                          /* (and everything after it: in order) */
		}
		__asm__ volatile ("dmb oshld" ::: "memory");    /* (the size the controller wrote with HWO) */
		size = trb->size & 0xffffff;
		length = size > USB_FRAME_MAX ? 0 : USB_FRAME_MAX - size;
		if (length != 0) {
			usbn_inval(usbn_dma_va(USB_DMA_OUT_DATA + USB_FRAME_MAX * out_next), length);
			if (usbnet_ncm) {
				ncm_input((const uint8_t *)usbn_dma_va(USB_DMA_OUT_DATA + USB_FRAME_MAX * out_next), length);
			} else {
				usbnet_input((const uint8_t *)usbn_dma_va(USB_DMA_OUT_DATA + USB_FRAME_MAX * out_next),
				    length);
			}
		}
		out_refill(out_next);
		refilled++;
		if (++out_next == USB_TRB_DATA) {
			out_next = 0;
		}
	}
	/* (a controller that found no HWO TRB waits for this before it looks again, gadget.c:1415) */
	if (refilled != 0 && out_running) {
		usbn_ep_cmd(USB_EP_OUT, USB_DEPCMD_UPDATETRANSFER | USB_DEPCMD_PARAM(out_rsc), 0, 0, 0);
	}
}

static void
usbnet_out_kick(void)
{
	uint64_t trb = usbn_dma_pa(USB_DMA_OUT_TRB + USB_TRB_STRIDE * out_next);

	out_rsc = 0;
	out_running = usbn_ep_cmd_rsc(USB_EP_OUT, USB_DEPCMD_STARTTRANSFER, (uint32_t)(trb >> 32), (uint32_t)trb,
	    0, &out_rsc) == 0;
	if (!out_running) {
		printf("usb_net: ep2 OUT: the ring cannot be started\n");
	}
}

static void
usbnet_out_fill(void)
{
	for (uint32_t slot = 0; slot < USB_TRB_DATA; slot++) {
		out_refill(slot);
	}
	/* the link TRB, whose HWO is never cleared (gadget.c:818) */
	usbn_trb(USB_DMA_OUT_TRB + USB_TRB_STRIDE * USB_TRB_DATA, usbn_dma_pa(USB_DMA_OUT_TRB), 0,
	    USB_TRB_HWO | USB_TRB_TYPE(USB_TRBCTL_LINK));
	out_next = 0;
	usbnet_out_kick();
}

/* Start at the oldest armed TRB: the controller walks on from there by itself (gadget.c:1415). */
static void
usbnet_in_kick(void)
{
	uint64_t trb = usbn_dma_pa(USB_DMA_IN_TRB + USB_TRB_STRIDE * in_head);

	in_rsc = 0;
	if (usbn_ep_cmd_rsc(USB_EP_IN, USB_DEPCMD_STARTTRANSFER, (uint32_t)(trb >> 32), (uint32_t)trb, 0,
	    &in_rsc) != 0) {
		printf("usb_net: ep2 IN: the transfer cannot be started\n");
		in_running = false;
		return;
	}
	in_running = true;
}

static void
notify_send(uint32_t phase)
{
	uint8_t *data = (uint8_t *)usbn_dma_va(USB_DMA_NOTIFY_DATA);
	uint64_t trb = usbn_dma_pa(USB_DMA_NOTIFY_TRB);
	uint32_t length;

	if (notify_phase != 0) {
		return;                         /* (one at a time: the first one is still the controller's) */
	}
	if (phase == (usbnet_ncm ? 2u : 1u)) {
		/* NETWORK_CONNECTION (CDC 1.2 6.3.1): connected; NCM's second, as f_ncm.c sends it */
		data[0] = 0xa1;
		data[1] = 0x00;
		data[2] = 0x01;                 /* wValue: 1 = connected */
		data[3] = 0x00;
		data[4] = 0x00;
		data[5] = 0x00;
		data[6] = 0x00;                 /* wLength */
		data[7] = 0x00;
		length = 8;
	} else {
		/* CONNECTION_SPEED_CHANGE (6.3.2): 480 Mbit/s each way */
		data[0] = 0xa1;
		data[1] = 0x2a;
		data[2] = 0x00;
		data[3] = 0x00;
		data[4] = 0x08;                 /* wLength: two 32-bit rates */
		data[5] = 0x00;
		data[6] = 0x00;                 /* 480000000, little endian */
		data[7] = 0x38;
		data[8] = 0x9c;
		data[9] = 0x1c;
		data[10] = 0x00;
		data[11] = 0x38;
		data[12] = 0x9c;
		data[13] = 0x1c;
		length = 16;
	}
	usbn_clean((vm_offset_t)data, length);
	usbn_trb(USB_DMA_NOTIFY_TRB, usbn_dma_pa(USB_DMA_NOTIFY_DATA), length,
	    USB_TRB_HWO | USB_TRB_LST | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_NORMAL));
	notify_phase = phase;
	if (usbn_ep_cmd_rsc(USB_EP_NOTIFY, USB_DEPCMD_STARTTRANSFER, (uint32_t)(trb >> 32), (uint32_t)trb, 0,
	    &notify_rsc) != 0) {
		printf("usb_net: ep1 IN: the notification cannot be sent\n");
		notify_phase = 0;
	}
}

/* SET_CONFIGURATION(1): the three endpoints of the ECM function (gadget.c:670, f_ecm.c). */
static void
usbnet_configured(void)
{
	uint32_t bits = (1u << USB_EP_NOTIFY) | (1u << USB_EP_OUT) | (1u << USB_EP_IN);

	/*
	 * INIT every time (gadget.c:701 on each enable): the data toggles start again as the host's do.
	 * The endpoints are off while that happens; their transfer resources were assigned once, at the
	 * start (usbn.c dwc3_endpoints()). FIFO 1 for the interrupt endpoint, 2 for bulk IN.
	 */
	usbn_wr(USB_DALEPENA, usbn_rd(USB_DALEPENA) & ~bits);
	if (usbn_ep_config(USB_EP_NOTIFY, USB_DEPCFG_TYPE_INTR, USB_MAXPACKET_NOTIFY, 1, 9, false, 0) != 0 ||
	    usbn_ep_config(USB_EP_OUT, USB_DEPCFG_TYPE_BULK, USB_MAXPACKET, 0, 0, true, 0) != 0 ||
	    usbn_ep_config(USB_EP_IN, USB_DEPCFG_TYPE_BULK, USB_MAXPACKET, USB_EP_NUMBER_IN, 0, true, 0) != 0) {
		printf("usb_net: the endpoints cannot be configured\n");
		return;
	}
	usbn_wr(USB_DALEPENA, usbn_rd(USB_DALEPENA) | bits);
	printf("usb_net: endpoints: ep1 IN notify %u bytes, ep2 OUT and IN %u\n", USB_MAXPACKET_NOTIFY, USB_MAXPACKET);
}

/* The IN ring empty: no data TRB armed, the link TRB back to the first. */
static void
in_fill(void)
{
	for (uint32_t slot = 0; slot < USB_TRB_DATA; slot++) {
		usbn_trb(USB_DMA_IN_TRB + USB_TRB_STRIDE * slot, 0, 0, 0);
	}
	usbn_trb(USB_DMA_IN_TRB + USB_TRB_STRIDE * USB_TRB_DATA, usbn_dma_pa(USB_DMA_IN_TRB), 0,
	    USB_TRB_HWO | USB_TRB_TYPE(USB_TRBCTL_LINK));
	in_head = in_tail = in_used = 0;
}

/* The TRBs the controller has given back, oldest first (called with usbnet_lock held). */
static void
in_reclaim(void)
{
	while (in_used != 0) {
		struct usbn_trb *trb =
		    (struct usbn_trb *)usbn_dma_va(USB_DMA_IN_TRB + USB_TRB_STRIDE * in_head);

		usbn_inval((vm_offset_t)trb, sizeof(*trb));
		if ((trb->ctrl & USB_TRB_HWO) != 0) {
			break;
		}
		in_used--;
		in_head = (in_head + 1) % USB_TRB_DATA;
	}
}

static void
usbnet_data_stop(void)
{
	bool was_up;

	/* (data_up first, under the lock: no frame is queued behind our back from here on) */
	lck_mtx_lock(&usbnet_lock);
	was_up = data_up;
	data_up = false;
	if (in_running) {
		/* (gadget.c:3240: ENDTRANSFER with FORCERM, then the core's moment, gadget.c:3247) */
		usbn_ep_cmd(USB_EP_IN, USB_DEPCMD_ENDTRANSFER | USB_DEPCMD_FORCERM | USB_DEPCMD_CMDIOC |
		    USB_DEPCMD_PARAM(in_rsc), 0, 0, 0);
		delay(100);
	}
	in_rsc = 0;
	in_fill();
	in_running = false;
	lck_mtx_unlock(&usbnet_lock);
	if (out_running) {
		usbn_ep_cmd(USB_EP_OUT, USB_DEPCMD_ENDTRANSFER | USB_DEPCMD_FORCERM | USB_DEPCMD_CMDIOC |
		    USB_DEPCMD_PARAM(out_rsc), 0, 0, 0);
		delay(100);
	}
	out_rsc = 0;
	out_running = false;
	out_next = 0;
	if (notify_phase != 0) {
		/* (a notification the host never read keeps its transfer: end it, or the next cannot start) */
		usbn_ep_cmd(USB_EP_NOTIFY, USB_DEPCMD_ENDTRANSFER | USB_DEPCMD_FORCERM | USB_DEPCMD_CMDIOC |
		    USB_DEPCMD_PARAM(notify_rsc), 0, 0, 0);
		delay(100);
	}
	notify_phase = 0;
	if (was_up) {
		usbnet_link(false);
	}
}

static void
usbnet_data_start(void)
{
	if (!usbnet_attached || !ep0_configured) {
		printf("usb_net: interface 1 alternate 1: no data path\n");
		return;
	}
	/* the bulk endpoints anew (data toggles from DATA0, as after f_ecm's set_alt) */
	usbnet_configured();
	usbnet_out_fill();
	lck_mtx_lock(&usbnet_lock);
	in_fill();
	in_running = false;
	in_rsc = 0;
	data_up = true;
	lck_mtx_unlock(&usbnet_lock);
	usbnet_link(true);
	notify_send(1);
}

static void
usbnet_alt(uint32_t alt)
{
	/* (f_ecm's set_alt: whatever ran stops first, then alternate 1 starts afresh) */
	usbnet_data_stop();
	ep0_alt = alt;
	if (alt == 1) {
		usbnet_data_start();
	}
}

/*
 * One frame into the IN ring (called with usbnet_lock held): the zero-length TRB first, then the
 * frame's with HWO last, so the controller never meets half of a chain; then STARTTRANSFER if no
 * transfer runs yet, else the no-response UPDATETRANSFER (gadget.c:1415).
 */
static bool
usbnet_tx(mbuf_t packet, uint32_t length)
{
	uint32_t slot = in_tail, next = (in_tail + 1) % USB_TRB_DATA;
	uint32_t offset = usbnet_ncm ? NCM_TX_HDR : 0;
	bool zlp;
	uint8_t *buffer;
	static uint16_t sequence;

	if (!data_up) {
		return false;                   /* (stopped between the caller's look and the lock) */
	}
	in_reclaim();
	if (in_used + (zlp ? 2 : 1) > USB_TRB_DATA - 1) {
		return false;                   /* (the ring is full: the frame is dropped) */
	}
	if (offset + length + 1 > USB_FRAME_MAX) {
		return false;
	}
	buffer = (uint8_t *)usbn_dma_va(USB_DMA_IN_DATA + USB_FRAME_MAX * slot);
	if (mbuf_copydata(packet, 0, length, buffer + offset) != 0) {
		return false;
	}
	if (usbnet_ncm) {
		uint32_t frame = length;

		length += NCM_TX_HDR;
		/* (a block of a multiple of 512 bytes would want a zero-length packet after it; a pad
		 * byte inside the block, which wBlockLength counts, ends it as well: sweet_usb) */
		if ((length % USB_MAXPACKET) == 0) {
			buffer[length++] = 0;
		}
		st32(buffer + 0, NCM_NTH16_SIG);
		st16(buffer + 4, 12);                   /* wHeaderLength */
		st16(buffer + 6, sequence++);           /* wSequence */
		st16(buffer + 8, length);               /* wBlockLength */
		st16(buffer + 10, 12);                  /* wNdpIndex */
		st32(buffer + 12, NCM_NDP16_SIG);
		st16(buffer + 16, 16);                  /* wLength */
		st16(buffer + 18, 0);                   /* wNextNdpIndex */
		st16(buffer + 20, NCM_TX_HDR);          /* the frame: index, length */
		st16(buffer + 22, frame);
		st16(buffer + 24, 0);                   /* the table's end */
		st16(buffer + 26, 0);
	}
	zlp = (length % USB_MAXPACKET) == 0;
	if (zlp) {
		usbn_trb(USB_DMA_IN_TRB + USB_TRB_STRIDE * next,
		    usbn_dma_pa(USB_DMA_IN_DATA + USB_FRAME_MAX * next), 0,
		    USB_TRB_HWO | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_NORMAL));
		usbn_trb(USB_DMA_IN_TRB + USB_TRB_STRIDE * slot,
		    usbn_dma_pa(USB_DMA_IN_DATA + USB_FRAME_MAX * slot), length,
		    USB_TRB_HWO | USB_TRB_CHN | USB_TRB_TYPE(USB_TRBCTL_NORMAL));
		in_tail = (next + 1) % USB_TRB_DATA;
		in_used += 2;
	} else {
		usbn_trb(USB_DMA_IN_TRB + USB_TRB_STRIDE * slot,
		    usbn_dma_pa(USB_DMA_IN_DATA + USB_FRAME_MAX * slot), length,
		    USB_TRB_HWO | USB_TRB_IOC | USB_TRB_TYPE(USB_TRBCTL_NORMAL));
		in_tail = next;
		in_used += 1;
	}
	if (!in_running) {
		usbnet_in_kick();
	} else {
		usbn_ep_cmd(USB_EP_IN, USB_DEPCMD_UPDATETRANSFER | USB_DEPCMD_PARAM(in_rsc), 0, 0, 0);
	}
	return true;
}


/* ---- the interface: hakios.net.c's, with this as its link -------------------------------------- */

static bool
usbnet_transmit(mbuf_t packet, size_t length)
{
	bool queued = false;

	if (link_up && data_up && length != 0 && length <= ETHER_MAX_LEN - ETHER_CRC_LEN) {
		lck_mtx_lock(&usbnet_lock);
		queued = usbnet_tx(packet, (uint32_t)length);
		lck_mtx_unlock(&usbnet_lock);
	}
	if (queued) {
		usbnet_count.tx_frames++;
		usbnet_count.tx_bytes += length;
	} else {
		usbnet_count.drops++;
	}
	return queued;
}

/* A frame has arrived from the host. */
static void
usbnet_input(const uint8_t *frame, size_t length)
{
	usbnet_count.rx_frames++;
	usbnet_count.rx_bytes += length;
	if (!usbnet_attached || length < ETHER_HDR_LEN || length > ETHER_MAX_LEN - ETHER_CRC_LEN) {
		usbnet_count.drops++;
		return;
	}
	hakios_net_input_unit(USBNET_UNIT, frame, length);
}

static void
usbnet_link(bool up)
{
	if (!usbnet_attached || link_up == up) {
		return;
	}
	link_up = up;
	printf("usb_net: en%u: the link is %s\n", USBNET_UNIT, up ? "up" : "down");
	hakios_net_link_state_unit(USBNET_UNIT, up);
}

static struct hakios_net_link usbnet_link_desc = {
	.name = "usb_net",
	.unit = USBNET_UNIT,
	.address = { 0x02, 0xa0, 0xb0, 0xc0, 0xd0, 0x02 },
	.up = false,
	.transmit = usbnet_transmit,
};

/* ---- the address: 192.168.7.2/24, once --------------------------------------------------------- */

/*
 * struct in_aliasreq, laid out here rather than including netinet/in_var.h from bsd/dev: three
 * sockaddr_in of 16 bytes after the name, as net/if.h's ifaliasreq (what SIOCAIFADDR carries).
 */
struct in_aliasreq {
	char ifra_name[IFNAMSIZ];
	struct sockaddr_in ifra_addr;
	struct sockaddr_in ifra_broadaddr;
	struct sockaddr_in ifra_mask;
};

static void
usbnet_ipv4(void)
{
	struct in_aliasreq ifra;
	struct ifreq ifr;
	socket_t so = NULL;
	uint32_t ip = usb_ip, mask = 0xffffff00;
	char name[8];
	errno_t error;

	snprintf(name, sizeof(name), "en%u", USBNET_UNIT);
	error = sock_socket(PF_INET, SOCK_DGRAM, 0, NULL, NULL, &so);
	if (error != 0) {
		printf("usb_net: no socket to give %s its address: %d\n", name, error);
		return;
	}
	bzero(&ifr, sizeof(ifr));
	strlcpy(ifr.ifr_name, name, sizeof(ifr.ifr_name));
	error = sock_ioctl(so, SIOCPROTOATTACH, &ifr);
	printf("usb_net: SIOCPROTOATTACH %s: %d%s\n", name, error, error == EEXIST ? " (already attached)" : "");
	bzero(&ifra, sizeof(ifra));
	strlcpy(ifra.ifra_name, name, sizeof(ifra.ifra_name));
	ifra.ifra_addr.sin_len = ifra.ifra_mask.sin_len = ifra.ifra_broadaddr.sin_len = sizeof(struct sockaddr_in);
	ifra.ifra_addr.sin_family = ifra.ifra_mask.sin_family = ifra.ifra_broadaddr.sin_family = AF_INET;
	ifra.ifra_addr.sin_addr.s_addr = htonl(ip);
	ifra.ifra_mask.sin_addr.s_addr = htonl(mask);
	ifra.ifra_broadaddr.sin_addr.s_addr = htonl((ip & mask) | ~mask);
	error = sock_ioctl(so, SIOCAIFADDR, &ifra);
	printf("usb_net: SIOCAIFADDR %s %u.%u.%u.%u/24: %d\n", name, (ip >> 24) & 0xff, (ip >> 16) & 0xff, (ip >> 8) & 0xff,
	    ip & 0xff, error);
	bzero(&ifr, sizeof(ifr));
	strlcpy(ifr.ifr_name, name, sizeof(ifr.ifr_name));
	error = sock_ioctl(so, SIOCGIFFLAGS, &ifr);
	if (error == 0) {
		ifr.ifr_flags |= IFF_UP;
		error = sock_ioctl(so, SIOCSIFFLAGS, &ifr);
	}
	printf("usb_net: %s up: %d\n", name, error);
	sock_close(so);
}

/* ---- the log port ------------------------------------------------------------------------------ */

static socket_t
port_listen(uint16_t port)
{
	struct sockaddr_in me = { .sin_len = sizeof(me), .sin_family = AF_INET, .sin_port = htons(port) };
	socket_t ls = NULL;
	int on = 1;

	for (;;) {
		if (sock_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, NULL, &ls) == 0) {
			(void)sock_setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
			if (sock_bind(ls, (struct sockaddr *)&me) == 0 && sock_listen(ls, 2) == 0) {
				return ls;
			}
			sock_close(ls);
			ls = NULL;
		}
		IOSleep(1000);
	}
}

static bool
port_send(socket_t cs, const void *data, size_t length)
{
	while (length > 0) {
		struct iovec iov = { .iov_base = (void *)(uintptr_t)data, .iov_len = length };
		struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
		size_t sent = 0;

		if (sock_send(cs, &msg, 0, &sent) != 0 || sent == 0) {
			return false;
		}
		data = (const char *)data + sent;
		length -= sent;
	}
	return true;
}

static void
log_port_thread(void *arg __unused, wait_result_t wr __unused)
{
	static char chunk[16384];
	socket_t ls = port_listen(5140);

	for (;;) {
		socket_t cs = NULL;
		uint64_t pos = 0;
		struct timeval patience = { .tv_sec = 20, .tv_usec = 0 };

		if (sock_accept(ls, NULL, 0, 0, NULL, NULL, &cs) != 0 || cs == NULL) {
			IOSleep(500);
			continue;
		}
		/* (a reader that went away without a word -- a cable pulled -- frees this thread in 20 s) */
		(void)sock_setsockopt(cs, SOL_SOCKET, SO_SNDTIMEO, &patience, sizeof(patience));
		for (;;) {
			uint32_t n = loader_log_copy(&pos, chunk, sizeof(chunk));

			if (n == 0) {
				IOSleep(100);
				/* (an empty send finds out whether the reader is still there) */
				if (!port_send(cs, "", 0) || !link_up) {
					break;
				}
				continue;
			}
			if (!port_send(cs, chunk, n)) {
				break;
			}
		}
		sock_close(cs);
	}
}

/* ---- what the controller side calls ------------------------------------------------------------ */

void
usbn_ecm_connect(void)
{
	/* (gadget.c:3392 dwc3_gadget_conndone_interrupt: the speed is known, ep0 is 64 by now) */
	if (usbn_ep_config(USB_EP_EP0OUT, USB_DEPCFG_TYPE_CONTROL, 64, 0, 0, false, 2) != 0 ||
	    usbn_ep_config(USB_EP_EP0IN, USB_DEPCFG_TYPE_CONTROL, 64, 0, 0, false, 2) != 0) {
		printf("usb_net: ep0 cannot be told the speed\n");
	}
}

void
usbn_ecm_reset(void)
{
	usbnet_data_stop();
	ep0_configured = false;
	ep0_alt = 0;
	/* (a halt the host set does not outlive the reset: gadget.c dwc3_clear_stall_all_ep) */
	usbn_ep_cmd(USB_EP_NOTIFY, USB_DEPCMD_CLEARSTALL, 0, 0, 0);
	usbn_ep_cmd(USB_EP_OUT, USB_DEPCMD_CLEARSTALL, 0, 0, 0);
	usbn_ep_cmd(USB_EP_IN, USB_DEPCMD_CLEARSTALL, 0, 0, 0);
	/* (an armed SETUP TRB lives through a bus reset, which is what ep0.c assumes; a request that was
	 * under way is ended first, gadget.c:3360 dwc3_ep0_end_control_data) */
	if (ep0_state != EP0_ARMED) {
		ep0_end();
		usbn_ep_cmd(USB_EP_EP0OUT, USB_DEPCMD_SETSTALL, 0, 0, 0);
		ep0_restart();
	}
}

void
usbn_ecm_disconnect(void)
{
	usbnet_data_stop();
	ep0_configured = false;
	ep0_alt = 0;
	if (ep0_state != EP0_ARMED) {
		ep0_end();
		ep0_restart();
	}
}

void
usbn_ecm_ep_event(unsigned int phys, uint32_t event, uint32_t status, uint32_t param)
{
	(void)param;

	switch (phys) {
	case USB_EP_EP0OUT:
	case USB_EP_EP0IN:
		if (event == USB_DEPEVT_XFERCOMPLETE || event == USB_DEPEVT_XFERINPROGRESS) {
			ep0_complete(phys);
		} else if (event == USB_DEPEVT_XFERNOTREADY) {
			ep0_notready(phys, status);
		}
		break;
	case USB_EP_NOTIFY:
		if (event == USB_DEPEVT_XFERCOMPLETE) {
			uint32_t phase = notify_phase;

			notify_phase = 0;
			if (phase == 1) {
				notify_send(2);
			}
		}
		break;
	case USB_EP_OUT:
		out_drain();
		if (event == USB_DEPEVT_XFERNOTREADY && !out_running && data_up) {
			usbnet_out_kick();      /* (no transfer runs: start one at the oldest TRB, gadget.c:3098) */
		}
		break;
	case USB_EP_IN:
		lck_mtx_lock(&usbnet_lock);
		in_reclaim();
		if (event == USB_DEPEVT_XFERCOMPLETE) {
			/* (only after an ENDTRANSFER, as no TRB carries LST: the transfer is over) */
			in_running = false;
			in_rsc = 0;
		}
		if (!in_running && in_used != 0 && data_up) {
			usbnet_in_kick();
		}
		lck_mtx_unlock(&usbnet_lock);
		break;
	default:
		printf("usb_net: endpoint %u: event %u\n", phys, event);
		break;
	}
}

void
usbn_ecm_counters(void)
{
	if (!ep0_configured) {
		return;
	}
	printf("usb_net: %llu frames in (%llu bytes), %llu out (%llu bytes), %llu dropped; %llu events\n",
	    usbnet_count.rx_frames, usbnet_count.rx_bytes, usbnet_count.tx_frames, usbnet_count.tx_bytes,
	    usbnet_count.drops, usbn_events());
}

/* ---- the start --------------------------------------------------------------------------------- */

static void
usbnet_thread(__unused void *parameter, __unused wait_result_t result)
{
	thread_t thread;

	dwc3 = ml_io_map(DWC3_PHYS, DWC3_SIZE);
	if (dwc3 == 0) {
		printf("usb_net: dwc3 cannot be mapped\n");
		thread_terminate(current_thread());
	}
	usbnet_ncm = true;
	if (!hakios_net_attach(&usbnet_link_desc)) {
		printf("usb_net: en%u cannot be attached\n", USBNET_UNIT);
		thread_terminate(current_thread());
	}
	usbnet_attached = true;
	usbnet_ipv4();
	if (kernel_thread_start(log_port_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	/* (the loader's UEFI cannot run while the kernel is on its core: nothing to wait for or ask) */
	usb_taken = true;
	if (!dma_area()) {
		thread_terminate(current_thread());
	}
	if (!dwc3_take()) {
		printf("usb_net: the controller is not ours; nothing more is done\n");
		thread_terminate(current_thread());
	}
	event_loop();
}

void
hakios_usb_net_init(void)
{
	uint32_t enable = 0;
	thread_t thread;

	if (!PE_parse_boot_argn("usb_net", &enable, sizeof(enable)) || enable == 0) {
		return;
	}
	PE_parse_boot_argn("usb_net_ip", &usb_ip, sizeof(usb_ip));
	if (kernel_thread_start(usbnet_thread, NULL, &thread) != KERN_SUCCESS) {
		printf("usb_net: the thread cannot be started\n");
		return;
	}
	thread_deallocate(thread);
}
