/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MHI host for the radio on PCIe0 (QCA6490 on the OnePlus 10 Pro), the bodies of the C5 API of
 * wifi/lx/include/linux/mhi.h, and the IPCR client that turns channels 20/21 into a QRTR link (C4/C6).
 *
 * The register and ring formats, and the order of power-up, are those of Linux's drivers/bus/mhi/host
 * (init.c, pm.c, boot.c, main.c; GPL-2.0, Copyright (c) 2018-2020 The Linux Foundation, and common.h,
 * Copyright (c) 2022 Linaro Ltd.): values and sequence only, written again for this layer, which has
 * no workqueues, no tasklets, no interrupts and no host locks of Linux's kind. What differs, and why:
 *
 *   Interrupts   None arrive. The handlers Linux would run in an interrupt are registered with
 *                request_irq() exactly as Linux does and run on the one polling thread of lx_pci.c,
 *                which calls every registered handler each millisecond. lx_mhi_poll() does the same
 *                work for whoever has no such thread. Both use one servicing path, so they may meet.
 *   Workers      Linux's state-transition worker is a queue of transitions run by whoever services
 *                (the thread in mhi_sync_power_up(), or a handler); one runs at a time. Waits in it
 *                service the event rings themselves, so a transition may wait for a command completion.
 *   Buffers      Memory the radio reads and writes must be of the one pool (C4). Client buffers are
 *                therefore copied through bounce slots owned here (Linux's use_bounce_buf mode): an
 *                UL buffer is copied when queued, a DL buffer is filled when its completion is
 *                delivered. The pool never gives memory back (dma_free_coherent does not), so every
 *                block taken is kept by the controller and reused by size across power cycles.
 *   Device wake  Held from the first M0 until power-down, like iOSPlus10's stage 1 and unlike Linux's
 *                idle M2: the device is never allowed into M1/M2 (ath11k comments that register
 *                reads in M2 hang the system).
 *   Suspend      mhi_pm_suspend/resume_force and RDDM are not supported and say so.
 *
 * One controller at a time (one radio): the IPCR client keeps one set of state.
 */
#ifdef CONFIG_ATH11K_PCI
#include <linux/mhi.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <lx_os.h>
#include "lx.h"

#if defined(LX_HAKIOS_SMMU_H)
#include LX_HAKIOS_SMMU_H
#endif
/* C3's window (hakios.smmu.h). A copy is used when that header is not on the include path; the
 * host test compares it with the real one. */
#define MHI_POOL_BASE_FALLBACK 0xa0000000ull
#define MHI_POOL_SIZE_FALLBACK (64ull << 20)
#ifndef HAKIOS_WIFI_IOVA_BASE
#define HAKIOS_WIFI_IOVA_BASE MHI_POOL_BASE_FALLBACK
#define HAKIOS_WIFI_POOL_SIZE MHI_POOL_SIZE_FALLBACK
#endif

_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "MHI structures are little-endian; host is too");

#define MHI_LOG(fmt, ...) lx_tell("mhi: " fmt "\n", ##__VA_ARGS__)

/* ---- registers (Linux drivers/bus/mhi/common.h) ---- */
#define R_MHICFG 0x10
#define R_CHDBOFF 0x18
#define R_ERDBOFF 0x20
#define R_BHIOFF 0x28
#define R_BHIEOFF 0x2c
#define R_MHICTRL 0x38
#define R_MHISTATUS 0x48
#define R_CCABAP_LOWER 0x58
#define R_CCABAP_HIGHER 0x5c
#define R_ECABAP_LOWER 0x60
#define R_ECABAP_HIGHER 0x64
#define R_CRCBAP_LOWER 0x68
#define R_CRCBAP_HIGHER 0x6c
#define R_CRDB_LOWER 0x70
#define R_MHICTRLBASE_LOWER 0x80
#define R_MHICTRLBASE_HIGHER 0x84
#define R_MHICTRLLIMIT_LOWER 0x88
#define R_MHICTRLLIMIT_HIGHER 0x8c
#define R_MHIDATABASE_LOWER 0x98
#define R_MHIDATABASE_HIGHER 0x9c
#define R_MHIDATALIMIT_LOWER 0xa0
#define R_MHIDATALIMIT_HIGHER 0xa4
#define MHICFG_NHWER_MASK 0xff000000u
#define MHICFG_NER_MASK 0x00ff0000u
#define MHICTRL_MHISTATE_MASK 0x0000ff00u
#define MHICTRL_RESET_MASK 0x00000002u
#define MHISTATUS_MHISTATE_MASK 0x0000ff00u
#define MHISTATUS_SYSERR_MASK 0x00000004u
#define MHISTATUS_READY_MASK 0x00000001u

#define BHI_IMGADDR_LOW 0x08
#define BHI_IMGADDR_HIGH 0x0c
#define BHI_IMGSIZE 0x10
#define BHI_IMGTXDB 0x18
#define BHI_INTVEC 0x20
#define BHI_EXECENV 0x28
#define BHI_STATUS 0x2c
#define BHI_ERRCODE 0x30
#define BHI_ERRDBG1 0x34
#define BHI_ERRDBG2 0x38
#define BHI_ERRDBG3 0x3c
#define BHI_SERIALNU 0x40
#define BHI_TXDB_SEQNUM_BMSK 0x3fffffffu
#define BHI_STATUS_MASK 0xc0000000u
#define BHI_STATUS_ERROR 0x03
#define BHI_STATUS_SUCCESS 0x02

#define BHIE_TXVECADDR_LOW_OFFS 0x2c
#define BHIE_TXVECADDR_HIGH_OFFS 0x30
#define BHIE_TXVECSIZE_OFFS 0x34
#define BHIE_TXVECDB_OFFS 0x3c
#define BHIE_TXVECSTATUS_OFFS 0x44
#define BHIE_TXVECDB_SEQNUM_BMSK 0x3fffffffu
#define BHIE_TXVECSTATUS_STATUS_BMSK 0xc0000000u
#define BHIE_TXVECSTATUS_STATUS_XFER_COMPL 0x02
#define BHIE_TXVECSTATUS_STATUS_ERROR 0x03

#define MHI_DEV_WAKE_DB 127
#define CMD_EL_PER_RING 128
#define MHI_MAX_MTU 0xffffu
#define MHI_TIMEOUT_DEFAULT_MS 1000u
#define MHI_NAME_SIZE 32
#define IPCR_PACKET_MAX (32u + 8192u)	/* QRTR v1 header and the most C6 allows */
#define SLOT_ALIGN 64u

/* ---- ring elements and contexts (Linux common.h, init.c) ---- */
struct mhi_el {
	u64 ptr;
	u32 dw0, dw1;
};
/* One layout serves the channel, event and command contexts: 44 bytes, 64-bit fields on 4-byte
 * alignment as Linux's __packed __aligned(4). */
struct mhi_ctx {
	u32 w0, w1, w2;
	u32 rbase[2], rlen[2], rp[2], wp[2];
};
_Static_assert(sizeof(struct mhi_el) == 16, "ring element is 16 bytes");
_Static_assert(sizeof(struct mhi_ctx) == 44, "context is 44 bytes");

enum { PKT_NOOP_CMD = 1, PKT_TRANSFER = 2, PKT_RESET_CHAN_CMD = 0x10, PKT_STOP_CHAN_CMD = 0x11,
       PKT_START_CHAN_CMD = 0x12, PKT_STATE_CHANGE = 0x20, PKT_CMD_COMPLETION = 0x21,
       PKT_TX_EVENT = 0x22, PKT_EE_EVENT = 0x40, PKT_BW_REQ = 0x50 };
enum { CC_INVALID = 0, CC_SUCCESS = 1, CC_EOT = 2, CC_OVERFLOW = 3, CC_EOB = 4, CC_OOB = 5,
       CC_DB_MODE = 6, CC_UNDEFINED_ERR = 0x10, CC_BAD_TRE = 0x11 };
enum { CH_DISABLED = 0, CH_ENABLED = 1 };
#define ER_TYPE_VALID 1u

/* power-management state: Linux's bits, the ones used here */
#define PM_DISABLE 0x1u
#define PM_POR 0x2u
#define PM_M0 0x4u
#define PM_FW_DL_ERR 0x80u
#define PM_SYS_ERR_FAIL 0x400u
#define PM_SHUTDOWN 0x800u
#define PM_LD_ERR_FATAL 0x1000u
#define PM_ERRORS (PM_FW_DL_ERR | PM_SYS_ERR_FAIL | PM_SHUTDOWN | PM_LD_ERR_FATAL)
#define PM_REG_OK (PM_POR | PM_M0 | PM_FW_DL_ERR | PM_SYS_ERR_FAIL | PM_SHUTDOWN)
#define PM_EVENTS_OK (PM_POR | PM_M0)

/* transitions the executor runs, in the order the device produces them */
#define T_PBL 0x1u
#define T_READY 0x2u
#define T_SBL 0x4u
#define T_MISSION 0x8u

#define MAX_EV 8
#define MAX_BLK 96
#define MAX_DEV 8
#define MAX_DRV 4
#define EV_QUOTA 128

struct mhi_ring {
	struct mhi_el *el;
	dma_addr_t iova;
	u32 n, rp, wp;
	void *blk;	/* what to give back */
};
struct mhi_bufinfo {
	void *cb;	/* returned to the client in mhi_result.buf_addr */
	void *dst;	/* a DL buffer is filled here at completion */
	u32 len;
};
struct mhi_chan {
	const char *name;
	u32 num, elements, er, ee_mask, pollcfg;
	enum mhi_ch_type type;
	enum dma_data_direction dir;
	enum mhi_db_brst_mode brst;
	bool configured, db_mode;
	volatile int state;
	volatile u32 ccs;
	volatile bool cmd_done;
	volatile u32 cb_busy;	/* callbacks being run: unprepare waits for them */
	struct mhi_ring tre;
	struct mhi_bufinfo *buf;
	u8 *bb;
	dma_addr_t bb_dma;
	size_t slot;
	struct mhi_device *dev;
	void (*xfer_cb)(struct mhi_device *, struct mhi_result *);
	void *db;
};
struct mhi_event {
	u32 index, intmod, irq, elements;
	enum mhi_er_data_type type;
	enum mhi_db_brst_mode brst;
	bool db_mode, requested;
	volatile bool busy;
	struct mhi_ring ring;
	void *db;
	struct mhi_priv *p;
};
struct mhi_dev_slot {
	struct mhi_device *dev;
	const struct mhi_driver *drv;
	const struct mhi_device_id *id;
	bool bound;
};
struct mhi_blk {
	void *va;
	dma_addr_t dma;
	size_t size;
	bool used, taken;
};

struct mhi_priv {
	struct mhi_controller *c;
	int locked;
	u32 max_chan, n_ev;
	struct mhi_chan *chan;
	struct mhi_event ev[MAX_EV];
	struct mhi_ring cmd;
	struct mhi_ctx *chan_ctx, *er_ctx, *cmd_ctx;
	dma_addr_t chan_ctx_dma, er_ctx_dma, cmd_ctx_dma;
	u64 win_lo, win_hi;	/* inclusive: the part of C3's pool the caller allows */
	u8 *bhi, *bhie;
	u32 chdboff, erdboff;
	struct mhi_blk blk[MAX_BLK];

	volatile u32 pm;
	volatile enum mhi_state dev_state;
	volatile enum mhi_ee_type ee;
	volatile u32 pending;
	volatile bool tr_busy, mission_done, watch_busy;
	volatile int fail_err;
	bool prepared, registered, irq0, irqs_on, m0_requested, sbl_seen, mission_seen;
	bool rddm_told, syserr_told, bhi_fault;
	int builtin_err;
	u32 dev_wake, pending_pkts, watch_n, ready_timeout_ms;
	bool wake_set, wake_hold;
	u8 *wake_db;
	struct mhi_dev_slot dev_slot[MAX_DEV];
	u32 serial;
};

#define PRIV(c) ((struct mhi_priv *)(c)->private_data)

/* one controller, one IPCR client, one driver table */
static struct mhi_controller *active_ctrl;
static const struct mhi_driver *drivers[MAX_DRV];
static struct mhi_driver ipcr_driver;	/* the IPCR client, defined below */

/* ---- the two small locks: data (never held across a client call or a sleep), and the claim flags ---- */
static void
L(struct mhi_priv *p)
{
	lx_os_lock();
	while (p->locked) {
		lx_os_sleep(p, 0);
	}
	p->locked = 1;
	lx_os_unlock();
}
static void
U(struct mhi_priv *p)
{
	lx_os_lock();
	p->locked = 0;
	lx_os_wakeup(p);
	lx_os_unlock();
}

static inline u32 lo32(u64 v) { return (u32)v; }
static inline u32 hi32(u64 v) { return (u32)(v >> 32); }
static inline u32 ctz32(u32 v) { return (u32)__builtin_ctz(v); }

static void
fence(void)
{
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline u64
ctx_get64(const volatile u32 *w)
{
	return (u64)w[0] | ((u64)w[1] << 32);
}
static inline void
ctx_set64(volatile u32 *w, u64 v)
{
	w[0] = lo32(v);
	w[1] = hi32(v);
}

/* ---- register access through the controller's callbacks ---- */
static int
rd(struct mhi_controller *c, void *base, u32 off, u32 *v)
{
	int r = c->read_reg(c, (u8 *)base + off, v);

	return r < 0 ? r : (r ? -r : 0);
}
static int
wr(struct mhi_controller *c, void *base, u32 off, u32 v)
{
	c->write_reg(c, (u8 *)base + off, v);
	return 0;
}
static int
rd_field(struct mhi_controller *c, void *base, u32 off, u32 mask, u32 *v)
{
	u32 t;
	int r = rd(c, base, off, &t);

	if (r) {
		return r;
	}
	*v = (t & mask) >> ctz32(mask);
	return 0;
}
static int
wr_field(struct mhi_controller *c, void *base, u32 off, u32 mask, u32 v)
{
	u32 t;
	int r = rd(c, base, off, &t);

	if (r) {
		return r;
	}
	t = (t & ~mask) | ((v << ctz32(mask)) & mask);
	return wr(c, base, off, t);
}
/* 64-bit doorbell: high word first, the low word rings (Linux mhi_write_db) */
static void
write_db(struct mhi_controller *c, void *db, u64 val)
{
	wr(c, db, 4, hi32(val));
	wr(c, db, 0, lo32(val));
}

/* ---- named things for the log ---- */
static const char *
ee_name(u32 ee)
{
	static const char *const n[] = { "PBL", "SBL", "AMSS", "RDDM", "WFW", "PTHRU", "EDL", "FP" };

	if (ee < sizeof(n) / sizeof(n[0])) {
		return n[ee];
	}
	return ee == MHI_EE_DISABLE_TRANSITION ? "DISABLE" : "invalid";
}
static const char *
state_name(u32 s)
{
	static const char *const n[] = { "RESET", "READY", "M0", "M1", "M2", "M3", "M3_FAST", "BHI" };

	if (s < sizeof(n) / sizeof(n[0])) {
		return n[s];
	}
	return s == 0xff ? "SYS_ERR" : "invalid";
}

/* ---- coherent blocks: all of C4's pool, taken once and kept by size ----
 * Every block is inside the pool window and the caller's [iova_start, iova_stop]. The pool does not give
 * memory back, so a block that is let go goes to the free list of its size, and what the controller
 * holds is returned to dma_free_coherent() only when it is freed. */
static void *
blk_get(struct mhi_priv *p, size_t size, dma_addr_t *dma)
{
	struct mhi_controller *c = p->c;
	struct mhi_blk *b = NULL;
	int i;
	void *va;
	dma_addr_t d;

	if (size == 0) {
		return NULL;
	}
	L(p);
	for (i = 0; i < MAX_BLK; i++) {
		if (p->blk[i].taken && !p->blk[i].used && p->blk[i].size == size) {
			b = &p->blk[i];
			break;
		}
	}
	if (b) {
		b->used = true;
		U(p);
		memset(b->va, 0, size);
		*dma = b->dma;
		return b->va;
	}
	for (i = 0; i < MAX_BLK; i++) {
		if (!p->blk[i].taken) {
			b = &p->blk[i];
			b->taken = true;
			b->used = true;
			b->size = size;
			b->va = NULL;
			break;
		}
	}
	U(p);
	if (!b) {
		MHI_LOG("no room for another DMA block (%zu bytes)", size);
		return NULL;
	}
	va = dma_alloc_coherent(c->cntrl_dev, size, &d, GFP_KERNEL);
	if (va == NULL) {
		MHI_LOG("no coherent memory for %zu bytes", size);
	} else if ((u64)d < p->win_lo || (u64)d + size - 1 < (u64)d || (u64)d + size - 1 > p->win_hi) {
		MHI_LOG("DMA block %#llx+%zu is outside the pool window %#llx-%#llx", (unsigned long long)d,
		    size, (unsigned long long)p->win_lo, (unsigned long long)p->win_hi);
		dma_free_coherent(c->cntrl_dev, size, va, d);
		va = NULL;
	}
	L(p);
	if (va == NULL) {
		b->taken = false;
		b->used = false;
		U(p);
		return NULL;
	}
	b->va = va;
	b->dma = d;
	U(p);
	memset(va, 0, size);
	*dma = d;
	return va;
}
static void
blk_put(struct mhi_priv *p, void *va)
{
	int i;

	if (va == NULL) {
		return;
	}
	L(p);
	for (i = 0; i < MAX_BLK; i++) {
		if (p->blk[i].taken && p->blk[i].va == va) {
			p->blk[i].used = false;
			break;
		}
	}
	U(p);
}
static void
blk_free_all(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	int i;

	for (i = 0; i < MAX_BLK; i++) {
		if (p->blk[i].taken && p->blk[i].va) {
			dma_free_coherent(c->cntrl_dev, p->blk[i].size, p->blk[i].va, p->blk[i].dma);
		}
		memset(&p->blk[i], 0, sizeof(p->blk[i]));
	}
}

/* a ring of n elements, its base aligned to its length (Linux mhi_alloc_aligned_ring) */
static int
ring_alloc(struct mhi_priv *p, struct mhi_ring *r, u32 n)
{
	u64 len = (u64)n * sizeof(struct mhi_el);
	u64 align = 1;
	dma_addr_t d, base;
	void *va;

	if (n < 2) {
		return -EINVAL;
	}
	while (align < len) {
		align <<= 1;
	}
	va = blk_get(p, (size_t)(len + align - 1), &d);
	if (!va) {
		return -ENOMEM;
	}
	base = (d + align - 1) & ~(dma_addr_t)(align - 1);
	r->blk = va;
	r->iova = base;
	r->el = (struct mhi_el *)((u8 *)va + (base - d));
	r->n = n;
	r->rp = r->wp = 0;
	return 0;
}
static void
ring_free(struct mhi_priv *p, struct mhi_ring *r)
{
	blk_put(p, r->blk);
	memset(r, 0, sizeof(*r));
}
static inline dma_addr_t
ring_iova(const struct mhi_ring *r, u32 idx)
{
	return r->iova + (dma_addr_t)idx * sizeof(struct mhi_el);
}
static inline u32
ring_avail(const struct mhi_ring *r)
{
	return (r->rp + r->n - r->wp - 1) % r->n;
}
/* a pointer the device wrote, as an element index: false if it is not inside the ring */
static bool
ring_index(const struct mhi_ring *r, u64 addr, u32 *idx)
{
	if (!r->el || addr < r->iova || addr >= r->iova + (u64)r->n * sizeof(struct mhi_el) ||
	    (addr & (sizeof(struct mhi_el) - 1))) {
		return false;
	}
	*idx = (u32)((addr - r->iova) / sizeof(struct mhi_el));
	return true;
}

/* ---- dumps ---- */
/* wifi_mhi_dump=1: the registers the BHI/MHI keep of a failed or stalled boot (iOSPlus10 0339/0343).
 * The SBL log lives in the chip's SRAM, reached through the window of the PCI layer, not here: the
 * pointer and size the SBL left in ERRDBG2/3 are printed. */
static void
dump(struct mhi_priv *p, const char *why)
{
	struct mhi_controller *c = p->c;
	u32 v[14], i;
	static const u32 mhi_off[] = { R_MHICFG, R_MHICTRL, R_MHISTATUS };
	static const u32 bhi_off[] = { BHI_EXECENV, BHI_STATUS, BHI_ERRCODE, BHI_ERRDBG1, BHI_ERRDBG2,
	    BHI_ERRDBG3, BHI_IMGADDR_HIGH, BHI_IMGADDR_LOW, BHI_IMGSIZE, BHI_INTVEC };

	if (!lx_os_option("wifi_mhi_dump")) {
		return;
	}
	if (!(p->pm & PM_REG_OK) && !(p->pm & PM_DISABLE)) {
		MHI_LOG("dump (%s): registers not accessible in this state", why);
		return;
	}
	for (i = 0; i < 3; i++) {
		if (rd(c, c->regs, mhi_off[i], &v[i])) {
			v[i] = 0xdeadbeef;
		}
	}
	MHI_LOG("dump (%s): MHICFG %#x MHICTRL %#x MHISTATUS %#x pm %#x ee %s state %s", why, v[0], v[1], v[2],
	    p->pm, ee_name(p->ee), state_name(p->dev_state));
	if (!p->bhi) {
		MHI_LOG("dump (%s): BHI offset not read yet", why);
		return;
	}
	for (i = 0; i < 10; i++) {
		if (rd(c, p->bhi, bhi_off[i], &v[i])) {
			v[i] = 0xdeadbeef;
		}
	}
	MHI_LOG("dump (%s): BHI EXECENV %#x STATUS %#x ERRCODE %#x ERRDBG1 %#x IMGADDR %#x:%#x IMGSIZE %#x INTVEC %#x",
	    why, v[0], v[1], v[2], v[3], v[6], v[7], v[8], v[9]);
	MHI_LOG("dump (%s): SBL log pointer (ERRDBG2) %#x size (ERRDBG3) %#x", why, v[4], v[5]);
	if (p->bhie) {
		u32 s = 0;

		if (!rd(c, p->bhie, BHIE_TXVECSTATUS_OFFS, &s)) {
			MHI_LOG("dump (%s): BHIe TXVECSTATUS %#x", why, s);
		}
	}
}

/* ---- state helpers ---- */
static bool
aborted(struct mhi_priv *p)
{
	return (p->pm & (PM_ERRORS | PM_DISABLE)) != 0;
}

static void
state_set(struct mhi_priv *p, u32 pm)
{
	L(p);
	p->pm = pm;
	U(p);
}

/* device wake: the count of who needs the device awake; the doorbell is rung on 0 -> 1 and back.
 * Linux rings it only in M0/M2; here M0 is the only state. Called under L. */
static void
wake_get_locked(struct mhi_priv *p)
{
	p->dev_wake++;
	if ((p->pm & PM_M0) && !p->wake_set && p->wake_db) {
		write_db(p->c, p->wake_db, 1);
		p->wake_set = true;
	}
}
static void
wake_put_locked(struct mhi_priv *p)
{
	if (p->dev_wake) {
		p->dev_wake--;
	}
	if (p->dev_wake == 0 && p->wake_set && p->wake_db && (p->pm & PM_REG_OK)) {
		write_db(p->c, p->wake_db, 0);
		p->wake_set = false;
	}
}

/* ---- doorbells ---- */
static void
er_db(struct mhi_priv *p, struct mhi_event *e)	/* under L */
{
	u64 val = ctx_get64(p->er_ctx[e->index].wp);

	if (!e->db) {
		return;
	}
	if (e->brst == MHI_DB_BRST_ENABLE) {
		if (e->db_mode) {
			write_db(p->c, e->db, val);
			e->db_mode = false;
		}
	} else {
		write_db(p->c, e->db, val);
	}
}
static void
chan_db(struct mhi_priv *p, struct mhi_chan *ch)	/* under L */
{
	u64 val = ring_iova(&ch->tre, ch->tre.wp);

	fence();
	ctx_set64(p->chan_ctx[ch->num].wp, val);
	if (ch->brst == MHI_DB_BRST_ENABLE) {
		if (ch->db_mode) {
			write_db(p->c, ch->db, val);
			ch->db_mode = false;
		}
	} else {
		write_db(p->c, ch->db, val);
	}
}
static void
cmd_db(struct mhi_priv *p)	/* under L */
{
	u64 val = ring_iova(&p->cmd, p->cmd.wp);

	fence();
	ctx_set64(p->cmd_ctx[0].wp, val);
	write_db(p->c, (u8 *)p->c->regs + R_CRDB_LOWER, val);
}

/* ---- waiting: services the event rings, so a wait for an event can end ---- */
static int pump(struct mhi_priv *p);
#define MHI_WAIT(p, cond, ms, rc)						\
	do {									\
		u64 _end = lx_os_nanoseconds() + (u64)(ms) * 1000000ull;	\
		(rc) = 0;							\
		for (;;) {							\
			if (cond) {						\
				break;						\
			}							\
			if (aborted(p)) {					\
				(rc) = -EIO;					\
				break;						\
			}							\
			if (lx_os_nanoseconds() >= _end) {			\
				(rc) = -ETIMEDOUT;				\
				break;						\
			}							\
			(void)pump(p);						\
			if (cond) {						\
				break;						\
			}							\
			lx_pause_ms(1);						\
		}								\
	} while (0)

/* poll one register field until it equals val: -ETIMEDOUT, -EIO (read fault or a dead link) */
static int
poll_field_x(struct mhi_priv *p, void *base, u32 off, u32 mask, u32 val, u32 timeout_ms, bool honor_abort)
{
	struct mhi_controller *c = p->c;
	u64 end = lx_os_nanoseconds() + (u64)timeout_ms * 1000000ull;

	for (;;) {
		u32 raw, out;
		int r = rd(c, base, off, &raw);

		if (r) {
			return r;
		}
		if (raw == 0xffffffffu) {
			return -EIO;	/* what a dead PCIe link reads */
		}
		out = (raw & mask) >> ctz32(mask);
		if (out == val) {
			return 0;
		}
		if (honor_abort && aborted(p)) {
			return -EIO;
		}
		if (lx_os_nanoseconds() >= end) {
			return -ETIMEDOUT;
		}
		lx_pause_ms(1);
	}
}
#define poll_field(p, base, off, mask, val, ms) poll_field_x(p, base, off, mask, val, ms, true)
#define poll_field_noabort(p, base, off, mask, val, ms) poll_field_x(p, base, off, mask, val, ms, false)

static enum mhi_ee_type
get_exec_env(struct mhi_priv *p)
{
	u32 v;

	if (!p->bhi || rd(p->c, p->bhi, BHI_EXECENV, &v) || v > MHI_EE_MAX_SUPPORTED) {
		return MHI_EE_MAX;
	}
	return (enum mhi_ee_type)v;
}

static void
set_mhi_state(struct mhi_priv *p, enum mhi_state st)
{
	struct mhi_controller *c = p->c;
	int r;

	if (st == MHI_STATE_RESET) {
		r = wr_field(c, c->regs, R_MHICTRL, MHICTRL_RESET_MASK, 1);
	} else {
		r = wr_field(c, c->regs, R_MHICTRL, MHICTRL_MHISTATE_MASK, st);
	}
	if (r) {
		MHI_LOG("could not set MHI state %s", state_name(st));
	}
}

enum mhi_state
mhi_get_mhi_state(struct mhi_controller *c)
{
	u32 s;

	if (!c || rd_field(c, c->regs, R_MHISTATUS, MHISTATUS_MHISTATE_MASK, &s)) {
		return MHI_STATE_MAX;
	}
	return (enum mhi_state)s;
}

/* ---- the contexts: Linux mhi_init_dev_ctxt ---- */
static void
dev_ctxt_deinit(struct mhi_priv *p)
{
	u32 i;

	for (i = 0; i < p->n_ev; i++) {
		ring_free(p, &p->ev[i].ring);
	}
	ring_free(p, &p->cmd);
	blk_put(p, p->chan_ctx);
	blk_put(p, p->er_ctx);
	blk_put(p, p->cmd_ctx);
	p->chan_ctx = p->er_ctx = p->cmd_ctx = NULL;
	p->prepared = false;
}

static int
dev_ctxt_init(struct mhi_priv *p)
{
	u32 i;
	int r = -ENOMEM;

	p->dev_wake = 0;
	p->pending_pkts = 0;
	p->wake_set = false;
	p->wake_hold = false;
	p->chan_ctx = blk_get(p, (size_t)p->max_chan * sizeof(struct mhi_ctx), &p->chan_ctx_dma);
	p->er_ctx = blk_get(p, (size_t)p->n_ev * sizeof(struct mhi_ctx), &p->er_ctx_dma);
	p->cmd_ctx = blk_get(p, sizeof(struct mhi_ctx), &p->cmd_ctx_dma);
	if (!p->chan_ctx || !p->er_ctx || !p->cmd_ctx) {
		goto fail;
	}
	for (i = 0; i < p->max_chan; i++) {
		struct mhi_chan *ch = &p->chan[i];

		if (!ch->configured) {
			continue;
		}
		p->chan_ctx[i].w0 = (CH_DISABLED) | ((u32)ch->brst << 8) | (ch->pollcfg << 10);
		p->chan_ctx[i].w1 = (u32)ch->type;
		p->chan_ctx[i].w2 = ch->er;
		ch->state = CH_DISABLED;
	}
	for (i = 0; i < p->n_ev; i++) {
		struct mhi_event *e = &p->ev[i];

		p->er_ctx[i].w0 = e->intmod << 16;
		p->er_ctx[i].w1 = ER_TYPE_VALID;
		p->er_ctx[i].w2 = e->irq;
		e->db_mode = true;
		r = ring_alloc(p, &e->ring, e->elements);
		if (r) {
			goto fail;
		}
		ctx_set64(p->er_ctx[i].rbase, e->ring.iova);
		ctx_set64(p->er_ctx[i].rlen, (u64)e->ring.n * sizeof(struct mhi_el));
		ctx_set64(p->er_ctx[i].rp, e->ring.iova);
		ctx_set64(p->er_ctx[i].wp, e->ring.iova);
	}
	r = ring_alloc(p, &p->cmd, CMD_EL_PER_RING);
	if (r) {
		goto fail;
	}
	ctx_set64(p->cmd_ctx[0].rbase, p->cmd.iova);
	ctx_set64(p->cmd_ctx[0].rlen, (u64)p->cmd.n * sizeof(struct mhi_el));
	ctx_set64(p->cmd_ctx[0].rp, p->cmd.iova);
	ctx_set64(p->cmd_ctx[0].wp, p->cmd.iova);
	fence();
	return 0;
fail:
	dev_ctxt_deinit(p);
	return r ? r : -ENOMEM;
}

/* Linux mhi_init_mmio: after READY */
static int
init_mmio(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	u32 val, i;
	struct { u32 off, v; } regs[] = {
		{ R_CCABAP_HIGHER, hi32(p->chan_ctx_dma) },
		{ R_CCABAP_LOWER, lo32(p->chan_ctx_dma) },
		{ R_ECABAP_HIGHER, hi32(p->er_ctx_dma) },
		{ R_ECABAP_LOWER, lo32(p->er_ctx_dma) },
		{ R_CRCBAP_HIGHER, hi32(p->cmd_ctx_dma) },
		{ R_CRCBAP_LOWER, lo32(p->cmd_ctx_dma) },
		{ R_MHICTRLBASE_HIGHER, hi32(p->win_lo) },
		{ R_MHICTRLBASE_LOWER, lo32(p->win_lo) },
		{ R_MHIDATABASE_HIGHER, hi32(p->win_lo) },
		{ R_MHIDATABASE_LOWER, lo32(p->win_lo) },
		{ R_MHICTRLLIMIT_HIGHER, hi32(p->win_hi) },
		{ R_MHICTRLLIMIT_LOWER, lo32(p->win_hi) },
		{ R_MHIDATALIMIT_HIGHER, hi32(p->win_hi) },
		{ R_MHIDATALIMIT_LOWER, lo32(p->win_hi) },
	};

	if (rd(c, c->regs, R_CHDBOFF, &val)) {
		MHI_LOG("cannot read CHDBOFF");
		return -EIO;
	}
	if (val >= c->reg_len - 8 * MHI_DEV_WAKE_DB) {
		MHI_LOG("CHDB offset %#x is out of range (BAR %#zx)", val, c->reg_len);
		return -ERANGE;
	}
	p->chdboff = val;
	p->wake_db = (u8 *)c->regs + val + 8 * MHI_DEV_WAKE_DB;
	p->wake_set = false;
	for (i = 0; i < p->max_chan; i++) {
		p->chan[i].db = (u8 *)c->regs + val + 8 * i;
	}
	if (rd(c, c->regs, R_ERDBOFF, &val)) {
		MHI_LOG("cannot read ERDBOFF");
		return -EIO;
	}
	if (val >= c->reg_len - 8 * p->n_ev) {
		MHI_LOG("ERDB offset %#x is out of range (BAR %#zx)", val, c->reg_len);
		return -ERANGE;
	}
	p->erdboff = val;
	for (i = 0; i < p->n_ev; i++) {
		p->ev[i].db = (u8 *)c->regs + val + 8 * i;
	}
	for (i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		wr(c, c->regs, regs[i].off, regs[i].v);
	}
	if (wr_field(c, c->regs, R_MHICFG, MHICFG_NER_MASK, p->n_ev) ||
	    wr_field(c, c->regs, R_MHICFG, MHICFG_NHWER_MASK, 0)) {
		MHI_LOG("cannot write MHICFG");
		return -EIO;
	}
	return 0;
}

/* ---- images: Linux boot.c ---- */
/* image_info (mhi.h) only points at blocks held in p->blk; the vector table entry is Linux's */
struct bhi_vec_entry {
	u64 dma_addr;
	u64 size;
};
_Static_assert(sizeof(struct bhi_vec_entry) == 16, "vector entries are 16 bytes");

static void
image_free(struct mhi_priv *p, struct image_info *img)
{
	u32 i;

	if (!img) {
		return;
	}
	for (i = 0; i < img->entries; i++) {
		blk_put(p, img->mhi_buf[i].buf);
	}
	kfree(img->mhi_buf);
	kfree(img);
}

/* A vector table for an image of alloc_size bytes in segments of seg_len; the last entry is the table */
static int
image_table_alloc(struct mhi_priv *p, struct image_info **out, size_t alloc_size, size_t seg_len)
{
	struct mhi_controller *c = p->c;
	size_t segments;
	struct image_info *img;
	size_t i;

	(void)c;
	if (!seg_len || !alloc_size) {
		return -EINVAL;
	}
	segments = (alloc_size + seg_len - 1) / seg_len + 1;
	img = kzalloc(sizeof(*img), GFP_KERNEL);
	if (!img) {
		return -ENOMEM;
	}
	img->mhi_buf = kzalloc(sizeof(*img->mhi_buf) * segments, GFP_KERNEL);
	if (!img->mhi_buf) {
		kfree(img);
		return -ENOMEM;
	}
	for (i = 0; i < segments; i++) {
		struct mhi_buf *b = &img->mhi_buf[i];
		size_t vec = seg_len;

		if (i == segments - 1) {
			vec = sizeof(struct bhi_vec_entry) * i;
		}
		b->len = vec;
		b->buf = blk_get(p, vec, &b->dma_addr);
		if (!b->buf) {
			img->entries = (u32)i;
			image_free(p, img);
			return -ENOMEM;
		}
	}
	img->bhi_vec = img->mhi_buf[segments - 1].buf;
	img->entries = (u32)segments;
	*out = img;
	return 0;
}

/* Copy the image into the table's segments and fill the vector (Linux mhi_firmware_copy_bhie) */
static void
image_copy(const u8 *src, size_t remainder, struct image_info *img)
{
	struct mhi_buf *b = img->mhi_buf;
	struct bhi_vec_entry *v = (struct bhi_vec_entry *)img->bhi_vec;

	while (remainder) {
		size_t n = remainder < b->len ? remainder : b->len;

		memcpy(b->buf, src, n);
		v->dma_addr = b->dma_addr;
		v->size = n;
		src += n;
		remainder -= n;
		v++;
		b++;
	}
	fence();
}

static u32
random_seq(u32 mask)
{
	u32 v = 0;

	lx_os_random(&v, sizeof(v));
	v &= mask;
	return v ? v : 1;
}

/* BHI: one buffer of the first sbl_size bytes, to the PBL (Linux mhi_fw_load_bhi) */
static int
load_bhi(struct mhi_priv *p, const u8 *data, size_t size)
{
	struct mhi_controller *c = p->c;
	dma_addr_t dma;
	void *buf = blk_get(p, size, &dma);
	u32 seq = random_seq(BHI_TXDB_SEQNUM_BMSK), status = 0;
	int r;

	if (!buf) {
		return -ENOMEM;
	}
	memcpy(buf, data, size);
	fence();
	MHI_LOG("BHI: %zu bytes at %#llx, session %u", size, (unsigned long long)dma, seq);
	wr(c, p->bhi, BHI_STATUS, 0);
	wr(c, p->bhi, BHI_IMGADDR_HIGH, hi32(dma));
	wr(c, p->bhi, BHI_IMGADDR_LOW, lo32(dma));
	wr(c, p->bhi, BHI_IMGSIZE, (u32)size);
	wr(c, p->bhi, BHI_IMGTXDB, seq);
	MHI_WAIT(p, (rd_field(c, p->bhi, BHI_STATUS, BHI_STATUS_MASK, &status) == 0 && status != 0),
	    c->timeout_ms, r);
	if (r == 0 && status == BHI_STATUS_ERROR) {
		MHI_LOG("BHI: image transfer failed");
		dump(p, "BHI error");
		r = -EIO;
	} else if (r == 0 && status != BHI_STATUS_SUCCESS) {
		MHI_LOG("BHI: unexpected status %u", status);
		r = -EIO;
	} else if (r == -ETIMEDOUT) {
		MHI_LOG("BHI: no completion after %u ms", c->timeout_ms);
		dump(p, "BHI stall");
	}
	blk_put(p, buf);
	if (r == 0) {
		MHI_LOG("BHI: done");
	}
	return r;
}

/* BHIe: the vector table of an image, to the SBL (Linux mhi_fw_load_bhie) */
static int
load_bhie(struct mhi_priv *p, const struct mhi_buf *table)
{
	struct mhi_controller *c = p->c;
	u32 seq = random_seq(BHIE_TXVECDB_SEQNUM_BMSK), status = 0;
	int r;

	if (!(p->pm & PM_REG_OK)) {
		return -EIO;
	}
	wr(c, p->bhie, BHIE_TXVECADDR_HIGH_OFFS, hi32(table->dma_addr));
	wr(c, p->bhie, BHIE_TXVECADDR_LOW_OFFS, lo32(table->dma_addr));
	wr(c, p->bhie, BHIE_TXVECSIZE_OFFS, (u32)table->len);
	r = wr_field(c, p->bhie, BHIE_TXVECDB_OFFS, BHIE_TXVECDB_SEQNUM_BMSK, seq);
	if (r) {
		return r;
	}
	MHI_WAIT(p, (rd_field(c, p->bhie, BHIE_TXVECSTATUS_OFFS, BHIE_TXVECSTATUS_STATUS_BMSK, &status) == 0 &&
	    status != 0), c->timeout_ms, r);
	if (r == -ETIMEDOUT) {
		MHI_LOG("BHIe: no completion after %u ms", c->timeout_ms);
		dump(p, "BHIe stall");
		return r;
	}
	if (r) {
		return r;
	}
	if (status != BHIE_TXVECSTATUS_STATUS_XFER_COMPL) {
		MHI_LOG("BHIe: transfer failed, status %u", status);
		dump(p, "BHIe error");
		return -EIO;
	}
	return 0;
}

/* ---- failures of the power-up path ---- */
static void
wake_all(struct mhi_priv *p)
{
	lx_os_lock();
	lx_os_wakeup((const void *)&p->pm);
	lx_os_unlock();
}

static void
fail_pm(struct mhi_priv *p, u32 pm, int err, const char *why)
{
	L(p);
	if (!(p->pm & PM_ERRORS) && !(p->pm & PM_DISABLE)) {
		p->pm = pm;
		p->fail_err = err ? err : -EIO;
	}
	U(p);
	MHI_LOG("%s", why);
	wake_all(p);
}

/* ---- firmware choice and the PBL step: Linux mhi_fw_load_handler ---- */
static int ready_transition(struct mhi_priv *p);

static void
fw_load_handler(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	const u8 *fw_data = c->fw_data;
	size_t fw_sz = c->fw_sz, size;
	enum mhi_ee_type ee;
	int r;
	struct image_info *img = NULL;

	if (p->pm & PM_ERRORS) {
		return;
	}
	if (rd(c, p->bhi, BHI_SERIALNU, &p->serial)) {
		MHI_LOG("could not read the serial number over BHI");
	}
	L(p);
	if (p->pm & PM_REG_OK) {
		p->ee = ee = get_exec_env(p);
	} else {
		ee = MHI_EE_MAX;
	}
	U(p);
	MHI_LOG("EE %s", ee_name(ee));
	if (ee != MHI_EE_PBL) {
		if (ee == MHI_EE_PTHRU) {
			goto ready;	/* nothing to load */
		}
		fail_pm(p, PM_FW_DL_ERR, -EOPNOTSUPP, ee == MHI_EE_EDL ? "EDL: no EDL image in this controller" :
		    "the execution environment cannot take a firmware load");
		return;
	}
	if (!c->fw_image && c->fbc_download && fw_data && fw_sz) {
		if (!c->sbl_size) {
			fail_pm(p, PM_FW_DL_ERR, -EINVAL, "fw_data given but no sbl_size");
			return;
		}
	} else if (c->fw_image) {
		unsigned long n = 0;
		const void *d = NULL;
		int e = lx_os_firmware(c->fw_image, &d, &n);

		if (e || !d || !n) {
			lx_tell("mhi: firmware %s: error %d\n", c->fw_image, e);
			fail_pm(p, PM_FW_DL_ERR, e ? -e : -ENOENT, "no firmware");
			return;
		}
		fw_data = d;
		fw_sz = n;
	} else {
		fail_pm(p, PM_FW_DL_ERR, -EINVAL, "no firmware image defined");
		return;
	}
	if (c->fbc_download && (!c->sbl_size || !c->seg_len)) {
		fail_pm(p, PM_FW_DL_ERR, -EINVAL, "fbc download needs sbl_size and seg_len");
		return;
	}
	size = c->fbc_download ? c->sbl_size : fw_sz;
	if (size > fw_sz) {
		size = fw_sz;	/* the SBL size is a maximum */
	}
	MHI_LOG("image %zu bytes, SBL part %zu, segments %zu, %s", fw_sz, size, (size_t)c->seg_len,
	    c->fbc_download ? "FBC (BHI then BHIe)" : (c->seg_len ? "BHIe" : "BHI"));
	if (!c->fbc_download && c->seg_len) {
		/* BHIe only in the PBL (Linux MHI_FW_LOAD_BHIE) */
		r = image_table_alloc(p, &img, size, c->seg_len);
		if (r == 0) {
			image_copy(fw_data, size, img);
			r = load_bhie(p, &img->mhi_buf[img->entries - 1]);
			image_free(p, img);
		}
	} else {
		r = load_bhi(p, fw_data, size);
	}
	if (r) {
		fail_pm(p, PM_FW_DL_ERR, r, "the image did not load over BHI");
		return;
	}
	L(p);
	p->dev_state = MHI_STATE_RESET;
	U(p);
	if (c->fbc_download) {
		/* the second ELF of a combined file starts after the SBL */
		if (fw_sz > c->sbl_size && !memcmp(fw_data + c->sbl_size, "\177ELF", 4)) {
			fw_data += c->sbl_size;
			fw_sz -= c->sbl_size;
		}
		r = image_table_alloc(p, &img, fw_sz, c->seg_len);
		if (r) {
			fail_pm(p, PM_FW_DL_ERR, r, "no memory for the BHIe table");
			return;
		}
		image_copy(fw_data, fw_sz, img);
		c->fbc_image = img;
		MHI_LOG("BHIe table: %zu bytes in %u segments of %zu, vector %#llx", fw_sz, img->entries - 1,
		    (size_t)c->seg_len, (unsigned long long)img->mhi_buf[img->entries - 1].dma_addr);
	}
ready:
	r = ready_transition(p);
	if (r) {
		if (c->fbc_image) {
			image_free(p, c->fbc_image);
			c->fbc_image = NULL;
		}
		fail_pm(p, PM_FW_DL_ERR, r, "the device did not become READY");
		return;
	}
	MHI_LOG("waiting for the device to enter SBL or mission mode");
}

/* ---- RESET -> READY -> contexts -> event rings -> M0: Linux mhi_ready_state_transition ---- */
static int
ready_transition(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	u32 i, to;
	int r;

	if (p->pm & PM_LD_ERR_FATAL) {
		MHI_LOG("link is not accessible");
		return -EIO;
	}
	r = poll_field(p, c->regs, R_MHICTRL, MHICTRL_RESET_MASK, 0, c->timeout_ms);
	if (r) {
		MHI_LOG("the device did not clear MHI reset (%d)", r);
		dump(p, "RESET stall");
		return r;
	}
	to = p->ready_timeout_ms ? p->ready_timeout_ms : c->timeout_ms;
	r = poll_field(p, c->regs, R_MHISTATUS, MHISTATUS_READY_MASK, 1, to);
	if (r) {
		MHI_LOG("the device did not enter READY after %u ms (%d)", to, r);
		dump(p, "READY stall");
		return r;
	}
	L(p);
	if (p->pm & PM_ERRORS) {
		U(p);
		return -EIO;
	}
	p->pm = PM_POR;
	p->dev_state = MHI_STATE_READY;
	U(p);
	MHI_LOG("READY");
	r = init_mmio(p);
	if (r) {
		MHI_LOG("could not configure the MMIO registers");
		return r;
	}
	MHI_LOG("MMIO contexts programmed: chan %#llx ev %#llx cmd %#llx, window %#llx-%#llx",
	    (unsigned long long)p->chan_ctx_dma, (unsigned long long)p->er_ctx_dma,
	    (unsigned long long)p->cmd_ctx_dma, (unsigned long long)p->win_lo, (unsigned long long)p->win_hi);
	L(p);
	for (i = 0; i < p->n_ev; i++) {
		struct mhi_event *e = &p->ev[i];

		/* the ring is the device's but one element */
		e->ring.wp = e->ring.n - 1;
		ctx_set64(p->er_ctx[i].wp, ring_iova(&e->ring, e->ring.wp));
		fence();
		er_db(p, e);
	}
	p->m0_requested = true;
	set_mhi_state(p, MHI_STATE_M0);
	U(p);
	MHI_LOG("M0 requested");
	return 0;
}

/* M0 reached (an event, or MHISTATUS): Linux mhi_pm_m0_transition. Called under L. */
static void
m0_transition(struct mhi_priv *p, const char *how)
{
	u32 i;

	if (!(p->pm & (PM_POR | PM_M0))) {
		return;
	}
	p->dev_state = MHI_STATE_M0;
	p->pm = PM_M0;
	if (!p->wake_hold) {
		p->wake_hold = true;
		wake_get_locked(p);
	}
	if (p->ee == MHI_EE_AMSS || p->ee == MHI_EE_WFW || p->ee == MHI_EE_FP) {
		for (i = 0; i < p->n_ev; i++) {
			er_db(p, &p->ev[i]);
		}
		if (p->cmd.rp != p->cmd.wp) {
			cmd_db(p);
		}
	}
	for (i = 0; i < p->max_chan; i++) {
		struct mhi_chan *ch = &p->chan[i];

		if (ch->tre.el && ch->tre.wp != ch->tre.rp && ch->state == CH_ENABLED) {
			chan_db(p, ch);
		}
	}
	lx_os_lock();
	lx_os_wakeup((const void *)&p->pm);
	lx_os_unlock();
	MHI_LOG("M0 (%s), device wake asserted", how);
}

/* ---- client devices and drivers: the bus, in a table ---- */
static struct mhi_device *
dev_alloc(struct mhi_controller *c, const char *name)
{
	struct mhi_device *d = kzalloc(sizeof(*d), GFP_KERNEL);

	if (!d) {
		return NULL;
	}
	d->mhi_cntrl = c;
	d->name = name;
	d->dev_type = MHI_DEVICE_XFER;
	d->ul_chan_id = d->dl_chan_id = -1;
	return d;
}

/* Linux's struct mhi_device_id (mod_devicetable.h) */
struct mhi_device_id {
	char chan[MHI_NAME_SIZE];
	unsigned long driver_data;
};

static void chan_reset(struct mhi_priv *p, struct mhi_chan *ch, bool command);
static void chan_wait_idle(struct mhi_chan *ch);

static const struct mhi_device_id *
drv_match(const struct mhi_driver *drv, const char *name)
{
	const struct mhi_device_id *id;

	for (id = drv->id_table; id && id->chan[0]; id++) {
		if (!strcmp(id->chan, name)) {
			return id;
		}
	}
	return NULL;
}

static void
dev_bind(struct mhi_priv *p, struct mhi_dev_slot *s)
{
	u32 i;

	if (s->bound) {
		return;
	}
	for (i = 0; i < MAX_DRV; i++) {
		const struct mhi_driver *drv = drivers[i];
		const struct mhi_device_id *id;
		int r;

		if (!drv || !(id = drv_match(drv, s->dev->name))) {
			continue;
		}
		if ((s->dev->ul_chan && !drv->ul_xfer_cb) || (s->dev->dl_chan && !drv->dl_xfer_cb) || !drv->probe) {
			MHI_LOG("driver for %s lacks a probe or a transfer callback", s->dev->name);
			continue;
		}
		s->drv = drv;
		s->id = id;
		s->dev->id = id;
		if (s->dev->ul_chan) {
			s->dev->ul_chan->xfer_cb = drv->ul_xfer_cb;
		}
		if (s->dev->dl_chan) {
			s->dev->dl_chan->xfer_cb = drv->dl_xfer_cb;
		}
		r = drv->probe(s->dev, id);
		if (r) {
			MHI_LOG("probe of %s failed: %d", s->dev->name, r);
			if (drv == &ipcr_driver) {
				p->builtin_err = r ? r : -EIO;
			}
			s->drv = NULL;
			s->id = NULL;
			s->dev->id = NULL;
			if (s->dev->ul_chan) {
				s->dev->ul_chan->xfer_cb = NULL;
			}
			if (s->dev->dl_chan) {
				s->dev->dl_chan->xfer_cb = NULL;
			}
			continue;
		}
		s->bound = true;
		return;
	}
}

static void
dev_unbind(struct mhi_priv *p, struct mhi_dev_slot *s)
{
	if (s->bound && s->drv && s->drv->remove) {
		s->drv->remove(s->dev);
	}
	if (s->dev->ul_chan) {
		chan_reset(p, s->dev->ul_chan, true);
	}
	if (s->dev->dl_chan) {
		chan_reset(p, s->dev->dl_chan, true);
	}
	s->bound = false;
	s->drv = NULL;
	s->id = NULL;
	if (s->dev->ul_chan) {
		s->dev->ul_chan->xfer_cb = NULL;
	}
	if (s->dev->dl_chan) {
		s->dev->dl_chan->xfer_cb = NULL;
	}
}

/* Linux mhi_create_devices: one device for a channel, or for two neighbours of one name */
static void
create_devices(struct mhi_priv *p, enum mhi_ee_type ee)
{
	struct mhi_controller *c = p->c;
	u32 i;

	for (i = 0; i < p->max_chan; i++) {
		struct mhi_chan *ch = &p->chan[i], *pair = NULL;
		struct mhi_device *d;
		u32 k;

		if (!ch->configured || !(ch->ee_mask & (1u << ee)) || ch->dev) {
			continue;
		}
		for (k = 0; k < MAX_DEV && p->dev_slot[k].dev; k++) {
		}
		if (k == MAX_DEV) {
			MHI_LOG("too many client devices");
			return;
		}
		d = dev_alloc(c, ch->name);
		if (!d) {
			MHI_LOG("no memory for a client device");
			return;
		}
		if (ch->dir == DMA_TO_DEVICE) {
			d->ul_chan = ch;
			d->ul_chan_id = (int)ch->num;
		} else {
			d->dl_chan = ch;
			d->dl_chan_id = (int)ch->num;
		}
		if (i + 1 < p->max_chan && p->chan[i + 1].configured && p->chan[i + 1].name && ch->name &&
		    !strcmp(p->chan[i + 1].name, ch->name) && (p->chan[i + 1].ee_mask & (1u << ee))) {
			pair = &p->chan[i + 1];
			if (pair->dir == DMA_TO_DEVICE && !d->ul_chan) {
				d->ul_chan = pair;
				d->ul_chan_id = (int)pair->num;
			} else if (pair->dir != DMA_TO_DEVICE && !d->dl_chan) {
				d->dl_chan = pair;
				d->dl_chan_id = (int)pair->num;
			} else {
				pair = NULL;
			}
		}
		ch->dev = d;
		if (pair) {
			pair->dev = d;
			i++;
		}
		p->dev_slot[k].dev = d;
		p->dev_slot[k].drv = NULL;
		p->dev_slot[k].bound = false;
		MHI_LOG("device %s: UL channel %d, DL channel %d", d->name, d->ul_chan_id, d->dl_chan_id);
		dev_bind(p, &p->dev_slot[k]);
	}
}

static void
destroy_devices(struct mhi_priv *p)
{
	u32 k;

	for (k = 0; k < MAX_DEV; k++) {
		struct mhi_dev_slot *s = &p->dev_slot[k];

		if (!s->dev) {
			continue;
		}
		dev_unbind(p, s);
		if (s->dev->ul_chan) {
			s->dev->ul_chan->dev = NULL;
		}
		if (s->dev->dl_chan) {
			s->dev->dl_chan->dev = NULL;
		}
		kfree(s->dev);
		memset(s, 0, sizeof(*s));
	}
}

/* ---- the event rings: Linux main.c mhi_process_ctrl_ev_ring / mhi_process_data_event_ring ----
 * One consumer per ring at a time (ev->busy). What an event asks of a client (a completion, a status
 * callback) is done after L is released. */
struct action {
	enum { A_NONE, A_STATUS, A_XFER } kind;
	enum mhi_callback cb;
	struct mhi_chan *ch;
	struct mhi_result res;
	void (*xfer)(struct mhi_device *, struct mhi_result *);
	struct mhi_device *dev;
};

/* the device completed through the TRE the event points to: one buffer for the client */
static void
xfer_event(struct mhi_priv *p, struct mhi_chan *ch, const struct mhi_el *ev, struct action *a)
{
	u32 code = (ev->dw0 >> 24) & 0xff, len = ev->dw0 & 0xffff, idx;
	struct mhi_bufinfo *bi;

	if (ch->state != CH_ENABLED) {
		return;
	}
	if (code == CC_OOB || code == CC_DB_MODE) {
		ch->db_mode = true;
		if (ch->tre.wp != ch->tre.rp && (p->pm & PM_M0)) {
			chan_db(p, ch);
		}
		return;
	}
	if (code != CC_EOT && code != CC_EOB && code != CC_OVERFLOW) {
		MHI_LOG("channel %u: transfer event code %#x ignored", ch->num, code);
		return;
	}
	if (!ring_index(&ch->tre, ev->ptr, &idx)) {
		MHI_LOG("channel %u: event points outside the transfer ring", ch->num);
		return;
	}
	if (idx != ch->tre.rp) {
		MHI_LOG("channel %u: event points to an unexpected TRE (%u, expected %u)", ch->num, idx, ch->tre.rp);
		return;
	}
	bi = &ch->buf[idx];
	if (len > bi->len) {
		len = bi->len;
	}
	if (ch->dir == DMA_FROM_DEVICE && bi->dst && len) {
		memcpy(bi->dst, ch->bb + (size_t)idx * ch->slot, len);
	}
	a->kind = A_XFER;
	a->ch = ch;
	a->res.buf_addr = bi->cb;
	a->res.bytes_xferd = len;
	a->res.dir = ch->dir;
	a->res.transaction_status = (code == CC_OVERFLOW) ? -EOVERFLOW : 0;
	a->xfer = ch->xfer_cb;
	a->dev = ch->dev;
	memset(bi, 0, sizeof(*bi));
	ch->tre.rp = (ch->tre.rp + 1) % ch->tre.n;
	if (ch->dir == DMA_TO_DEVICE && p->pending_pkts) {
		p->pending_pkts--;
	}
	if (a->xfer) {
		ch->cb_busy++;
	}
}

static void
syserr(struct mhi_priv *p, const char *how, struct action *a)	/* under L */
{
	if (!(p->pm & (PM_POR | PM_M0))) {
		return;
	}
	p->pm = PM_SYS_ERR_FAIL;
	if (!p->syserr_told) {
		p->syserr_told = true;
		a->kind = A_STATUS;
		a->cb = MHI_CB_SYS_ERROR;
	}
	MHI_LOG("system error (%s)", how);
}

static void
ctrl_event(struct mhi_priv *p, const struct mhi_el *ev, struct action *a)	/* under L */
{
	u32 type = (ev->dw1 >> 16) & 0xff, code = (ev->dw0 >> 24) & 0xff;

	switch (type) {
	case PKT_STATE_CHANGE:
		MHI_LOG("state change event: %s", state_name(code));
		if (code == MHI_STATE_M0) {
			m0_transition(p, "event");
		} else if (code == MHI_STATE_SYS_ERR) {
			syserr(p, "state change event", a);
		} else if (code == MHI_STATE_M1) {
			MHI_LOG("M1 requested by the device; device wake is held, staying in M0");
		} else {
			MHI_LOG("unexpected state %s ignored", state_name(code));
		}
		break;
	case PKT_EE_EVENT:
		MHI_LOG("EE event: %s", ee_name(code));
		if (code == MHI_EE_SBL) {
			if (!p->sbl_seen) {
				p->sbl_seen = true;
				p->pending |= T_SBL;
			}
		} else if (code == MHI_EE_AMSS || code == MHI_EE_WFW) {
			if (!p->mission_seen) {
				p->mission_seen = true;
				p->pending |= T_MISSION;
			}
		} else if (code == MHI_EE_RDDM) {
			p->ee = MHI_EE_RDDM;
			if (!p->rddm_told) {
				p->rddm_told = true;
				a->kind = A_STATUS;
				a->cb = MHI_CB_EE_RDDM;
			}
		} else {
			MHI_LOG("unhandled EE event %s", ee_name(code));
		}
		break;
	case PKT_CMD_COMPLETION: {
		u32 idx;

		if (!ring_index(&p->cmd, ev->ptr, &idx)) {
			MHI_LOG("command completion points outside the command ring");
		} else {
			u32 chn = (p->cmd.el[idx].dw1 >> 24) & 0xff;

			if (chn < p->max_chan && p->chan[chn].configured) {
				p->chan[chn].ccs = code;
				p->chan[chn].cmd_done = true;
			} else {
				MHI_LOG("command completion for an invalid channel %u", chn);
			}
			p->cmd.rp = (p->cmd.rp + 1) % p->cmd.n;
		}
		break;
	}
	case PKT_BW_REQ:
		a->kind = A_STATUS;
		a->cb = MHI_CB_BW_REQ;
		break;
	default:
		MHI_LOG("unhandled event type %#x", type);
		break;
	}
}

static void
run_action(struct mhi_priv *p, struct action *a)
{
	if (a->kind == A_STATUS) {
		p->c->status_cb(p->c, a->cb);
	} else if (a->kind == A_XFER) {
		if (a->xfer) {
			a->xfer(a->dev, &a->res);
			L(p);
			if (a->ch->cb_busy) {
				a->ch->cb_busy--;
			}
			U(p);
		}
	}
}

static int
ev_process(struct mhi_priv *p, struct mhi_event *e)
{
	int count = 0;
	u32 quota = EV_QUOTA;

	L(p);
	if (e->busy || !e->ring.el || !(p->pm & PM_EVENTS_OK)) {
		U(p);
		return 0;
	}
	e->busy = true;
	U(p);
	while (quota--) {
		struct action a = { A_NONE };
		struct mhi_el ev;
		u32 dev_idx, type, chn;
		u64 rp;

		L(p);
		if (!(p->pm & PM_EVENTS_OK)) {
			U(p);
			break;
		}
		rp = ctx_get64(p->er_ctx[e->index].rp);
		if (!ring_index(&e->ring, rp, &dev_idx)) {
			U(p);
			MHI_LOG("event ring %u: read pointer %#llx is outside the ring", e->index,
			    (unsigned long long)rp);
			count = -EIO;
			break;
		}
		if (dev_idx == e->ring.rp) {
			U(p);
			break;
		}
		fence();
		ev = e->ring.el[e->ring.rp];
		type = (ev.dw1 >> 16) & 0xff;
		chn = (ev.dw1 >> 24) & 0xff;
		if (type == PKT_TX_EVENT) {
			if (chn < p->max_chan && p->chan[chn].configured && p->chan[chn].er == e->index) {
				xfer_event(p, &p->chan[chn], &ev, &a);
			} else {
				MHI_LOG("transfer event for an unknown channel %u", chn);
			}
		} else if (e->type == MHI_ER_CTRL) {
			ctrl_event(p, &ev, &a);
		} else {
			MHI_LOG("event type %#x on data ring %u ignored", type, e->index);
		}
		U(p);
		run_action(p, &a);
		L(p);
		/* give the element back (Linux mhi_recycle_ev_ring_element) */
		e->ring.wp = (e->ring.wp + 1) % e->ring.n;
		ctx_set64(p->er_ctx[e->index].wp, ring_iova(&e->ring, e->ring.wp));
		e->ring.rp = (e->ring.rp + 1) % e->ring.n;
		U(p);
		count++;
	}
	L(p);
	if (count > 0 && (p->pm & PM_REG_OK)) {
		er_db(p, e);
	}
	e->busy = false;
	U(p);
	return count;
}

/* While powering up the device may show its EE and state in registers before (or without) an event:
 * the host looks (iOSPlus10 0261 did only this). After mission mode, SYSERR is looked for rarely. */
static void
watch(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	u32 st, ee, pm;
	bool powering;

	L(p);
	pm = p->pm;
	if (p->watch_busy || !(pm & (PM_POR | PM_M0)) || !p->bhi) {
		U(p);
		return;
	}
	powering = !p->mission_seen;
	if (!powering && (++p->watch_n & 63)) {
		U(p);
		return;
	}
	p->watch_busy = true;
	U(p);
	if (rd(c, c->regs, R_MHISTATUS, &st) == 0) {
		struct action a = { A_NONE };

		if (st == 0xffffffffu) {
			fail_pm(p, PM_LD_ERR_FATAL, -EIO, "MHISTATUS reads all ones: the PCIe link is down");
		} else {
			L(p);
			if (((st & MHISTATUS_MHISTATE_MASK) >> 8) == MHI_STATE_SYS_ERR && (p->pm & (PM_POR | PM_M0))) {
				syserr(p, "MHISTATUS", &a);
			} else if (p->m0_requested && p->dev_state == MHI_STATE_READY &&
			    ((st & MHISTATUS_MHISTATE_MASK) >> 8) == MHI_STATE_M0) {
				m0_transition(p, "MHISTATUS");
			}
			U(p);
		}
		run_action(p, &a);
	}
	if (powering && rd(c, p->bhi, BHI_EXECENV, &ee) == 0 && ee <= MHI_EE_MAX_SUPPORTED) {
		L(p);
		if (ee == MHI_EE_SBL && !p->sbl_seen && p->ee != MHI_EE_SBL && (p->pm & PM_M0)) {
			p->sbl_seen = true;
			p->pending |= T_SBL;
			MHI_LOG("EE SBL seen in BHI_EXECENV");
		} else if ((ee == MHI_EE_AMSS || ee == MHI_EE_WFW) && !p->mission_seen && (p->pm & PM_M0)) {
			p->mission_seen = true;
			p->pending |= T_MISSION;
			MHI_LOG("EE %s seen in BHI_EXECENV", ee_name(ee));
		}
		U(p);
	}
	L(p);
	p->watch_busy = false;
	U(p);
}

static int
pump(struct mhi_priv *p)
{
	int n = 0;
	u32 i;

	for (i = 0; i < p->n_ev; i++) {
		int r = ev_process(p, &p->ev[i]);

		if (r > 0) {
			n += r;
		}
	}
	watch(p);
	return n;
}

/* ---- the transitions the events ask for: SBL and mission mode (Linux mhi_pm_st_worker) ---- */
static void
sbl_transition(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	int r;

	L(p);
	p->ee = MHI_EE_SBL;
	U(p);
	MHI_LOG("EE SBL");
	create_devices(p, MHI_EE_SBL);
	if (!c->fbc_download) {
		return;
	}
	if (!c->fbc_image) {
		fail_pm(p, PM_FW_DL_ERR, -EIO, "no BHIe image to download in SBL");
		return;
	}
	MHI_LOG("BHIe: sending the image");
	r = load_bhie(p, &c->fbc_image->mhi_buf[c->fbc_image->entries - 1]);
	if (r) {
		fail_pm(p, PM_FW_DL_ERR, r, "the image did not load over BHIe");
		return;
	}
	MHI_LOG("BHIe: done, waiting for mission mode");
}

static void
mission_transition(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	enum mhi_ee_type ee = MHI_EE_MAX;

	if (p->pm & PM_REG_OK) {
		ee = get_exec_env(p);
	}
	if (!(ee == MHI_EE_AMSS || ee == MHI_EE_WFW || ee == MHI_EE_FP)) {
		fail_pm(p, PM_LD_ERR_FATAL, -EIO, "the execution environment is not mission mode");
		MHI_LOG("EE %s read for the mission mode transition", ee_name(ee));
		return;
	}
	L(p);
	p->ee = ee;
	U(p);
	MHI_LOG("EE %s: mission mode", ee_name(ee));
	c->status_cb(c, MHI_CB_EE_MISSION_MODE);
	L(p);
	wake_get_locked(p);
	U(p);
	create_devices(p, ee);
	L(p);
	wake_put_locked(p);
	p->mission_done = true;
	U(p);
	wake_all(p);
}

static void
run_transitions(struct mhi_priv *p)
{
	L(p);
	if (p->tr_busy || !p->pending) {
		U(p);
		return;
	}
	p->tr_busy = true;
	while (p->pending) {
		u32 bit = p->pending & -p->pending;

		p->pending &= ~bit;
		U(p);
		if (!(p->pm & PM_ERRORS)) {
			switch (bit) {
			case T_PBL:
				fw_load_handler(p);
				break;
			case T_READY:
				{
					int e = ready_transition(p);

					if (e) {
						fail_pm(p, PM_FW_DL_ERR, e, "the device did not become READY");
					}
				}
				break;
			case T_SBL:
				sbl_transition(p);
				break;
			case T_MISSION:
				mission_transition(p);
				break;
			}
		}
		L(p);
	}
	p->tr_busy = false;
	U(p);
}

/* the work an interrupt would have woken: both the handlers and lx_mhi_poll() end here */
static int
service(struct mhi_priv *p)
{
	int n = pump(p);

	run_transitions(p);
	return n;
}

int lx_mhi_poll(struct mhi_controller *c);
int
lx_mhi_poll(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!p || !p->registered) {
		return -ENODEV;
	}
	return service(p);
}

static irqreturn_t
ev_handler(int irq, void *arg)
{
	struct mhi_event *e = arg;
	struct mhi_priv *p = e->p;

	if (!p->irqs_on) {
		return IRQ_NONE;
	}
	(void)ev_process(p, e);
	run_transitions(p);
	return IRQ_HANDLED;
}

/* BHI_INTVEC: execution-environment and state changes seen in registers */
static irqreturn_t
bhi_handler(int irq, void *arg)
{
	struct mhi_priv *p = arg;

	if (!p->irqs_on) {
		return IRQ_NONE;
	}
	watch(p);
	run_transitions(p);
	return IRQ_HANDLED;
}

/* ---- power up ---- */
static void
irqs_enable(struct mhi_priv *p, bool on)
{
	struct mhi_controller *c = p->c;
	u32 i;

	if (on == p->irqs_on) {
		return;
	}
	if (!on) {
		p->irqs_on = false;	/* handlers return at once; then they are held off */
	}
	if (on) {
		enable_irq(c->irq[0]);
		for (i = 0; i < p->n_ev; i++) {
			enable_irq(c->irq[p->ev[i].irq]);
		}
		p->irqs_on = true;
	} else {
		disable_irq(c->irq[0]);
		for (i = 0; i < p->n_ev; i++) {
			disable_irq(c->irq[p->ev[i].irq]);
		}
	}
}

int
mhi_prepare_for_power_up(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	u32 off;
	int r;

	if (!p || !p->registered) {
		return -EINVAL;
	}
	if (!(p->pm & PM_DISABLE)) {
		MHI_LOG("prepare for power up while powered");
		return -EBUSY;
	}
	if (p->prepared) {
		dev_ctxt_deinit(p);	/* again after a failed start: the same blocks are reused */
	}
	r = dev_ctxt_init(p);
	if (r) {
		return r;
	}
	if (rd(c, c->regs, R_BHIOFF, &off)) {
		MHI_LOG("cannot read the BHI offset");
		r = -EIO;
		goto fail;
	}
	if (off >= c->reg_len) {
		MHI_LOG("BHI offset %#x is out of range (BAR %#zx)", off, c->reg_len);
		r = -ERANGE;
		goto fail;
	}
	p->bhi = (u8 *)c->regs + off;
	if (c->fbc_download || c->rddm_size || c->seg_len) {
		if (rd(c, c->regs, R_BHIEOFF, &off)) {
			MHI_LOG("cannot read the BHIe offset");
			r = -EIO;
			goto fail;
		}
		if (off >= c->reg_len) {
			MHI_LOG("BHIe offset %#x is out of range (BAR %#zx)", off, c->reg_len);
			r = -ERANGE;
			goto fail;
		}
		p->bhie = (u8 *)c->regs + off;
	}
	if (c->rddm_size) {
		MHI_LOG("RDDM is not supported: no RDDM table is set up (rddm_image stays NULL)");
	}
	p->prepared = true;
	MHI_LOG("prepared: BHI %#x, %u channels, %u event rings", (u32)(p->bhi - (u8 *)c->regs), p->max_chan, p->n_ev);
	return 0;
fail:
	dev_ctxt_deinit(p);
	p->bhi = p->bhie = NULL;
	return r;
}

int
mhi_async_power_up(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	enum mhi_ee_type ee;
	enum mhi_state st;
	u32 i;
	int r;

	if (!p || !p->registered || !p->prepared) {
		return -EINVAL;
	}
	if (!c->read_reg || !c->write_reg) {
		return -EINVAL;
	}
	MHI_LOG("power up requested");
	L(p);
	if (!(p->pm & PM_DISABLE)) {
		U(p);
		return -EBUSY;
	}
	p->pm = PM_POR;
	p->ee = MHI_EE_MAX;
	p->pending = 0;
	p->tr_busy = false;
	p->mission_done = p->mission_seen = p->sbl_seen = p->m0_requested = false;
	p->rddm_told = p->syserr_told = false;
	p->builtin_err = 0;
	p->fail_err = 0;
	p->watch_n = 0;
	p->dev_state = MHI_STATE_RESET;
	for (i = 0; i < p->n_ev; i++) {
		p->ev[i].ring.rp = p->ev[i].ring.wp = 0;
	}
	U(p);
	wr(c, p->bhi, BHI_INTVEC, 0);
	ee = get_exec_env(p);
	L(p);
	p->ee = ee;
	U(p);
	if (!(ee == MHI_EE_PBL || ee == MHI_EE_PTHRU || ee == MHI_EE_EDL || ee == MHI_EE_AMSS)) {
		MHI_LOG("EE %s is not valid for power up", ee_name(ee));
		r = -EIO;
		goto fail;
	}
	st = mhi_get_mhi_state(c);
	MHI_LOG("attempting power up with EE %s, state %s", ee_name(ee), state_name(st));
	if (st == MHI_STATE_SYS_ERR) {
		set_mhi_state(p, MHI_STATE_RESET);
		r = poll_field(p, c->regs, R_MHICTRL, MHICTRL_RESET_MASK, 0, c->timeout_ms);
		if (r) {
			MHI_LOG("failed to reset MHI after a system error");
			goto fail;
		}
		wr(c, p->bhi, BHI_INTVEC, 0);
	}
	irqs_enable(p, true);
	L(p);
	p->pending |= (ee == MHI_EE_PBL || ee == MHI_EE_PTHRU || ee == MHI_EE_EDL) ? T_PBL : T_READY;
	U(p);
	MHI_LOG("power on setup done, %s", (ee == MHI_EE_PBL || ee == MHI_EE_PTHRU || ee == MHI_EE_EDL) ?
	    "waiting to load the firmware" : "device already in mission mode, waiting for READY");
	return 0;
fail:
	state_set(p, PM_DISABLE);
	return r;
}

static void power_down_inner(struct mhi_priv *p, bool graceful);

int
mhi_sync_power_up(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	u32 to;
	u64 end;
	int r;

	r = mhi_async_power_up(c);
	if (r) {
		return r;
	}
	to = p->ready_timeout_ms ? p->ready_timeout_ms : c->timeout_ms;
	end = lx_os_nanoseconds() + (u64)to * 1000000ull;
	for (;;) {
		service(p);
		if (p->mission_done || (p->pm & PM_ERRORS)) {
			break;
		}
		if (lx_os_nanoseconds() >= end) {
			break;
		}
		lx_pause_ms(1);
	}
	if (p->mission_done && !(p->pm & PM_ERRORS) && !p->builtin_err) {
		MHI_LOG("power up done: mission mode");
		return 0;
	}
	if (p->builtin_err) {
		r = p->builtin_err < 0 ? p->builtin_err : -EIO;
		MHI_LOG("power up failed: the IPCR channels did not start (%d)", r);
	} else if (p->pm & PM_ERRORS) {
		r = p->fail_err ? p->fail_err : -EIO;
		MHI_LOG("power up failed: %d (pm state %#x, EE %s)", r, p->pm, ee_name(p->ee));
	} else {
		r = -ETIMEDOUT;
		MHI_LOG("power up timed out after %u ms (EE %s, state %s)", to, ee_name(p->ee),
		    state_name(p->dev_state));
		dump(p, "power-up stall");
	}
	power_down_inner(p, true);
	return r;
}

/* ---- power down: Linux __mhi_power_down + mhi_pm_disable_transition ----
 * Reset is always tried (the C2 register accessors survive a dead link): the device must stop DMA before
 * the contexts and the pool go. graceful only decides whether the device's answer is waited for. */
static void
power_down_inner(struct mhi_priv *p, bool graceful)
{
	struct mhi_controller *c = p->c;
	u64 end;
	u32 i;
	u32 wait_ms = c->timeout_ms < 1000 ? c->timeout_ms : 1000;

	L(p);
	if (p->pm & PM_DISABLE) {
		U(p);
		return;
	}
	p->pm = graceful ? PM_SHUTDOWN : PM_LD_ERR_FATAL;
	p->ee = MHI_EE_DISABLE_TRANSITION;
	p->dev_state = MHI_STATE_RESET;
	p->pending = 0;
	U(p);
	wake_all(p);
	MHI_LOG("power down%s", graceful ? "" : " (not graceful)");
	/* let the servicing in other threads end */
	end = lx_os_nanoseconds() + (u64)c->timeout_ms * 1000000ull;
	for (;;) {
		bool busy = p->tr_busy || p->watch_busy;

		for (i = 0; i < p->n_ev; i++) {
			busy = busy || p->ev[i].busy;
		}
		if (!busy || lx_os_nanoseconds() >= end) {
			break;
		}
		lx_pause_ms(1);
	}
	/* stop the device: wake off, then reset */
	L(p);
	if (p->wake_db && p->wake_set) {
		write_db(c, p->wake_db, 0);
		p->wake_set = false;
	}
	p->dev_wake = 0;
	p->wake_hold = false;
	U(p);
	if (p->prepared) {
		set_mhi_state(p, MHI_STATE_RESET);
		if (graceful) {
			int r = poll_field_noabort(p, c->regs, R_MHICTRL, MHICTRL_RESET_MASK, 0, wait_ms);

			if (r) {
				MHI_LOG("the device did not clear MHI reset (%d)", r);
			}
		}
		if (p->bhi) {
			wr(c, p->bhi, BHI_INTVEC, 0);
		}
	}
	irqs_enable(p, false);
	destroy_devices(p);
	for (i = 0; i < p->max_chan; i++) {
		if (p->chan[i].configured) {
			chan_reset(p, &p->chan[i], false);
		}
	}
	L(p);
	p->cmd.rp = p->cmd.wp = 0;
	if (p->cmd_ctx) {
		ctx_set64(p->cmd_ctx[0].rp, p->cmd.iova);
		ctx_set64(p->cmd_ctx[0].wp, p->cmd.iova);
	}
	for (i = 0; i < p->n_ev; i++) {
		struct mhi_ring *r = &p->ev[i].ring;

		r->rp = r->wp = 0;
		if (p->er_ctx && r->el) {
			ctx_set64(p->er_ctx[i].rp, r->iova);
			ctx_set64(p->er_ctx[i].wp, r->iova);
		}
	}
	p->pm = PM_DISABLE;
	p->pending_pkts = 0;
	p->mission_done = false;
	U(p);
	wake_all(p);
	MHI_LOG("powered down");
}

void
mhi_power_down(struct mhi_controller *c, bool graceful)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (p && p->registered) {
		power_down_inner(p, graceful);
	}
}

void
mhi_power_down_keep_dev(struct mhi_controller *c, bool graceful)
{
	if (c) {
		MHI_LOG("power_down_keep_dev: suspend is not supported, so the devices go too");
	}
	mhi_power_down(c, graceful);
}

void
mhi_unprepare_after_power_down(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!p || !p->registered) {
		return;
	}
	if (!(p->pm & PM_DISABLE)) {
		MHI_LOG("unprepare while powered: powering down first");
		power_down_inner(p, false);
	}
	if (c->fbc_image) {
		image_free(p, c->fbc_image);
		c->fbc_image = NULL;
	}
	if (c->rddm_image) {
		image_free(p, c->rddm_image);
		c->rddm_image = NULL;
	}
	p->bhi = p->bhie = NULL;
	if (p->prepared) {
		dev_ctxt_deinit(p);
	}
	MHI_LOG("unprepared");
}

/* ---- device wake for the clients: Linux mhi_device_get_sync / mhi_device_put ---- */
int
mhi_device_get_sync(struct mhi_device *d)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	int r;

	if (!p || !p->registered) {
		return -EINVAL;
	}
	L(p);
	if ((p->pm & (PM_ERRORS | PM_DISABLE))) {
		U(p);
		return -EIO;
	}
	wake_get_locked(p);
	U(p);
	MHI_WAIT(p, p->pm == PM_M0, c->timeout_ms, r);
	if (r) {
		L(p);
		wake_put_locked(p);
		U(p);
		return -EIO;
	}
	d->dev_wake++;
	return 0;
}

void
mhi_device_put(struct mhi_device *d)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!p || !p->registered) {
		return;
	}
	if (d->dev_wake) {
		d->dev_wake--;
	}
	L(p);
	wake_put_locked(p);
	U(p);
}

/* ---- channels: Linux mhi_init_chan_ctxt, mhi_update_channel_state, mhi_reset_chan ---- */
static void
chan_wait_idle(struct mhi_chan *ch)
{
	u64 end = lx_os_nanoseconds() + 5000ull * 1000000ull;

	while (ch->cb_busy && lx_os_nanoseconds() < end) {
		lx_pause_ms(1);
	}
	if (ch->cb_busy) {
		MHI_LOG("channel %u: a callback is still running", ch->num);
	}
}

static int
chan_cmd(struct mhi_priv *p, struct mhi_chan *ch, u32 type)
{
	struct mhi_controller *c = p->c;
	struct mhi_el *el;
	int r;

	L(p);
	if (!(p->pm & PM_M0) || ring_avail(&p->cmd) == 0) {
		U(p);
		return (p->pm & PM_M0) ? -ENOMEM : -EIO;
	}
	ch->cmd_done = false;
	ch->ccs = 0;
	el = &p->cmd.el[p->cmd.wp];
	el->ptr = 0;
	el->dw0 = 0;
	el->dw1 = (ch->num << 24) | (type << 16);
	p->cmd.wp = (p->cmd.wp + 1) % p->cmd.n;
	cmd_db(p);
	U(p);
	MHI_WAIT(p, ch->cmd_done, c->timeout_ms, r);
	if (r == -ETIMEDOUT) {
		MHI_LOG("channel %u: no command completion after %u ms", ch->num, c->timeout_ms);
		dump(p, "channel command stall");
	}
	if (r == 0 && ch->ccs != CC_SUCCESS) {
		MHI_LOG("channel %u: command completion code %#x", ch->num, ch->ccs);
		r = -EIO;
	}
	return r;
}

static void
chan_free_rings(struct mhi_priv *p, struct mhi_chan *ch)
{
	ring_free(p, &ch->tre);
	blk_put(p, ch->bb);
	ch->bb = NULL;
	kfree(ch->buf);
	ch->buf = NULL;
	ch->slot = 0;
}

/* Complete what is still queued with -ENOTCONN (the contract of mhi.h), mark the channel disabled, ask
 * the device to reset it (if there is a device to ask), and give its rings back. */
static void
chan_reset(struct mhi_priv *p, struct mhi_chan *ch, bool command)
{
	struct mhi_result res;
	void (*cb)(struct mhi_device *, struct mhi_result *);
	struct mhi_device *dev;

	if (!ch->tre.el && ch->state == CH_DISABLED) {
		return;
	}
	L(p);
	ch->state = CH_DISABLED;
	U(p);
	if (command && (p->pm & PM_M0) && ch->tre.el) {
		int r = chan_cmd(p, ch, PKT_RESET_CHAN_CMD);

		if (r) {
			MHI_LOG("channel %u: reset command failed (%d)", ch->num, r);
		}
	}
	chan_wait_idle(ch);
	for (;;) {
		void *cbuf;

		L(p);
		if (!ch->tre.el || ch->tre.rp == ch->tre.wp) {
			U(p);
			break;
		}
		cbuf = ch->buf[ch->tre.rp].cb;
		memset(&ch->buf[ch->tre.rp], 0, sizeof(ch->buf[0]));
		ch->tre.rp = (ch->tre.rp + 1) % ch->tre.n;
		if (ch->dir == DMA_TO_DEVICE && p->pending_pkts) {
			p->pending_pkts--;
		}
		cb = ch->xfer_cb;
		dev = ch->dev;
		U(p);
		res.buf_addr = cbuf;
		res.bytes_xferd = 0;
		res.dir = ch->dir;
		res.transaction_status = -ENOTCONN;
		if (cb) {
			cb(dev, &res);
		}
	}
	L(p);
	if (p->chan_ctx) {
		p->chan_ctx[ch->num].w0 &= ~0xffu;
	}
	U(p);
	chan_free_rings(p, ch);
}

static int
chan_prepare(struct mhi_priv *p, struct mhi_chan *ch)
{
	struct mhi_controller *c = p->c;
	size_t slot;
	int r;

	if (!(ch->ee_mask & (1u << p->ee))) {
		MHI_LOG("channel %u: EE %s is not in the mask %#x", ch->num, ee_name(p->ee), ch->ee_mask);
		return -ENOTCONN;
	}
	if (ch->state == CH_ENABLED) {
		return 0;
	}
	slot = c->buffer_len > IPCR_PACKET_MAX ? c->buffer_len : IPCR_PACKET_MAX;
	slot = (slot + SLOT_ALIGN - 1) & ~(size_t)(SLOT_ALIGN - 1);
	r = ring_alloc(p, &ch->tre, ch->elements);
	if (r) {
		return -ENOMEM;
	}
	ch->buf = kzalloc(sizeof(*ch->buf) * ch->elements, GFP_KERNEL);
	ch->bb = blk_get(p, slot * ch->elements, &ch->bb_dma);
	if (!ch->buf || !ch->bb) {
		chan_free_rings(p, ch);
		return -ENOMEM;
	}
	ch->slot = slot;
	L(p);
	p->chan_ctx[ch->num].w0 = (p->chan_ctx[ch->num].w0 & ~0xffu) | CH_ENABLED;
	ctx_set64(p->chan_ctx[ch->num].rbase, ch->tre.iova);
	ctx_set64(p->chan_ctx[ch->num].rlen, (u64)ch->tre.n * sizeof(struct mhi_el));
	ctx_set64(p->chan_ctx[ch->num].rp, ch->tre.iova);
	ctx_set64(p->chan_ctx[ch->num].wp, ch->tre.iova);
	ch->db_mode = true;
	fence();
	U(p);
	r = chan_cmd(p, ch, PKT_START_CHAN_CMD);
	if (r) {
		L(p);
		p->chan_ctx[ch->num].w0 &= ~0xffu;
		U(p);
		chan_free_rings(p, ch);
		return r;
	}
	L(p);
	ch->state = CH_ENABLED;
	U(p);
	MHI_LOG("channel %u started", ch->num);
	return 0;
}

int
mhi_prepare_for_transfer(struct mhi_device *d)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	struct mhi_chan *ch[2];
	int i, r = 0;

	if (!p || !p->registered) {
		return -EINVAL;
	}
	ch[0] = d->ul_chan;
	ch[1] = d->dl_chan;
	/* the device must be awake and in M0 while the commands go */
	r = mhi_device_get_sync(p->c->mhi_dev);
	if (r) {
		return r;
	}
	for (i = 0; i < 2; i++) {
		if (!ch[i]) {
			continue;
		}
		r = chan_prepare(p, ch[i]);
		if (r) {
			break;
		}
	}
	if (r) {
		while (--i >= 0) {
			if (ch[i]) {
				chan_reset(p, ch[i], true);
			}
		}
	}
	mhi_device_put(p->c->mhi_dev);
	return r;
}

void
mhi_unprepare_from_transfer(struct mhi_device *d)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!p || !p->registered) {
		return;
	}
	if (d->dl_chan) {
		chan_reset(p, d->dl_chan, true);
	}
	if (d->ul_chan) {
		chan_reset(p, d->ul_chan, true);
	}
}

/* ---- transfers: Linux mhi_queue / mhi_gen_tre, with the bounce slot of the channel ---- */
static int
queue_xfer(struct mhi_device *d, enum dma_data_direction dir, void *cb, void *buf, size_t len,
    enum mhi_flags flags)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	struct mhi_chan *ch;
	struct mhi_el *tre;
	u32 idx;

	if (!p || !p->registered) {
		return -EINVAL;
	}
	ch = (dir == DMA_TO_DEVICE) ? d->ul_chan : d->dl_chan;
	if (!ch || (dir != DMA_TO_DEVICE && dir != DMA_FROM_DEVICE) || !buf) {
		return -EINVAL;
	}
	if (flags & MHI_CHAIN) {
		return -EOPNOTSUPP;
	}
	L(p);
	if (p->pm & (PM_ERRORS | PM_DISABLE)) {
		U(p);
		return -EIO;
	}
	if (ch->state != CH_ENABLED) {
		U(p);
		return -ENODEV;
	}
	if (len == 0 || len > ch->slot || len > 0xffff) {
		U(p);
		return -EINVAL;
	}
	if (ring_avail(&ch->tre) == 0) {
		U(p);
		return -EAGAIN;
	}
	idx = ch->tre.wp;
	if (dir == DMA_TO_DEVICE) {
		memcpy(ch->bb + (size_t)idx * ch->slot, buf, len);	/* the caller's buffer is its own again at once */
	}
	ch->buf[idx].cb = cb;
	ch->buf[idx].dst = (dir == DMA_FROM_DEVICE) ? buf : NULL;
	ch->buf[idx].len = (u32)len;
	tre = &ch->tre.el[idx];
	tre->ptr = ch->bb_dma + (u64)idx * ch->slot;
	tre->dw0 = (u32)len & 0xffff;
	tre->dw1 = (PKT_TRANSFER << 16) | ((flags & MHI_EOT) ? (1u << 9) : 0) | ((flags & MHI_EOB) ? (1u << 8) : 0);
	fence();
	ch->tre.wp = (ch->tre.wp + 1) % ch->tre.n;
	if (dir == DMA_TO_DEVICE) {
		p->pending_pkts++;
	}
	if (p->pm & PM_M0) {
		chan_db(p, ch);
	}
	U(p);
	return 0;
}

int
mhi_queue_buf(struct mhi_device *d, enum dma_data_direction dir, void *buf, size_t len, enum mhi_flags flags)
{
	return queue_xfer(d, dir, buf, buf, len, flags);
}

int
mhi_queue_skb(struct mhi_device *d, enum dma_data_direction dir, struct sk_buff *skb, size_t len,
    enum mhi_flags flags)
{
	if (!skb || len > skb->len) {
		return -EINVAL;
	}
	return queue_xfer(d, dir, skb, skb->data, len, flags);
}

int
mhi_get_free_desc_count(struct mhi_device *d, enum dma_data_direction dir)
{
	struct mhi_controller *c = d ? d->mhi_cntrl : NULL;
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	struct mhi_chan *ch;
	int n;

	if (!p || !p->registered) {
		return -EINVAL;
	}
	ch = (dir == DMA_TO_DEVICE) ? d->ul_chan : d->dl_chan;
	if (!ch) {
		return -EINVAL;
	}
	L(p);
	n = (ch->state == CH_ENABLED) ? (int)ring_avail(&ch->tre) : 0;
	U(p);
	return n;
}

/* ---- the bus: drivers ---- */
int
__mhi_driver_register(struct mhi_driver *drv, struct module *owner)
{
	struct mhi_priv *p = active_ctrl ? PRIV(active_ctrl) : NULL;
	u32 i, k;

	if (!drv || !drv->id_table || !drv->probe) {
		return -EINVAL;
	}
	for (i = 0; i < MAX_DRV; i++) {
		if (drivers[i] == drv) {
			return -EEXIST;
		}
	}
	for (i = 0; i < MAX_DRV && drivers[i]; i++) {
	}
	if (i == MAX_DRV) {
		return -ENOSPC;
	}
	drivers[i] = drv;
	/* devices that exist already (power-up came first) meet it now */
	if (p) {
		for (k = 0; k < MAX_DEV; k++) {
			if (p->dev_slot[k].dev && !p->dev_slot[k].bound) {
				dev_bind(p, &p->dev_slot[k]);
			}
		}
	}
	return 0;
}

void
mhi_driver_unregister(struct mhi_driver *drv)
{
	struct mhi_priv *p = active_ctrl ? PRIV(active_ctrl) : NULL;
	u32 i, k;

	for (i = 0; i < MAX_DRV; i++) {
		if (drivers[i] != drv) {
			continue;
		}
		if (p) {
			for (k = 0; k < MAX_DEV; k++) {
				struct mhi_dev_slot *s = &p->dev_slot[k];

				if (s->dev && s->bound && s->drv == drv) {
					dev_unbind(p, s);
				}
			}
		}
		drivers[i] = NULL;
	}
}

/* ---- the IPCR client: Linux net/qrtr/mhi.c, as a C6 link given to the kernel through C4 ----
 * Channels 20 (UL) and 21 (DL). RX buffers are queued on 21 before the link is published (the radio may
 * answer the first HELLO at once); a packet is delivered whole with lx_os_qrtr_receive() and its buffer
 * queued again. TX copies into the channel's slot when queued, so the packet may go out of scope at once. */
static struct {
	struct hakios_qrtr_link link;
	struct mhi_device *dev;
	u8 **rx;
	u32 rx_count;
	size_t rx_len;
	bool up;
} ipcr;

static int
ipcr_send(void *ctx, const void *packet, unsigned long len)
{
	int r;

	if (!ipcr.up || !ipcr.dev) {
		return ENOTCONN;
	}
	if (!packet || len == 0 || len > IPCR_PACKET_MAX) {
		return EINVAL;
	}
	r = queue_xfer(ipcr.dev, DMA_TO_DEVICE, (void *)packet, (void *)packet, len, MHI_EOT);
	return r < 0 ? -r : r;
}

static void
ipcr_ul_cb(struct mhi_device *d, struct mhi_result *res)
{
	/* nothing to give back: the packet was copied when it was queued */
}

static void
ipcr_dl_cb(struct mhi_device *d, struct mhi_result *res)
{
	int r;

	if (res->transaction_status == -ENOTCONN) {
		return;	/* channel reset: the buffer stays in ipcr.rx */
	}
	if (res->transaction_status == 0) {
		r = lx_os_qrtr_receive(&ipcr.link, res->buf_addr, res->bytes_xferd);
		if (r == EINVAL) {
			MHI_LOG("IPCR: invalid router packet (%zu bytes)", res->bytes_xferd);
		} else if (r && r != ENOTCONN) {
			MHI_LOG("IPCR: packet refused (%d)", r);
		}
	} else {
		MHI_LOG("IPCR: receive status %d (%zu bytes dropped)", res->transaction_status, res->bytes_xferd);
	}
	r = mhi_queue_buf(d, DMA_FROM_DEVICE, res->buf_addr, ipcr.rx_len, MHI_EOT);
	if (r) {
		MHI_LOG("IPCR: could not queue the buffer again (%d)", r);
	}
}

static void
ipcr_free_rx(void)
{
	u32 i;

	if (ipcr.rx) {
		for (i = 0; i < ipcr.rx_count; i++) {
			kfree(ipcr.rx[i]);
		}
		kfree(ipcr.rx);
	}
	ipcr.rx = NULL;
	ipcr.rx_count = 0;
}

static int
ipcr_probe(struct mhi_device *d, const struct mhi_device_id *id)
{
	int r, n, i;

	if (ipcr.dev) {
		return -EBUSY;
	}
	if (!d->ul_chan || !d->dl_chan) {
		MHI_LOG("IPCR: needs both channels");
		return -EINVAL;
	}
	r = mhi_prepare_for_transfer(d);
	if (r) {
		return r;
	}
	n = mhi_get_free_desc_count(d, DMA_FROM_DEVICE);
	ipcr.rx_len = d->mhi_cntrl->buffer_len;
	ipcr.rx = n > 0 ? kzalloc(sizeof(*ipcr.rx) * n, GFP_KERNEL) : NULL;
	if (n <= 0 || !ipcr.rx) {
		r = -ENOMEM;
		goto err_unprepare;
	}
	ipcr.rx_count = 0;
	for (i = 0; i < n; i++) {
		ipcr.rx[i] = kzalloc(ipcr.rx_len, GFP_KERNEL);
		if (!ipcr.rx[i]) {
			r = -ENOMEM;
			ipcr.rx_count = (u32)i;
			goto err_unprepare;
		}
		ipcr.rx_count = (u32)(i + 1);
		r = mhi_queue_buf(d, DMA_FROM_DEVICE, ipcr.rx[i], ipcr.rx_len, MHI_EOT);
		if (r) {
			MHI_LOG("IPCR: could not queue a receive buffer (%d)", r);
			goto err_unprepare;
		}
	}
	ipcr.dev = d;
	ipcr.link.send = ipcr_send;
	ipcr.link.ctx = &ipcr;
	ipcr.link.name = "mhi-ipcr";
	ipcr.up = true;
	r = lx_os_qrtr_link(&ipcr.link);
	if (r) {
		MHI_LOG("IPCR: the router refused the link (%d)", r);
		ipcr.up = false;
		ipcr.dev = NULL;
		r = -r;
		goto err_unprepare;
	}
	MHI_LOG("IPCR: link up with %d receive buffers of %zu bytes", n, ipcr.rx_len);
	return 0;
err_unprepare:
	mhi_unprepare_from_transfer(d);
	ipcr_free_rx();
	return r;
}

static void
ipcr_remove(struct mhi_device *d)
{
	if (ipcr.dev != d) {
		return;
	}
	ipcr.up = false;
	lx_os_qrtr_link(NULL);	/* returns when no callback is running */
	mhi_unprepare_from_transfer(d);
	ipcr.dev = NULL;
	ipcr_free_rx();
	MHI_LOG("IPCR: link down");
}

static const struct mhi_device_id ipcr_ids[] = { { .chan = "IPCR" }, { { 0 } } };
static struct mhi_driver ipcr_driver = {
	.id_table = ipcr_ids,
	.probe = ipcr_probe,
	.remove = ipcr_remove,
	.ul_xfer_cb = ipcr_ul_cb,
	.dl_xfer_cb = ipcr_dl_cb,
};

/* ---- the controller: Linux mhi_alloc_controller / mhi_register_controller ---- */
struct mhi_controller *
mhi_alloc_controller(void)
{
	struct mhi_controller *c = kzalloc(sizeof(*c), GFP_KERNEL);
	struct mhi_priv *p;

	if (!c) {
		return NULL;
	}
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		kfree(c);
		return NULL;
	}
	p->c = c;
	p->pm = PM_DISABLE;
	p->ee = MHI_EE_MAX;
	c->private_data = p;
	return c;
}

static void
unreg_irqs(struct mhi_priv *p)
{
	struct mhi_controller *c = p->c;
	u32 i;

	for (i = 0; i < p->n_ev; i++) {
		if (p->ev[i].requested) {
			free_irq(c->irq[p->ev[i].irq], &p->ev[i]);
			p->ev[i].requested = false;
		}
	}
	if (p->irq0) {
		free_irq(c->irq[0], p);
		p->irq0 = false;
	}
}

int
mhi_register_controller(struct mhi_controller *c, const struct mhi_controller_config *cfg)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;
	unsigned long flags;
	u32 i;
	int r;

	if (!p || !cfg) {
		return -EINVAL;
	}
	if (p->registered) {
		return -EBUSY;
	}
	if (!c->cntrl_dev || !c->regs || !c->reg_len || !c->runtime_get || !c->runtime_put || !c->status_cb ||
	    !c->read_reg || !c->write_reg || !c->irq || c->nr_irqs < 1) {
		MHI_LOG("register: the controller lacks a device, registers, callbacks or interrupts");
		return -EINVAL;
	}
	if (!cfg->ch_cfg || !cfg->event_cfg || cfg->max_channels < 1 || cfg->max_channels > 256 ||
	    cfg->num_events < 1 || cfg->num_events > MAX_EV || cfg->num_channels > cfg->max_channels) {
		MHI_LOG("register: bad channel/event configuration");
		return -EINVAL;
	}
	if (active_ctrl && active_ctrl != c) {
		MHI_LOG("register: one controller at a time");
		return -EBUSY;
	}
	if (c->iova_stop < c->iova_start) {
		MHI_LOG("register: iova_stop below iova_start");
		return -EINVAL;
	}
	/* C3's pool, cut by what the caller allows (ath11k gives the whole DMA mask, inclusive) */
	p->win_lo = c->iova_start > HAKIOS_WIFI_IOVA_BASE ? c->iova_start : HAKIOS_WIFI_IOVA_BASE;
	p->win_hi = c->iova_stop < HAKIOS_WIFI_IOVA_BASE + HAKIOS_WIFI_POOL_SIZE - 1 ?
	    c->iova_stop : HAKIOS_WIFI_IOVA_BASE + HAKIOS_WIFI_POOL_SIZE - 1;
	if (p->win_lo > p->win_hi) {
		MHI_LOG("register: the allowed IOVA range %#llx-%#llx does not meet C3's pool", (unsigned long long)c->iova_start,
		    (unsigned long long)c->iova_stop);
		return -EINVAL;
	}
	p->max_chan = cfg->max_channels;
	p->n_ev = cfg->num_events;
	p->chan = kzalloc(sizeof(*p->chan) * p->max_chan, GFP_KERNEL);
	if (!p->chan) {
		return -ENOMEM;
	}
	for (i = 0; i < cfg->num_channels; i++) {
		const struct mhi_channel_config *cc = &cfg->ch_cfg[i];
		struct mhi_chan *ch;

		if (cc->num >= p->max_chan) {
			MHI_LOG("register: channel %u is beyond max_channels", cc->num);
			r = -EINVAL;
			goto fail;
		}
		if (cc->offload_channel || cc->type == MHI_CH_TYPE_INBOUND_COALESCED) {
			MHI_LOG("register: channel %u: offload and coalesced channels are not supported", cc->num);
			r = -EOPNOTSUPP;
			goto fail;
		}
		if ((cc->dir != DMA_TO_DEVICE && cc->dir != DMA_FROM_DEVICE) || cc->num_elements < 2 ||
		    cc->event_ring >= cfg->num_events || (cc->doorbell != MHI_DB_BRST_DISABLE &&
		    cc->doorbell != MHI_DB_BRST_ENABLE)) {
			MHI_LOG("register: channel %u has a bad configuration", cc->num);
			r = -EINVAL;
			goto fail;
		}
		ch = &p->chan[cc->num];
		if (ch->configured) {
			MHI_LOG("register: channel %u configured twice", cc->num);
			r = -EINVAL;
			goto fail;
		}
		ch->name = cc->name;
		ch->num = cc->num;
		ch->elements = cc->num_elements;
		ch->er = cc->event_ring;
		ch->dir = cc->dir;
		ch->type = cc->type ? cc->type : (enum mhi_ch_type)cc->dir;
		ch->ee_mask = cc->ee_mask;
		ch->pollcfg = cc->pollcfg;
		ch->brst = cc->doorbell;
		ch->configured = true;
	}
	for (i = 0; i < cfg->num_events; i++) {
		const struct mhi_event_config *ec = &cfg->event_cfg[i];
		struct mhi_event *e = &p->ev[i];

		if (ec->hardware_event || ec->client_managed || ec->offload_channel) {
			MHI_LOG("register: event ring %u: hardware, client-managed and offload rings are not supported", i);
			r = -EOPNOTSUPP;
			goto fail;
		}
		if (ec->num_elements < 2 || ec->irq >= c->nr_irqs ||
		    (ec->mode != MHI_DB_BRST_DISABLE && ec->mode != MHI_DB_BRST_ENABLE) ||
		    (ec->data_type != MHI_ER_DATA && ec->data_type != MHI_ER_CTRL)) {
			MHI_LOG("register: event ring %u has a bad configuration", i);
			r = -EINVAL;
			goto fail;
		}
		e->index = i;
		e->p = p;
		e->intmod = ec->irq_moderation_ms;
		e->irq = ec->irq;
		e->type = ec->data_type;
		e->brst = ec->mode;
		e->elements = ec->num_elements;
	}
	c->timeout_ms = cfg->timeout_ms ? cfg->timeout_ms : MHI_TIMEOUT_DEFAULT_MS;
	p->ready_timeout_ms = cfg->ready_timeout_ms;
	c->buffer_len = cfg->buf_len ? cfg->buf_len : MHI_MAX_MTU;
	/* use_bounce_buf is ignored: buffers are copied in any case (the radio reaches only the pool) */
	/* Linux requests the interrupts here and holds them off until power-up; the handlers run on the
	 * polling thread of the PCI layer */
	flags = c->irq_flags ? c->irq_flags : IRQF_SHARED;
	r = request_irq(c->irq[0], bhi_handler, flags, "bhi", p);
	if (r) {
		MHI_LOG("register: interrupt %d for BHI: error %d", c->irq[0], r);
		goto fail;
	}
	p->irq0 = true;
	disable_irq(c->irq[0]);
	for (i = 0; i < p->n_ev; i++) {
		r = request_irq(c->irq[p->ev[i].irq], ev_handler, flags, "mhi", &p->ev[i]);
		if (r) {
			MHI_LOG("register: interrupt %d for event ring %u: error %d", c->irq[p->ev[i].irq], i, r);
			goto fail;
		}
		p->ev[i].requested = true;
		disable_irq(c->irq[p->ev[i].irq]);
	}
	c->mhi_dev = dev_alloc(c, "mhi-controller");
	if (!c->mhi_dev) {
		r = -ENOMEM;
		goto fail;
	}
	c->mhi_dev->dev_type = MHI_DEVICE_CONTROLLER;
	p->pm = PM_DISABLE;
	p->ee = MHI_EE_MAX;
	p->registered = true;
	active_ctrl = c;
	r = __mhi_driver_register(&ipcr_driver, NULL);
	if (r) {
		MHI_LOG("register: IPCR client: error %d", r);
		p->registered = false;
		active_ctrl = NULL;
		kfree(c->mhi_dev);
		c->mhi_dev = NULL;
		goto fail;
	}
	MHI_LOG("registered: %u channels, %u event rings, DMA window %#llx-%#llx, timeout %u ms", cfg->num_channels,
	    cfg->num_events, (unsigned long long)p->win_lo, (unsigned long long)p->win_hi, c->timeout_ms);
	return 0;
fail:
	unreg_irqs(p);
	kfree(p->chan);
	p->chan = NULL;
	memset(p->ev, 0, sizeof(p->ev));
	return r < 0 ? r : -r;
}

void
mhi_unregister_controller(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!p || !p->registered) {
		return;
	}
	power_down_inner(p, false);
	if (p->prepared) {
		dev_ctxt_deinit(p);
	}
	mhi_driver_unregister(&ipcr_driver);
	unreg_irqs(p);
	p->registered = false;
	if (active_ctrl == c) {
		active_ctrl = NULL;
	}
	kfree(p->chan);
	p->chan = NULL;
	memset(p->ev, 0, sizeof(p->ev));
	kfree(c->mhi_dev);
	c->mhi_dev = NULL;
	MHI_LOG("unregistered");
}

void
mhi_free_controller(struct mhi_controller *c)
{
	struct mhi_priv *p = c ? PRIV(c) : NULL;

	if (!c) {
		return;
	}
	if (p) {
		if (p->registered) {
			mhi_unregister_controller(c);
		}
		if (c->fbc_image) {
			image_free(p, c->fbc_image);
			c->fbc_image = NULL;
		}
		blk_free_all(p);
		kfree(p);
		c->private_data = NULL;
	}
	kfree(c);
}

/* ---- what is not here, said when asked ---- */
int
mhi_pm_suspend(struct mhi_controller *c)
{
	MHI_LOG("suspend is not supported");
	return -EOPNOTSUPP;
}

int
mhi_pm_resume_force(struct mhi_controller *c)
{
	MHI_LOG("resume is not supported (nothing was suspended)");
	return -EOPNOTSUPP;
}

int
mhi_download_rddm_image(struct mhi_controller *c, bool in_panic)
{
	MHI_LOG("RDDM download is not supported");
	return -EOPNOTSUPP;
}

#endif /* CONFIG_ATH11K_PCI: the AHB-only object has none of this */
