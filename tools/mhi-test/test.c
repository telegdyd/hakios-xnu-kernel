/*
 * Host test of wifi/lx/src/lx_mhi.c: the production file is compiled into this one, with only the
 * kernel (C4), the Linux headers and the chip replaced. The chip is a scripted fake behind the
 * controller's read_reg/write_reg: PBL (BHI), SBL (READY, M0, BHIe), AMSS (channel commands, IPCR),
 * reading the host's contexts and rings out of the same memory the host wrote.
 */
#include "fixture.h"
#include "hakios.smmu.h"
#include "../../wifi/lx/src/lx_mhi.c"

static int checks, fails;
#define CHECK(cond) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define MUST(cond) do { checks++; if (!(cond)) { printf("FATAL %s:%d: %s\n", __FILE__, __LINE__, #cond); exit(1); } } while (0)

_Static_assert(MHI_POOL_BASE_FALLBACK == HAKIOS_WIFI_IOVA_BASE, "fallback pool base drifted from hakios.smmu.h");
_Static_assert(MHI_POOL_SIZE_FALLBACK == HAKIOS_WIFI_POOL_SIZE, "fallback pool size drifted from hakios.smmu.h");

/* ---------------------------------------------------------------- environment */
#define POOL_BYTES (24u << 20)
static uint8_t pool[POOL_BYTES] __attribute__((aligned(4096)));
static u32 pool_used;
static unsigned dma_calls, dma_fail_at, dma_frees;
static u64 pool_base = HAKIOS_WIFI_IOVA_BASE;
static u64 window_base = HAKIOS_WIFI_IOVA_BASE;	/* what lx_base.c says the pool's radio address is */
static bool window_none;

int
lx_pool_window(unsigned long long *base, unsigned long *size)
{
	if (window_none) {
		return ENODEV;
	}
	*base = window_base;
	*size = HAKIOS_WIFI_POOL_SIZE;
	return 0;
}

void *
test_dma_alloc(struct device *dev, size_t size, dma_addr_t *dma)
{
	dma_calls++;
	if (dma_fail_at && dma_calls == dma_fail_at) {
		return NULL;
	}
	size = (size + 4095) & ~(size_t)4095;
	if (pool_used + size > POOL_BYTES) {
		return NULL;
	}
	memset(pool + pool_used, 0, size);
	*dma = pool_base + pool_used;
	pool_used += (u32)size;
	return pool + pool_used - size;
}
void
test_dma_free(struct device *dev, size_t size, void *va, dma_addr_t dma)
{
	dma_frees++;	/* (the real pool gives nothing back either) */
}
static uint8_t *
iova2va(u64 iova)
{
	if (iova < pool_base || iova >= pool_base + pool_used) {
		return NULL;
	}
	return pool + (iova - pool_base);
}

static u64 now_ns;
static unsigned option_dump;
static char (*lines)[256];
static unsigned nlines;
#define MAXLINES 8192
void
lx_tell(const char *fmt, ...)
{
	va_list ap;

	if (!lines) {
		lines = calloc(MAXLINES, 256);
	}
	if (nlines < MAXLINES) {
		va_start(ap, fmt);
		vsnprintf(lines[nlines], 256, fmt, ap);
		va_end(ap);
		nlines++;
	}
}
/* C1 stage lines said by the production file (WP6: stages 5 and 6): the order, the verdict and the first detail words */
static struct {
	unsigned n[32], count;
	int ok[32];
	char detail[32][96];
} said;
void
lx_os_stage(unsigned n, const char *name, int ok, const char *fmt, ...)
{
	va_list ap;

	if (said.count < 32) {
		said.n[said.count] = n;
		said.ok[said.count] = ok;
		va_start(ap, fmt);
		vsnprintf(said.detail[said.count], sizeof(said.detail[0]), fmt, ap);
		va_end(ap);
		said.count++;
	}
}
static unsigned
lines_with(const char *needle, unsigned from)
{
	unsigned i, n = 0;

	for (i = from; i < nlines; i++) {
		if (strstr(lines[i], needle)) {
			n++;
		}
	}
	return n;
}
static int
line_index(const char *needle, unsigned from)
{
	unsigned i;

	for (i = from; i < nlines; i++) {
		if (strstr(lines[i], needle)) {
			return (int)i;
		}
	}
	return -1;
}

void lx_os_lock(void) {}
void lx_os_unlock(void) {}
void lx_os_wakeup(const void *a) {}
int
lx_os_sleep(const void *a, unsigned long long ns)
{
	printf("FATAL: a lock was waited for in the single-threaded test\n");
	abort();
}
unsigned long long
lx_os_nanoseconds(void)
{
	now_ns += 1000;
	return now_ns;
}
static u32 rng = 0x1234567;
void
lx_os_random(void *buf, unsigned long n)
{
	u8 *b = buf;

	while (n--) {
		rng ^= rng << 13;
		rng ^= rng >> 17;
		rng ^= rng << 5;
		*b++ = (u8)rng;
	}
}
unsigned
lx_os_option(const char *name)
{
	return !strcmp(name, "wifi_mhi_dump") ? option_dump : 0;
}
static const u8 *fw_file;
static size_t fw_file_len;
int
lx_os_firmware(const char *name, const void **data, unsigned long *size)
{
	if (!strcmp(name, "qcom/amss.bin") && fw_file) {
		*data = fw_file;
		*size = fw_file_len;
		return 0;
	}
	return ENOENT;
}

/* C4's QRTR link */
static const struct hakios_qrtr_link *qlink;
static unsigned qlink_ups, qlink_downs, qlink_fail;
static unsigned rx_packets, rx_bytes;
static u8 rx_last[9000];
int
lx_os_qrtr_link(const struct hakios_qrtr_link *l)
{
	if (!l) {
		qlink = NULL;
		qlink_downs++;
		return 0;
	}
	if (qlink_fail) {
		return qlink_fail;
	}
	qlink = l;
	qlink_ups++;
	return 0;
}
int
lx_os_qrtr_receive(const struct hakios_qrtr_link *l, const void *packet, unsigned long len)
{
	if (l != qlink || !qlink) {
		return ENOTCONN;
	}
	if (len < 32 || len > 32 + 8192 || (len & 3)) {
		return EINVAL;
	}
	memcpy(rx_last, packet, len);
	rx_packets++;
	rx_bytes += (unsigned)len;
	return 0;
}

/* interrupts: what lx_pci.c's polling thread would call */
static struct {
	irq_handler_t h;
	void *arg;
	int depth;
	bool used;
} irqt[8];
static unsigned irq_enables, irq_disables, irq_requests;
int
request_irq(unsigned irq, irq_handler_t h, unsigned long flags, const char *name, void *arg)
{
	assert(irq >= 512 && irq < 520);
	if (irqt[irq - 512].used) {
		return -EBUSY;
	}
	irqt[irq - 512].used = true;
	irqt[irq - 512].h = h;
	irqt[irq - 512].arg = arg;
	irqt[irq - 512].depth = 0;
	irq_requests++;
	return 0;
}
void
free_irq(unsigned irq, void *arg)
{
	assert(irqt[irq - 512].used && irqt[irq - 512].arg == arg);
	memset(&irqt[irq - 512], 0, sizeof(irqt[0]));
}
void
enable_irq(unsigned irq)
{
	irq_enables++;
	if (irqt[irq - 512].depth) {
		irqt[irq - 512].depth--;
	}
}
void
disable_irq(unsigned irq)
{
	irq_disables++;
	irqt[irq - 512].depth++;
}
static unsigned handler_calls;
static void
run_handlers(void)
{
	unsigned i;

	for (i = 0; i < 8; i++) {
		if (irqt[i].used && irqt[i].depth == 0) {
			handler_calls++;
			irqt[i].h(512 + i, irqt[i].arg);
		}
	}
}
static unsigned irqs_active(void)
{
	unsigned i, n = 0;

	for (i = 0; i < 8; i++) {
		n += irqt[i].used && irqt[i].depth == 0;
	}
	return n;
}

/* ---------------------------------------------------------------- the fake chip */
#define BAR_BYTES 0x10000u
#define BHI 0x200u
#define BHIE 0x300u
#define CHDB 0x1000u
#define ERDB 0x1800u
#define WAKEDB (CHDB + 8 * 127)

enum stage { S_PBL, S_SBL_BOOT, S_READY, S_M0, S_BHIE_DONE, S_AMSS };
enum stall { STALL_NONE, STALL_BHI, STALL_READY, STALL_M0, STALL_BHIE, STALL_AMSS };

struct wl { u32 off, val; };
static struct {
	u8 bar[BAR_BYTES];
	enum stage stage;
	int ee;
	u32 ready_in, amss_in;
	/* knobs */
	enum stall stall;
	bool bhi_error, bhie_error, syserr_at_m0, fail_start_chan, reads_dead;
	/* what the host programmed */
	bool pending_bhi, pending_bhie, pending_cmd;
	u32 pending_ul;
	u64 cmd_db;
	u32 cmd_rp, chan_rp[128], chan_wp[128];
	bool chan_on[128];
	u32 ev_idx[8];
	/* what was seen */
	struct wl wl[4096];
	unsigned nwl;
	unsigned resets, faults, ev_overrun, bhi_ok, bhie_ok, bhie_entries;
	size_t bhie_total;
	unsigned er_db_writes[8], wake_last, wake_writes, chdb_writes[128];
	u8 ul[256][9000];
	unsigned ul_len[256], ul_n;
	unsigned start_cmds[128], reset_cmds[128];
	const u8 *expect_sbl, *expect_full;
	size_t expect_sbl_len, expect_full_len;
	unsigned start_order[8], nstart;
	unsigned reset_order[8], nreset;
} dv;

static inline u32 *
breg(u32 off)
{
	return (u32 *)(dv.bar + off);
}
static u64
reg64(u32 lo, u32 hi)
{
	return (u64)*breg(lo) | ((u64)*breg(hi) << 32);
}
static struct mhi_ctx *
ctx_at(u64 base_iova, u32 idx)
{
	return (struct mhi_ctx *)iova2va(base_iova + (u64)idx * sizeof(struct mhi_ctx));
}

static void
ev_push(u32 ring, u64 ptr, u32 dw0, u32 dw1)
{
	struct mhi_ctx *c = ctx_at(reg64(R_ECABAP_LOWER, R_ECABAP_HIGHER), ring);
	u64 rbase = ctx_get64(c->rbase), len = ctx_get64(c->rlen);
	u32 n = (u32)(len / 16), wp_idx = (u32)((ctx_get64(c->wp) - rbase) / 16);
	struct mhi_el *el = (struct mhi_el *)iova2va(rbase);

	MUST(el != NULL);
	if (dv.ev_idx[ring] == wp_idx) {
		dv.ev_overrun++;	/* the host gave us no free element */
		return;
	}
	el[dv.ev_idx[ring]].ptr = ptr;
	el[dv.ev_idx[ring]].dw0 = dw0;
	el[dv.ev_idx[ring]].dw1 = dw1;
	dv.ev_idx[ring] = (dv.ev_idx[ring] + 1) % n;
	ctx_set64(c->rp, rbase + (u64)dv.ev_idx[ring] * 16);
}

static void
dev_reset(void)
{
	dv.resets++;
	dv.stage = S_PBL;
	dv.ee = MHI_EE_PBL;
	*breg(R_MHICTRL) = 0;
	*breg(R_MHISTATUS) = 0;
	*breg(BHI + BHI_STATUS) = 0;
	*breg(BHIE + BHIE_TXVECSTATUS_OFFS) = 0;
	dv.pending_bhi = dv.pending_bhie = dv.pending_cmd = false;
	dv.pending_ul = 0;
	dv.cmd_rp = 0;
	memset(dv.chan_rp, 0, sizeof(dv.chan_rp));
	memset(dv.chan_wp, 0, sizeof(dv.chan_wp));
	memset(dv.chan_on, 0, sizeof(dv.chan_on));
	memset(dv.ev_idx, 0, sizeof(dv.ev_idx));
	*breg(BHI + BHI_EXECENV) = MHI_EE_PBL;
}

static void
dev_write(u32 off, u32 val)
{
	if (dv.nwl < 4096) {
		dv.wl[dv.nwl].off = off;
		dv.wl[dv.nwl].val = val;
		dv.nwl++;
	}
	*breg(off) = val;
	if (off == R_MHICTRL) {
		if (val & MHICTRL_RESET_MASK) {
			dev_reset();
		} else if (((val & MHICTRL_MHISTATE_MASK) >> 8) == MHI_STATE_M0 && dv.stage == S_READY &&
		    dv.stall != STALL_M0) {
			dv.stage = S_M0;
			if (dv.syserr_at_m0) {
				*breg(R_MHISTATUS) = ((u32)MHI_STATE_SYS_ERR << 8) | MHISTATUS_SYSERR_MASK;
				ev_push(0, 0, (u32)MHI_STATE_SYS_ERR << 24, PKT_STATE_CHANGE << 16);
				return;
			}
			*breg(R_MHISTATUS) = (MHI_STATE_M0 << 8) | 1;
			ev_push(0, 0, MHI_STATE_M0 << 24, PKT_STATE_CHANGE << 16);
			ev_push(0, 0, MHI_EE_SBL << 24, PKT_EE_EVENT << 16);
		}
	} else if (off == BHI + BHI_IMGTXDB) {
		dv.pending_bhi = true;
	} else if (off == BHIE + BHIE_TXVECDB_OFFS) {
		dv.pending_bhie = true;
	} else if (off == R_CRDB_LOWER) {
		dv.cmd_db = reg64(R_CRDB_LOWER, R_CRDB_LOWER + 4);
		dv.pending_cmd = true;
	} else if (off >= CHDB && off < CHDB + 8 * 128 && (off & 7) == 0) {
		u32 ch = (off - CHDB) / 8;

		dv.chdb_writes[ch]++;
		if (ch < 127) {
			struct mhi_ctx *c = ctx_at(reg64(R_CCABAP_LOWER, R_CCABAP_HIGHER), ch);
			u64 wp = (u64)val | ((u64)*breg(off + 4) << 32);

			dv.chan_wp[ch] = (u32)((wp - ctx_get64(c->rbase)) / 16);
			if (ch == 20) {
				dv.pending_ul++;
			}
		} else {
			dv.wake_last = val;
			dv.wake_writes++;
		}
	} else if (off >= ERDB && off < ERDB + 8 * 8 && (off & 7) == 0) {
		dv.er_db_writes[(off - ERDB) / 8]++;
	}
}

static void
dev_tick(unsigned ms)
{
	if (dv.pending_bhi) {
		u64 addr = reg64(BHI + BHI_IMGADDR_LOW, BHI + BHI_IMGADDR_HIGH);
		u32 size = *breg(BHI + BHI_IMGSIZE);
		uint8_t *va = iova2va(addr);

		dv.pending_bhi = false;
		if (dv.stall == STALL_BHI) {
			return;
		}
		if (!va || dv.bhi_error || size != dv.expect_sbl_len || memcmp(va, dv.expect_sbl, size)) {
			if (!va) {
				dv.faults++;
			}
			*breg(BHI + BHI_STATUS) = (u32)BHI_STATUS_ERROR << 30;
			*breg(BHI + BHI_ERRCODE) = 0x1234;
			*breg(BHI + BHI_ERRDBG2) = 0x01403da0;
			*breg(BHI + BHI_ERRDBG3) = 0x30;
		} else {
			dv.bhi_ok++;
			*breg(BHI + BHI_STATUS) = (u32)BHI_STATUS_SUCCESS << 30;
			dv.ee = MHI_EE_SBL;
			*breg(BHI + BHI_EXECENV) = MHI_EE_SBL;
			dv.stage = S_SBL_BOOT;
			dv.ready_in = 3;
		}
	}
	if (dv.stage == S_SBL_BOOT) {
		if (dv.stall != STALL_READY) {
			if (dv.ready_in > ms) {
				dv.ready_in -= ms;
			} else {
				*breg(R_MHISTATUS) = (MHI_STATE_READY << 8) | 1;
				dv.stage = S_READY;
			}
		}
	}
	if (dv.pending_bhie) {
		u64 vec = reg64(BHIE + BHIE_TXVECADDR_LOW_OFFS, BHIE + BHIE_TXVECADDR_HIGH_OFFS);
		u32 size = *breg(BHIE + BHIE_TXVECSIZE_OFFS), i, seq = *breg(BHIE + BHIE_TXVECDB_OFFS);
		struct { u64 a, s; } *e = (void *)iova2va(vec);
		size_t total = 0;
		bool ok = e != NULL && size % 16 == 0;

		dv.pending_bhie = false;
		if (dv.stall == STALL_BHIE) {
			return;
		}
		dv.bhie_entries = size / 16;
		for (i = 0; ok && i < size / 16; i++) {
			uint8_t *va = iova2va(e[i].a);

			if (!va || total + e[i].s > dv.expect_full_len || memcmp(va, dv.expect_full + total, e[i].s)) {
				ok = false;
				if (!va) {
					dv.faults++;
				}
			}
			total += e[i].s;
		}
		dv.bhie_total = total;
		if (ok && total == dv.expect_full_len && !dv.bhie_error) {
			dv.bhie_ok++;
			*breg(BHIE + BHIE_TXVECSTATUS_OFFS) = ((u32)BHIE_TXVECSTATUS_STATUS_XFER_COMPL << 30) | (seq & 0x3fffffff);
			dv.stage = S_BHIE_DONE;
			dv.amss_in = 2;
		} else {
			*breg(BHIE + BHIE_TXVECSTATUS_OFFS) = (u32)BHIE_TXVECSTATUS_STATUS_ERROR << 30;
		}
	}
	if (dv.stage == S_BHIE_DONE && dv.stall != STALL_AMSS) {
		if (dv.amss_in > ms) {
			dv.amss_in -= ms;
		} else {
			dv.ee = MHI_EE_AMSS;
			*breg(BHI + BHI_EXECENV) = MHI_EE_AMSS;
			dv.stage = S_AMSS;
			ev_push(0, 0, MHI_EE_AMSS << 24, PKT_EE_EVENT << 16);
		}
	}
	if (dv.pending_cmd) {
		u64 crcbap = reg64(R_CRCBAP_LOWER, R_CRCBAP_HIGHER);
		struct mhi_ctx *cc = ctx_at(crcbap, 0);
		u64 rbase = ctx_get64(cc->rbase);
		u32 n = (u32)(ctx_get64(cc->rlen) / 16), wp = (u32)((dv.cmd_db - rbase) / 16);
		struct mhi_el *el = (struct mhi_el *)iova2va(rbase);

		dv.pending_cmd = false;
		while (dv.cmd_rp != wp) {
			u32 type = (el[dv.cmd_rp].dw1 >> 16) & 0xff, ch = el[dv.cmd_rp].dw1 >> 24;
			u32 code = CC_SUCCESS;

			if (type == PKT_START_CHAN_CMD) {
				struct mhi_ctx *x = ctx_at(reg64(R_CCABAP_LOWER, R_CCABAP_HIGHER), ch);

				if (dv.fail_start_chan || (x->w0 & 0xff) != CH_ENABLED) {
					code = CC_UNDEFINED_ERR;
				} else {
					dv.chan_on[ch] = true;
					dv.chan_rp[ch] = 0;
					dv.start_cmds[ch]++;
					if (dv.nstart < 8) {
						dv.start_order[dv.nstart++] = ch;
					}
				}
			} else if (type == PKT_RESET_CHAN_CMD) {
				dv.chan_on[ch] = false;
				dv.reset_cmds[ch]++;
				if (dv.nreset < 8) {
					dv.reset_order[dv.nreset++] = ch;
				}
			}
			ev_push(0, rbase + (u64)dv.cmd_rp * 16, code << 24, (PKT_CMD_COMPLETION << 16));
			dv.cmd_rp = (dv.cmd_rp + 1) % n;
		}
	}
	while (dv.pending_ul) {
		struct mhi_ctx *x = ctx_at(reg64(R_CCABAP_LOWER, R_CCABAP_HIGHER), 20);
		u64 rbase = ctx_get64(x->rbase);
		u32 n = (u32)(ctx_get64(x->rlen) / 16);
		struct mhi_el *el = (struct mhi_el *)iova2va(rbase);

		dv.pending_ul = 0;
		while (dv.chan_on[20] && dv.chan_rp[20] != dv.chan_wp[20]) {
			struct mhi_el *t = &el[dv.chan_rp[20]];
			u32 len = t->dw0 & 0xffff;
			uint8_t *va = iova2va(t->ptr);

			MUST(va != NULL);
			MUST(dv.ul_n < 256 && len <= 9000);
			memcpy(dv.ul[dv.ul_n], va, len);
			dv.ul_len[dv.ul_n++] = len;
			ev_push(x->w2, rbase + (u64)dv.chan_rp[20] * 16, (CC_EOT << 24) | len, (PKT_TX_EVENT << 16) | (20u << 24));
			dv.chan_rp[20] = (dv.chan_rp[20] + 1) % n;
		}
	}
}

/* a packet from the radio on channel 21; false if the host has queued no buffer */
static bool
dev_dl_send(const void *pkt, u32 len)
{
	struct mhi_ctx *x = ctx_at(reg64(R_CCABAP_LOWER, R_CCABAP_HIGHER), 21);
	u64 rbase = ctx_get64(x->rbase);
	u32 n = (u32)(ctx_get64(x->rlen) / 16);
	struct mhi_el *el = (struct mhi_el *)iova2va(rbase);
	struct mhi_el *t;

	if (!dv.chan_on[21] || dv.chan_rp[21] == dv.chan_wp[21]) {
		return false;
	}
	t = &el[dv.chan_rp[21]];
	MUST(iova2va(t->ptr) != NULL);
	MUST(len <= (t->dw0 & 0xffff));
	memcpy(iova2va(t->ptr), pkt, len);
	ev_push(x->w2, rbase + (u64)dv.chan_rp[21] * 16, (CC_EOT << 24) | len, (PKT_TX_EVENT << 16) | (21u << 24));
	dv.chan_rp[21] = (dv.chan_rp[21] + 1) % n;
	return true;
}

static bool poll_in_pause;
void
lx_pause_ms(unsigned ms)
{
	now_ns += (u64)ms * 1000000ull;
	dev_tick(ms);
	if (poll_in_pause) {
		run_handlers();
	}
}

static int read_fault_at = -1;
static unsigned reads;
static int
fake_read(struct mhi_controller *c, void __iomem *addr, u32 *out)
{
	u32 off = (u32)((u8 *)addr - dv.bar);

	assert((u8 *)addr >= dv.bar && off < BAR_BYTES && !(off & 3));
	reads++;
	if (read_fault_at >= 0 && reads == (unsigned)read_fault_at) {
		return -5;
	}
	*out = dv.reads_dead ? 0xffffffffu : *breg(off);
	return 0;
}
static void
fake_write(struct mhi_controller *c, void __iomem *addr, u32 val)
{
	u32 off = (u32)((u8 *)addr - dv.bar);

	assert((u8 *)addr >= dv.bar && off < BAR_BYTES && !(off & 3));
	dev_write(off, val);
}

/* ---------------------------------------------------------------- the host side, as ath11k sets it */
static struct mhi_channel_config ch_cfg[] = {
	{ .num = 20, .name = "IPCR", .num_elements = 64, .event_ring = 1, .dir = DMA_TO_DEVICE, .ee_mask = 0x4,
	  .doorbell = MHI_DB_BRST_DISABLE },
	{ .num = 21, .name = "IPCR", .num_elements = 64, .event_ring = 1, .dir = DMA_FROM_DEVICE, .ee_mask = 0x4,
	  .doorbell = MHI_DB_BRST_DISABLE },
};
static struct mhi_event_config ev_cfg[] = {
	{ .num_elements = 32, .irq_moderation_ms = 0, .irq = 1, .mode = MHI_DB_BRST_DISABLE, .data_type = MHI_ER_CTRL },
	{ .num_elements = 256, .irq_moderation_ms = 1, .irq = 2, .mode = MHI_DB_BRST_DISABLE, .priority = 1 },
};
static struct mhi_controller_config cfg = {
	.max_channels = 128, .timeout_ms = 2000, .buf_len = 8192, .num_channels = 2, .ch_cfg = ch_cfg,
	.num_events = 2, .event_cfg = ev_cfg,
};

static struct device cdev;
static int irqs[3] = { 512, 513, 514 };
static unsigned status_cbs[16];
static void status_cb(struct mhi_controller *c, enum mhi_callback cb) { status_cbs[cb]++; }
static int rt_get(struct mhi_controller *c) { return 0; }
static void rt_put(struct mhi_controller *c) {}

#define IMG_BYTES 5300000u
#define SZ_512K (512u * 1024u)
static u8 *fw;
static u8 *fw_elf;	/* the same with a second ELF header after the SBL */

static void
chip_reset_all(void)
{
	memset(&dv, 0, sizeof(dv));
	dev_reset();
	*breg(R_CHDBOFF) = CHDB;
	*breg(R_ERDBOFF) = ERDB;
	*breg(R_BHIOFF) = BHI;
	*breg(R_BHIEOFF) = BHIE;
	dv.expect_sbl = fw;
	dv.expect_sbl_len = SZ_512K;
	dv.expect_full = fw;
	dv.expect_full_len = IMG_BYTES;
}

static struct mhi_controller *
mk(void)
{
	struct mhi_controller *c = mhi_alloc_controller();

	MUST(c != NULL);
	c->cntrl_dev = &cdev;
	c->regs = dv.bar;
	c->reg_len = BAR_BYTES;
	c->fw_data = fw;
	c->fw_sz = IMG_BYTES;
	c->irq = irqs;
	c->nr_irqs = 3;
	c->iova_start = 0;
	c->iova_stop = 0xffffffffull;	/* ath11k: the whole 32-bit DMA mask, inclusive */
	c->rddm_size = 0x420000;
	c->sbl_size = SZ_512K;
	c->seg_len = SZ_512K;
	c->fbc_download = true;
	c->runtime_get = rt_get;
	c->runtime_put = rt_put;
	c->status_cb = status_cb;
	c->read_reg = fake_read;
	c->write_reg = fake_write;
	return c;
}

static void
world(void)
{
	chip_reset_all();
	memset(status_cbs, 0, sizeof(status_cbs));
	pool_used = 0;
	pool_base = window_base = HAKIOS_WIFI_IOVA_BASE;
	window_none = false;
	dma_calls = dma_fail_at = dma_frees = 0;
	qlink = NULL;
	qlink_ups = qlink_downs = qlink_fail = 0;
	rx_packets = rx_bytes = 0;
	irq_enables = irq_disables = 0;
	handler_calls = 0;
	poll_in_pause = false;
	option_dump = 0;
	read_fault_at = -1;
	reads = 0;
	nlines = 0;
}

/* a controller on a fresh chip and an empty pool, registered and with the short test timeout */
static struct mhi_controller *
fresh(void)
{
	struct mhi_controller *c;

	world();
	c = mk();
	MUST(mhi_register_controller(c, &cfg) == 0);
	c->timeout_ms = 500;
	return c;
}
static void
done(struct mhi_controller *c)
{
	mhi_unregister_controller(c);
	mhi_free_controller(c);
	MUST(irqs_active() == 0);
}
/* what a power cycle leaves in the fake, cleared between cycles */
static void
chip_counters_clear(void)
{
	dv.bhi_ok = dv.bhie_ok = 0;
	dv.nstart = dv.nreset = 0;
	dv.wake_writes = 0;
	dv.nwl = 0;
	memset(dv.start_cmds, 0, sizeof(dv.start_cmds));
	memset(dv.reset_cmds, 0, sizeof(dv.reset_cmds));
	memset(dv.chdb_writes, 0, sizeof(dv.chdb_writes));
	memset(status_cbs, 0, sizeof(status_cbs));
}

static int
find_write(u32 off, u32 val, bool any_val, unsigned from)
{
	unsigned i;

	for (i = from; i < dv.nwl; i++) {
		if (dv.wl[i].off == off && (any_val || dv.wl[i].val == val)) {
			return (int)i;
		}
	}
	return -1;
}

/* ------------------------------------------------------------ tests */
static void
test_layout(void)
{
	CHECK(sizeof(struct mhi_ctx) == 44);
	CHECK(offsetof(struct mhi_ctx, w0) == 0 && offsetof(struct mhi_ctx, w1) == 4 && offsetof(struct mhi_ctx, w2) == 8);
	CHECK(offsetof(struct mhi_ctx, rbase) == 12 && offsetof(struct mhi_ctx, rlen) == 20);
	CHECK(offsetof(struct mhi_ctx, rp) == 28 && offsetof(struct mhi_ctx, wp) == 36);
	CHECK(sizeof(struct mhi_el) == 16 && offsetof(struct mhi_el, dw0) == 8 && offsetof(struct mhi_el, dw1) == 12);
	CHECK(sizeof(struct bhi_vec_entry) == 16);
	/* register offsets and masks of Linux common.h */
	CHECK(R_MHICTRL == 0x38 && R_MHISTATUS == 0x48 && R_CCABAP_LOWER == 0x58 && R_ECABAP_LOWER == 0x60);
	CHECK(R_CRCBAP_LOWER == 0x68 && R_CRDB_LOWER == 0x70 && R_MHICTRLBASE_LOWER == 0x80);
	CHECK(R_MHICTRLLIMIT_LOWER == 0x88 && R_MHIDATABASE_LOWER == 0x98 && R_MHIDATALIMIT_LOWER == 0xa0);
	CHECK(BHI_IMGTXDB == 0x18 && BHI_EXECENV == 0x28 && BHI_STATUS == 0x2c && BHI_ERRDBG2 == 0x38);
	CHECK(BHIE_TXVECADDR_LOW_OFFS == 0x2c && BHIE_TXVECDB_OFFS == 0x3c && BHIE_TXVECSTATUS_OFFS == 0x44);
	CHECK(MHICFG_NER_MASK == 0x00ff0000 && MHICFG_NHWER_MASK == 0xff000000 && MHICTRL_MHISTATE_MASK == 0xff00);
	CHECK(MHI_STATE_M0 == 2 && MHI_STATE_SYS_ERR == 0xff && MHI_EE_SBL == 1 && MHI_EE_AMSS == 2);
	CHECK(PKT_START_CHAN_CMD == 0x12 && PKT_RESET_CHAN_CMD == 0x10 && PKT_TX_EVENT == 0x22 && PKT_CMD_COMPLETION == 0x21);
}

static void
test_register(void)
{
	struct mhi_controller *c;
	struct mhi_controller_config bad;
	struct mhi_event_config hw[2];

	world();
	c = mk();
	CHECK(mhi_register_controller(c, NULL) == -EINVAL);
	c->read_reg = NULL;
	CHECK(mhi_register_controller(c, &cfg) == -EINVAL);
	c->read_reg = fake_read;
	c->nr_irqs = 0;
	CHECK(mhi_register_controller(c, &cfg) == -EINVAL);
	c->nr_irqs = 3;
	c->iova_start = 0xb0000000ull;	/* beyond C3's pool */
	c->iova_stop = 0xbfffffffull;
	CHECK(mhi_register_controller(c, &cfg) == -EINVAL);
	c->iova_start = 0;
	c->iova_stop = 0;
	CHECK(mhi_register_controller(c, &cfg) == -EINVAL);	/* the inclusive range [0,0] misses the pool */
	c->iova_stop = 0xffffffffull;
	bad = cfg;
	memcpy(hw, ev_cfg, sizeof(hw));
	hw[1].hardware_event = true;
	bad.event_cfg = hw;
	CHECK(mhi_register_controller(c, &bad) == -EOPNOTSUPP);
	bad = cfg;
	bad.max_channels = 21;	/* channel 21 does not fit */
	CHECK(mhi_register_controller(c, &bad) == -EINVAL);
	/* a vector that is taken: what was requested before it is let go again */
	irqt[2].used = true;
	irqt[2].arg = (void *)1;
	CHECK(mhi_register_controller(c, &cfg) == -EBUSY);
	CHECK(!irqt[0].used && !irqt[1].used && irqt[2].arg == (void *)1);
	memset(&irqt[2], 0, sizeof(irqt[2]));
	CHECK(mhi_register_controller(c, &cfg) == 0);
	CHECK(mhi_register_controller(c, &cfg) == -EBUSY);
	CHECK(c->mhi_dev != NULL && c->mhi_dev->dev_type == MHI_DEVICE_CONTROLLER && c->mhi_dev->mhi_cntrl == c);
	CHECK(c->buffer_len == 8192 && c->timeout_ms == 2000);
	CHECK(irqs_active() == 0);	/* held off until power-up, as Linux does */
	CHECK(irqt[0].used && irqt[1].used && irqt[2].used);
	{
		struct mhi_controller *c2 = mk();

		CHECK(mhi_register_controller(c2, &cfg) == -EBUSY);	/* one radio */
		mhi_free_controller(c2);
	}
	done(c);
	CHECK(!irqt[0].used && !irqt[1].used && !irqt[2].used);
}

static void
test_prepare_encoding(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	u64 lo = 0xa0000000ull, hi = 0xa3ffffffull;
	unsigned i, n = 0;

	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(p->bhi == dv.bar + BHI && p->bhie == dv.bar + BHIE);
	CHECK(c->rddm_image == NULL && c->fbc_image == NULL);
	CHECK(lines_with("RDDM is not supported", 0) == 1);
	/* channel contexts: IPCR 20 out, 21 in, event ring 1; the others are zero */
	CHECK(p->chan_ctx[20].w0 == (CH_DISABLED | (MHI_DB_BRST_DISABLE << 8)) && p->chan_ctx[20].w0 == 0x200);
	CHECK(p->chan_ctx[20].w1 == 1 && p->chan_ctx[20].w2 == 1);
	CHECK(p->chan_ctx[21].w0 == 0x200 && p->chan_ctx[21].w1 == 2 && p->chan_ctx[21].w2 == 1);
	CHECK(p->chan_ctx[5].w0 == 0 && p->chan_ctx[5].w1 == 0 && p->chan_ctx[127].w2 == 0);
	/* event contexts: moderation in 31:16, type valid, the MSI vector the ring's irq index */
	CHECK(p->er_ctx[0].w0 == 0 && p->er_ctx[0].w1 == 1 && p->er_ctx[0].w2 == 1);
	CHECK(p->er_ctx[1].w0 == (1u << 16) && p->er_ctx[1].w1 == 1 && p->er_ctx[1].w2 == 2);
	CHECK(ctx_get64(p->er_ctx[0].rlen) == 32 * 16 && ctx_get64(p->er_ctx[1].rlen) == 256 * 16);
	CHECK(ctx_get64(p->er_ctx[0].rbase) % 512 == 0 && ctx_get64(p->er_ctx[1].rbase) % 4096 == 0);
	CHECK(ctx_get64(p->er_ctx[0].rp) == ctx_get64(p->er_ctx[0].rbase) && ctx_get64(p->er_ctx[1].wp) == ctx_get64(p->er_ctx[1].rbase));
	CHECK(ctx_get64(p->cmd_ctx[0].rlen) == 128 * 16 && ctx_get64(p->cmd_ctx[0].rbase) % 2048 == 0);
	CHECK(ctx_get64(p->er_ctx[0].rbase) == p->ev[0].ring.iova && p->cmd.iova == ctx_get64(p->cmd_ctx[0].rbase));
	/* everything the radio will read or write is inside C3's pool and the caller's window */
	CHECK(p->win_lo == lo && p->win_hi == hi);
	for (i = 0; i < MAX_BLK; i++) {
		if (p->blk[i].taken) {
			n++;
			CHECK(p->blk[i].dma >= lo && p->blk[i].dma + p->blk[i].size - 1 <= hi);
		}
	}
	CHECK(n >= 6);
	/* a second prepare uses what was taken */
	{
		u32 before = pool_used;

		CHECK(mhi_prepare_for_power_up(c) == 0);
		CHECK(pool_used == before);
	}
	mhi_unprepare_after_power_down(c);
	CHECK(p->bhi == NULL && p->chan_ctx == NULL);
	done(c);
}

static void
test_bhie_table(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	struct image_info *img = NULL;
	struct bhi_vec_entry *v;
	u32 i, used;

	MUST(image_table_alloc(p, &img, IMG_BYTES, SZ_512K) == 0);
	image_copy(fw, IMG_BYTES, img);
	CHECK(img->entries == 12);	/* 11 segments of 512 KiB (5.3 MB is 10.1 of them), then the table */
	v = (struct bhi_vec_entry *)img->bhi_vec;
	CHECK((void *)v == img->mhi_buf[11].buf);
	CHECK(img->mhi_buf[11].len == 11 * sizeof(struct bhi_vec_entry));
	for (i = 0; i < 11; i++) {
		CHECK(img->mhi_buf[i].len == SZ_512K);
		CHECK(v[i].dma_addr == img->mhi_buf[i].dma_addr);
		CHECK(v[i].size == (i < 10 ? SZ_512K : IMG_BYTES - 10u * SZ_512K));
		CHECK(!memcmp(img->mhi_buf[i].buf, fw + (size_t)i * SZ_512K, v[i].size));
		CHECK(img->mhi_buf[i].dma_addr >= 0xa0000000ull && img->mhi_buf[i].dma_addr + SZ_512K <= 0xa4000000ull);
	}
	CHECK(v[10].size == 5300000u - 10u * 524288u && v[10].size == 57120);
	used = pool_used;
	image_free(p, img);
	MUST(image_table_alloc(p, &img, IMG_BYTES, SZ_512K) == 0);	/* the same blocks again */
	CHECK(pool_used == used);
	image_free(p, img);
	/* an image that is exactly one segment, and one byte more */
	MUST(image_table_alloc(p, &img, SZ_512K, SZ_512K) == 0);
	CHECK(img->entries == 2 && img->mhi_buf[1].len == 16);
	image_free(p, img);
	MUST(image_table_alloc(p, &img, SZ_512K + 1, SZ_512K) == 0);
	CHECK(img->entries == 3 && img->mhi_buf[2].len == 32);
	image_free(p, img);
	CHECK(image_table_alloc(p, &img, 0, SZ_512K) == -EINVAL);
	/* an allocation failing part-way leaves nothing held */
	dma_fail_at = dma_calls + 5;
	CHECK(image_table_alloc(p, &img, 20u * SZ_512K, SZ_512K) == -ENOMEM);
	dma_fail_at = 0;
	done(c);
}

/* the whole path, with checks on what the chip saw */
static void
check_powered(struct mhi_controller *c, u32 expect_entries, size_t expect_total)
{
	struct mhi_priv *p = PRIV(c);
	int i_bhi, i_cc, i_er, i_m0, i_bhie, i_cmd;
	unsigned i;

	CHECK(p->pm == PM_M0 && p->ee == MHI_EE_AMSS && p->dev_state == MHI_STATE_M0);
	CHECK(dv.bhi_ok == 1 && dv.bhie_ok == 1 && dv.faults == 0 && dv.ev_overrun == 0);
	CHECK(dv.bhie_entries == expect_entries - 1 && dv.bhie_total == expect_total);
	CHECK(c->fbc_image != NULL && c->fbc_image->entries == expect_entries && c->rddm_image == NULL);
	CHECK(status_cbs[MHI_CB_EE_MISSION_MODE] == 1 && status_cbs[MHI_CB_SYS_ERROR] == 0);
	/* the programmed MMIO: contexts, event count, and the window cut to the pool */
	CHECK(reg64(R_CCABAP_LOWER, R_CCABAP_HIGHER) == p->chan_ctx_dma);
	CHECK(reg64(R_ECABAP_LOWER, R_ECABAP_HIGHER) == p->er_ctx_dma);
	CHECK(reg64(R_CRCBAP_LOWER, R_CRCBAP_HIGHER) == p->cmd_ctx_dma);
	CHECK(((*breg(R_MHICFG) & MHICFG_NER_MASK) >> 16) == 2 && (*breg(R_MHICFG) >> 24) == 0);
	CHECK(reg64(R_MHICTRLBASE_LOWER, R_MHICTRLBASE_HIGHER) == window_base);
	CHECK(reg64(R_MHIDATABASE_LOWER, R_MHIDATABASE_HIGHER) == window_base);
	CHECK(reg64(R_MHICTRLLIMIT_LOWER, R_MHICTRLLIMIT_HIGHER) == window_base + HAKIOS_WIFI_POOL_SIZE - 1);
	CHECK(reg64(R_MHIDATALIMIT_LOWER, R_MHIDATALIMIT_HIGHER) == window_base + HAKIOS_WIFI_POOL_SIZE - 1);
	/* order: BHI, contexts, event ring doorbells, M0, BHIe, channel commands */
	i_bhi = find_write(BHI + BHI_IMGTXDB, 0, true, 0);
	i_cc = find_write(R_CCABAP_LOWER, 0, true, 0);
	i_er = find_write(ERDB + 8 + 4, 0, true, 0);
	i_m0 = find_write(R_MHICTRL, 0, true, 0);
	while (i_m0 >= 0 && ((dv.wl[i_m0].val & MHICTRL_MHISTATE_MASK) >> 8) != MHI_STATE_M0) {
		i_m0 = find_write(R_MHICTRL, 0, true, (unsigned)i_m0 + 1);
	}
	i_bhie = find_write(BHIE + BHIE_TXVECDB_OFFS, 0, true, 0);
	i_cmd = find_write(R_CRDB_LOWER, 0, true, 0);
	CHECK(i_bhi >= 0 && i_cc > i_bhi && i_er > i_cc && i_m0 > i_er && i_bhie > i_m0 && i_cmd > i_bhie);
	/* doorbells: the high word goes first; each event ring is given to the chip but one element */
	CHECK(i_er >= 0 && dv.wl[i_er + 1].off == ERDB + 8);
	CHECK(dv.wl[i_er].val == hi32(p->ev[1].ring.iova + 255ull * 16) && dv.wl[i_er + 1].val == lo32(p->ev[1].ring.iova + 255ull * 16));
	{
		int e0 = find_write(ERDB + 4, 0, true, 0);

		CHECK(e0 >= 0 && e0 < i_m0 && dv.wl[e0 + 1].off == ERDB && dv.wl[e0 + 1].val == lo32(p->ev[0].ring.iova + 31ull * 16));
	}
	/* device wake asserted once M0 was seen, and still held */
	CHECK(dv.wake_last == 1 && dv.wake_writes == 1 && p->wake_set && p->wake_hold);
	/* channels 20 then 21 started; their contexts are the rings' */
	CHECK(dv.start_cmds[20] == 1 && dv.start_cmds[21] == 1 && dv.nstart == 2 && dv.start_order[0] == 20 && dv.start_order[1] == 21);
	CHECK((p->chan_ctx[20].w0 & 0xff) == CH_ENABLED && ctx_get64(p->chan_ctx[20].rlen) == 64 * 16);
	CHECK(ctx_get64(p->chan_ctx[21].rbase) == p->chan[21].tre.iova && p->chan[21].tre.iova % 1024 == 0);
	CHECK(dv.chan_wp[21] == 63 && dv.chdb_writes[21] == 63);	/* 63 receive buffers queued, a ring write each */
	/* the IPCR link, published through C4 */
	CHECK(qlink_ups >= 1 && qlink != NULL && !strcmp(qlink->name, "mhi-ipcr") && qlink->send != NULL);
	CHECK(irqs_active() == 3);
	/* one line per state change */
	{
		static const char *const want[] = { "power up requested", "BHI: done", "READY", "MMIO contexts programmed",
		    "M0 requested", "M0 (event)", "EE SBL", "BHIe: sending", "BHIe: done", "EE AMSS: mission mode",
		    "channel 20 started", "channel 21 started", "IPCR: link up", "power up done" };

		for (i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
			CHECK(lines_with(want[i], 0) >= 1);
		}
	}
}

static void
test_power_up(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	u32 cycle1_pool;
	int cycle;

	said.count = 0;
	MUST(mhi_prepare_for_power_up(c) == 0);
	poll_in_pause = true;	/* the polling thread runs while this one sleeps, as in lx_pci.c */
	MUST(mhi_sync_power_up(c) == 0);
	/* WP6: stage 5 (probe) when the chip is about to be powered, stage 6 (mhi: EE AMSS) when mission mode is reached: that order */
	CHECK(said.count == 2 && said.n[0] == 5 && said.ok[0] == 1 && said.n[1] == 6 && said.ok[1] == 1);
	CHECK(strstr(said.detail[1], "EE AMSS") != NULL);
	check_powered(c, 12, IMG_BYTES);
	CHECK(handler_calls > 0);
	CHECK(mhi_get_mhi_state(c) == MHI_STATE_M0);
	CHECK(mhi_prepare_for_power_up(c) == -EBUSY);
	CHECK(mhi_async_power_up(c) == -EBUSY);
	mhi_power_down(c, true);
	CHECK(p->pm == PM_DISABLE && dv.resets >= 2 && qlink == NULL && qlink_downs == 1);
	CHECK(irqs_active() == 0 && irq_disables == irq_enables + 3);	/* (held off at register, on, off) */
	CHECK(dv.wake_last == 0 && !p->wake_set);
	CHECK(p->chan[20].tre.el == NULL && p->chan[21].tre.el == NULL);
	mhi_unprepare_after_power_down(c);
	CHECK(c->fbc_image == NULL);
	cycle1_pool = pool_used;
	/* again and again: the chip restarts from PBL, nothing new is taken from the pool */
	for (cycle = 0; cycle < 3; cycle++) {
		unsigned ups = qlink_ups;

		chip_counters_clear();
		MUST(mhi_prepare_for_power_up(c) == 0);
		MUST(mhi_sync_power_up(c) == 0);
		check_powered(c, 12, IMG_BYTES);
		CHECK(qlink_ups == ups + 1);
		mhi_power_down(c, true);
		mhi_unprepare_after_power_down(c);
		CHECK(pool_used == cycle1_pool);
	}
	CHECK(dma_frees == 0);	/* nothing is given back while the controller lives: the pool never takes it */
	{
		unsigned blocks = 0, i;

		for (i = 0; i < MAX_BLK; i++) {
			blocks += p->blk[i].taken;
		}
		mhi_unregister_controller(c);
		mhi_free_controller(c);
		CHECK(dma_frees == blocks);	/* at the end each block goes back to dma_free_coherent once */
	}
}

static void
test_second_elf(void)
{
	struct mhi_controller *c = fresh();

	/* a file with a second ELF after the SBL: BHIe carries only that part */
	c->fw_data = fw_elf;
	dv.expect_sbl = fw_elf;
	dv.expect_full = fw_elf + SZ_512K;
	dv.expect_full_len = IMG_BYTES - SZ_512K;
	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	check_powered(c, 11, IMG_BYTES - SZ_512K);	/* 4775712 bytes: 10 segments and the table */
	mhi_power_down(c, true);
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
test_fw_by_name(void)
{
	struct mhi_controller *c = fresh();

	fw_file = fw;
	fw_file_len = IMG_BYTES;
	c->fw_data = NULL;
	c->fw_sz = 0;
	c->fw_image = "qcom/amss.bin";
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == 0);
	check_powered(c, 12, IMG_BYTES);
	mhi_power_down(c, true);
	mhi_unprepare_after_power_down(c);
	/* a name that is not there */
	chip_counters_clear();
	c->fw_image = "qcom/missing.bin";
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == -ENOENT);
	CHECK(PRIV(c)->pm == PM_DISABLE && dv.bhi_ok == 0 && dv.resets >= 2 && irqs_active() == 0);
	mhi_unprepare_after_power_down(c);
	/* no image at all */
	c->fw_image = NULL;
	c->fw_data = NULL;
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == -EINVAL);
	CHECK(PRIV(c)->pm == PM_DISABLE && irqs_active() == 0);
	mhi_unprepare_after_power_down(c);
	fw_file = NULL;
	done(c);
}

static void
stall_case(enum stall s, const char *what)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	u64 t0 = now_ns, spent;
	u32 pool_first;

	option_dump = 1;
	dv.stall = s;
	said.count = 0;
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == -ETIMEDOUT);
	CHECK(said.count == 2 && said.n[0] == 5 && said.ok[0] == 1 && said.n[1] == 6 && said.ok[1] == 0);	/* WP6: a stall is stage 6 FAILED */
	spent = (now_ns - t0) / 1000000ull;
	CHECK(spent >= 500 && spent < 4 * 500 + 200);	/* bounded: a few of the timeouts at most */
	CHECK(p->pm == PM_DISABLE && p->ee == MHI_EE_DISABLE_TRANSITION);
	CHECK(dv.resets >= 2);	/* the chip was told to reset (the fake counts its creation as one) */
	CHECK(irqs_active() == 0 && qlink == NULL && qlink_ups == 0 && dv.wake_last == 0);
	CHECK(lines_with("dump (", 0) >= 1 && lines_with("SBL log pointer", 0) >= 1);	/* wifi_mhi_dump=1 */
	CHECK(lines_with(what, 0) >= 1);
	mhi_unprepare_after_power_down(c);
	CHECK(c->fbc_image == NULL);
	pool_first = pool_used;
	/* and the same radio starts the next time, with nothing new from the pool but the channel rings */
	dv.stall = STALL_NONE;
	chip_counters_clear();
	nlines = 0;
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == 0);
	CHECK(pool_used < pool_first + 8 * 1024 * 1024);
	mhi_power_down(c, true);
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
failure_case(const char *name, int want, void (*arm)(void), const char *line)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	bool dead = !strcmp(name, "dead");

	MUST(mhi_prepare_for_power_up(c) == 0);
	arm();
	CHECK(mhi_sync_power_up(c) == want);
	CHECK(p->pm == PM_DISABLE && irqs_active() == 0 && qlink == NULL && qlink_ups == 0);
	CHECK(dead ? dv.resets >= 1 : dv.resets >= 2);
	CHECK(lines_with(line, 0) >= 1);
	CHECK(dv.wake_last == 0);
	mhi_unprepare_after_power_down(c);
	CHECK(c->fbc_image == NULL);
	done(c);
}
static void arm_bhi_error(void) { dv.bhi_error = true; }
static void arm_bhie_error(void) { dv.bhie_error = true; }
static void arm_syserr(void) { dv.syserr_at_m0 = true; }
static void arm_start_fail(void) { dv.fail_start_chan = true; }
static void arm_dead(void) { dv.reads_dead = true; }

static void
test_failures(void)
{
	failure_case("bhi", -EIO, arm_bhi_error, "BHI: image transfer failed");
	failure_case("bhie", -EIO, arm_bhie_error, "BHIe: transfer failed");
	failure_case("syserr", -EIO, arm_syserr, "system error");
	CHECK(status_cbs[MHI_CB_SYS_ERROR] == 1);
	failure_case("start", -EIO, arm_start_fail, "command completion code");
	failure_case("dead", -EIO, arm_dead, "not valid for power up");
}

static void
test_window(void)
{
	struct mhi_controller *c;

	/* the caller allows 64 KiB of the pool only: the 512 KiB SBL buffer cannot be had */
	world();
	c = mk();
	c->iova_start = 0;
	c->iova_stop = 0xa000ffffull;
	MUST(mhi_register_controller(c, &cfg) == 0);
	c->timeout_ms = 200;
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(PRIV(c)->win_hi == 0xa000ffffull);
	CHECK(mhi_sync_power_up(c) == -ENOMEM);
	CHECK(lines_with("outside the pool window", 0) >= 1);
	CHECK(dma_frees >= 1);	/* the block outside is handed straight back */
	CHECK(dv.bhi_ok == 0 && PRIV(c)->pm == PM_DISABLE);
	mhi_unprepare_after_power_down(c);
	done(c);
	/* blocks that are not where the pool says it is: nothing can be taken */
	world();
	pool_base = 0x30000000ull;
	c = mk();
	MUST(mhi_register_controller(c, &cfg) == 0);
	CHECK(mhi_prepare_for_power_up(c) == -ENOMEM);
	CHECK(PRIV(c)->chan_ctx == NULL && PRIV(c)->er_ctx == NULL && !PRIV(c)->prepared);
	done(c);
	/* no pool yet: the controller is refused */
	world();
	window_none = true;
	c = mk();
	CHECK(mhi_register_controller(c, &cfg) == -ENODEV);
	CHECK(lines_with("no DMA pool", 0) == 1);
	mhi_free_controller(c);
	window_none = false;
	/* wifi_smmu=bypass (WP8 F2): the pool's radio address is its physical address, below 4 GiB but not
	 * C3's IOVA; the window follows it and the whole path still runs */
	world();
	pool_base = window_base = 0x88000000ull;
	c = mk();
	MUST(mhi_register_controller(c, &cfg) == 0);
	c->timeout_ms = 500;
	CHECK(PRIV(c)->win_lo == 0x88000000ull && PRIV(c)->win_hi == 0x8bffffffull);
	MUST(mhi_prepare_for_power_up(c) == 0);
	poll_in_pause = true;
	CHECK(mhi_sync_power_up(c) == 0);
	check_powered(c, 12, IMG_BYTES);
	CHECK(lines_with("outside the pool window", 0) == 0);
	mhi_power_down(c, true);
	mhi_unprepare_after_power_down(c);
	done(c);
	/* an allocation that fails in the middle of preparing leaves nothing half-made */
	c = fresh();
	dma_fail_at = dma_calls + 4;
	CHECK(mhi_prepare_for_power_up(c) == -ENOMEM);
	CHECK(PRIV(c)->chan_ctx == NULL && !PRIV(c)->prepared && PRIV(c)->cmd.el == NULL && PRIV(c)->ev[0].ring.el == NULL);
	dma_fail_at = 0;
	CHECK(mhi_prepare_for_power_up(c) == 0);
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
test_refusals(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	unsigned n0 = nlines;

	CHECK(mhi_pm_suspend(c) == -EOPNOTSUPP);
	CHECK(mhi_pm_resume_force(c) == -EOPNOTSUPP);
	CHECK(mhi_download_rddm_image(c, false) == -EOPNOTSUPP);
	CHECK(mhi_download_rddm_image(c, true) == -EOPNOTSUPP);
	CHECK(lines_with("suspend is not supported", n0) == 1 && lines_with("resume is not supported", n0) == 1);
	CHECK(lines_with("RDDM download is not supported", n0) == 2);
	CHECK(c->rddm_image == NULL);
	/* not powered: nothing acts as if it were */
	CHECK(mhi_device_get_sync(c->mhi_dev) == -EIO);
	CHECK(mhi_get_free_desc_count(c->mhi_dev, DMA_FROM_DEVICE) == -EINVAL);
	CHECK(mhi_queue_buf(c->mhi_dev, DMA_TO_DEVICE, &n0, 4, MHI_EOT) == -EINVAL);
	CHECK(mhi_async_power_up(c) == -EINVAL);	/* not prepared */
	CHECK(mhi_prepare_for_transfer(c->mhi_dev) == -EIO);
	CHECK(mhi_get_mhi_state(NULL) == MHI_STATE_MAX);
	/* powered: suspend still says no, and the link stays */
	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	CHECK(mhi_pm_suspend(c) == -EOPNOTSUPP && p->pm == PM_M0 && qlink != NULL);
	mhi_power_down_keep_dev(c, true);	/* the devices go with it, and it says so */
	CHECK(p->pm == PM_DISABLE && qlink == NULL && lines_with("suspend is not supported, so the devices go too", 0) == 1);
	mhi_unprepare_after_power_down(c);
	/* unprepare of something powered quiesces it first */
	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	mhi_unprepare_after_power_down(c);
	CHECK(p->pm == PM_DISABLE && c->fbc_image == NULL && qlink == NULL && irqs_active() == 0);
	CHECK(lines_with("unprepare while powered", 0) == 1);
	done(c);
}

/* QRTR v1: eight little-endian words of header, then the payload padded to four bytes */
static u32
qpacket(u8 *buf, u32 payload, u8 fill)
{
	u32 *h = (u32 *)buf, pad = (payload + 3) & ~3u;

	h[0] = 1;	/* version */
	h[1] = 1;	/* DATA */
	h[2] = 0;
	h[3] = 0x40;
	h[4] = 0;
	h[5] = payload;
	h[6] = 1;
	h[7] = 0x41;
	memset(buf + 32, fill, pad);
	return 32 + pad;
}

static struct mhi_device *
ipcr_dev(struct mhi_controller *c)
{
	unsigned i;

	for (i = 0; i < MAX_DEV; i++) {
		if (PRIV(c)->dev_slot[i].dev && !strcmp(PRIV(c)->dev_slot[i].dev->name, "IPCR")) {
			return PRIV(c)->dev_slot[i].dev;
		}
	}
	return NULL;
}

static void
test_ipcr(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	u8 pkt[32 + 8192], scratch[32 + 8192];
	u32 len, i, k;
	struct mhi_device *d;
	int r;

	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	d = ipcr_dev(c);
	MUST(d != NULL && d->ul_chan_id == 20 && d->dl_chan_id == 21 && d->id != NULL);
	CHECK(mhi_get_free_desc_count(d, DMA_FROM_DEVICE) == 0);	/* all 63 receive buffers are queued */
	CHECK(mhi_get_free_desc_count(d, DMA_TO_DEVICE) == 63);

	/* RX: one packet from the chip, delivered whole through C4, the buffer queued again */
	len = qpacket(pkt, 13, 0xa5);
	CHECK(dev_dl_send(pkt, len));
	CHECK(lx_mhi_poll(c) >= 1);
	CHECK(rx_packets == 1 && rx_bytes == len && !memcmp(rx_last, pkt, len));
	CHECK(mhi_get_free_desc_count(d, DMA_FROM_DEVICE) == 0);	/* recycled */
	/* the polling thread's handlers do the same */
	len = qpacket(pkt, 100, 0x5a);
	CHECK(dev_dl_send(pkt, len));
	run_handlers();
	CHECK(rx_packets == 2 && !memcmp(rx_last, pkt, len));
	/* many more than the ring holds: the buffers keep coming back */
	for (i = 0; i < 300; i++) {
		len = qpacket(pkt, (i * 7) % 8000, (u8)i);
		MUST(dev_dl_send(pkt, len));
		run_handlers();
		CHECK(rx_packets == 3 + i && !memcmp(rx_last, pkt, len));
	}
	/* a packet the router refuses (not a multiple of four) does not cost a buffer */
	{
		u8 odd[33] = { 1 };
		unsigned before = rx_packets;

		MUST(dev_dl_send(odd, 33));
		run_handlers();
		CHECK(rx_packets == before && lines_with("invalid router packet", 0) == 1);
		CHECK(mhi_get_free_desc_count(d, DMA_FROM_DEVICE) == 0);
	}
	CHECK(dv.faults == 0 && dv.ev_overrun == 0);

	/* TX: copied when queued (the caller's buffer is scribbled on at once), seen by the chip later */
	len = qpacket(pkt, 40, 0x3c);
	memcpy(scratch, pkt, len);
	dv.ul_n = 0;
	r = qlink->send(qlink->ctx, scratch, len);
	CHECK(r == 0);
	memset(scratch, 0xee, len);
	CHECK(dv.ul_n == 0);	/* the chip has not looked yet */
	lx_pause_ms(1);
	CHECK(dv.ul_n == 1 && dv.ul_len[0] == len && !memcmp(dv.ul[0], pkt, len));
	lx_mhi_poll(c);
	CHECK(mhi_get_free_desc_count(d, DMA_TO_DEVICE) == 63);	/* the completion gave the slot back */
	/* more packets than the ring holds, with no time passing: the ring says EAGAIN, nothing is lost */
	dv.ul_n = 0;
	for (i = 0, k = 0; i < 70; i++) {
		len = qpacket(pkt, 8 + i, (u8)i);
		r = qlink->send(qlink->ctx, pkt, len);
		if (r == 0) {
			k++;
		} else {
			CHECK(r == EAGAIN);
		}
	}
	CHECK(k == 63);
	lx_pause_ms(1);
	lx_mhi_poll(c);
	CHECK(dv.ul_n == 63);
	for (i = 0; i < 63; i++) {
		CHECK(dv.ul_len[i] == 32 + ((8 + i + 3) & ~3u) && dv.ul[i][32] == (u8)i);
	}
	CHECK(mhi_get_free_desc_count(d, DMA_TO_DEVICE) == 63);
	/* limits: C6's longest packet goes; a longer one and an empty one do not */
	len = qpacket(pkt, 8192, 0x11);
	CHECK(len == 32 + 8192 && qlink->send(qlink->ctx, pkt, len) == 0);
	CHECK(qlink->send(qlink->ctx, pkt, 32 + 8192 + 1) == EINVAL);
	CHECK(qlink->send(qlink->ctx, pkt, 0) == EINVAL);
	CHECK(qlink->send(qlink->ctx, NULL, 8) == EINVAL);
	lx_pause_ms(1);
	lx_mhi_poll(c);
	CHECK(dv.ul_len[dv.ul_n - 1] == 32 + 8192 && !memcmp(dv.ul[dv.ul_n - 1], pkt, len));
	/* a socket buffer the same way */
	{
		struct sk_buff skb = { .data = pkt, .len = 36 };
		unsigned n = dv.ul_n;

		len = qpacket(pkt, 4, 0x77);
		CHECK(len == 36);
		CHECK(mhi_queue_skb(d, DMA_TO_DEVICE, &skb, 36, MHI_EOT) == 0);
		CHECK(mhi_queue_skb(d, DMA_TO_DEVICE, &skb, 37, MHI_EOT) == -EINVAL);
		CHECK(mhi_queue_buf(d, DMA_TO_DEVICE, pkt, 8, MHI_CHAIN) == -EOPNOTSUPP);
		lx_pause_ms(1);
		lx_mhi_poll(c);
		CHECK(dv.ul_n == n + 1 && !memcmp(dv.ul[n], pkt, 36));
	}
	/* the device wake the clients ask for rides on the one held since M0 */
	{
		u32 w = dv.wake_writes;

		CHECK(mhi_device_get_sync(c->mhi_dev) == 0);
		CHECK(c->mhi_dev->dev_wake == 1 && p->dev_wake == 2);
		mhi_device_put(c->mhi_dev);
		CHECK(c->mhi_dev->dev_wake == 0 && p->dev_wake == 1 && dv.wake_writes == w && dv.wake_last == 1);
	}

	/* the driver goes while powered: reset commands, the link down, the channels given back */
	{
		unsigned downs = qlink_downs, ups = qlink_ups;

		dv.nreset = 0;
		mhi_driver_unregister(&ipcr_driver);
		CHECK(qlink_downs == downs + 1 && qlink == NULL);
		CHECK(dv.nreset == 2 && dv.reset_order[0] == 21 && dv.reset_order[1] == 20);
		CHECK(p->chan[20].tre.el == NULL && p->chan[21].tre.el == NULL);
		CHECK(qpacket(pkt, 4, 0) && !dev_dl_send(pkt, 36));
		/* and registering it again meets the device that is still there */
		CHECK(__mhi_driver_register(&ipcr_driver, NULL) == 0);
		CHECK(qlink_ups == ups + 1 && qlink != NULL);
		CHECK(mhi_get_free_desc_count(d, DMA_FROM_DEVICE) == 0);
		len = qpacket(pkt, 20, 0x99);
		CHECK(dev_dl_send(pkt, len));
		lx_mhi_poll(c);
		CHECK(!memcmp(rx_last, pkt, len));
	}

	/* the radio goes away: the link goes down first */
	{
		unsigned downs = qlink_downs;

		mhi_power_down(c, true);
		CHECK(qlink_downs == downs + 1 && qlink == NULL && p->pm == PM_DISABLE);
		CHECK(mhi_queue_buf(c->mhi_dev, DMA_TO_DEVICE, pkt, 8, MHI_EOT) == -EINVAL);
	}
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
test_link_refused(void)
{
	struct mhi_controller *c = fresh();

	qlink_fail = EBUSY;	/* the router has a link already */
	MUST(mhi_prepare_for_power_up(c) == 0);
	CHECK(mhi_sync_power_up(c) == -EBUSY);
	CHECK(PRIV(c)->pm == PM_DISABLE && qlink == NULL);
	CHECK(dv.reset_cmds[20] + dv.reset_cmds[21] == 2);	/* the channels it had started are reset again */
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
test_syserr_in_operation(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);

	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	/* the firmware dies: the controller tells ath11k once, and everything after says EIO */
	ev_push(0, 0, (u32)MHI_STATE_SYS_ERR << 24, PKT_STATE_CHANGE << 16);
	lx_mhi_poll(c);
	CHECK(p->pm == PM_SYS_ERR_FAIL && status_cbs[MHI_CB_SYS_ERROR] == 1);
	ev_push(0, 0, (u32)MHI_STATE_SYS_ERR << 24, PKT_STATE_CHANGE << 16);
	lx_mhi_poll(c);
	CHECK(status_cbs[MHI_CB_SYS_ERROR] == 1);
	CHECK(mhi_queue_buf(ipcr_dev(c), DMA_TO_DEVICE, &c, 8, MHI_EOT) == -EIO);
	CHECK(mhi_device_get_sync(c->mhi_dev) == -EIO);
	CHECK(mhi_download_rddm_image(c, false) == -EOPNOTSUPP);
	/* a firmware crash that arrives as an RDDM event is reported too */
	mhi_power_down(c, true);
	CHECK(p->pm == PM_DISABLE && qlink == NULL);
	mhi_unprepare_after_power_down(c);
	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_sync_power_up(c) == 0);
	ev_push(0, 0, MHI_EE_RDDM << 24, PKT_EE_EVENT << 16);
	lx_mhi_poll(c);
	CHECK(status_cbs[MHI_CB_EE_RDDM] == 1 && p->ee == MHI_EE_RDDM);
	ev_push(0, 0, MHI_EE_RDDM << 24, PKT_EE_EVENT << 16);
	lx_mhi_poll(c);
	CHECK(status_cbs[MHI_CB_EE_RDDM] == 1);
	mhi_power_down(c, true);
	mhi_unprepare_after_power_down(c);
	done(c);
}

static void
test_async_and_poll_only(void)
{
	struct mhi_controller *c = fresh();
	struct mhi_priv *p = PRIV(c);
	unsigned i;

	/* async power-up returns at once; the handlers (the polling thread) carry it all the way */
	MUST(mhi_prepare_for_power_up(c) == 0);
	MUST(mhi_async_power_up(c) == 0);
	CHECK(p->pm == PM_POR && irqs_active() == 3 && dv.bhi_ok == 0);
	for (i = 0; i < 4000 && !p->mission_done; i++) {
		run_handlers();
		lx_pause_ms(1);
	}
	CHECK(p->mission_done && p->pm == PM_M0 && qlink != NULL && dv.bhi_ok == 1 && dv.bhie_ok == 1);
	mhi_power_down(c, true);
	CHECK(p->pm == PM_DISABLE);
	/* the handlers do nothing once it is down */
	{
		unsigned r0 = reads;

		run_handlers();
		CHECK(reads == r0);
	}
	mhi_unprepare_after_power_down(c);
	done(c);
}

int
main(void)
{
	size_t i;

	fw = malloc(IMG_BYTES);
	fw_elf = malloc(IMG_BYTES);
	for (i = 0; i < IMG_BYTES; i++) {
		fw[i] = (u8)(i * 2654435761u >> 24);
	}
	memcpy(fw_elf, fw, IMG_BYTES);
	memcpy(fw_elf, "\177ELF", 4);
	memcpy(fw_elf + SZ_512K, "\177ELF", 4);
	test_layout();
	printf("layout: ok\n");
	test_register();
	printf("register: ok\n");
	test_prepare_encoding();
	printf("prepare, context and ring encoding: ok\n");
	test_bhie_table();
	printf("BHIe vector table, 5.3 MB in 512 KiB segments: ok\n");
	test_power_up();
	printf("power-up against the fake chip, and three more times: ok\n");
	test_second_elf();
	printf("second ELF after the SBL: ok\n");
	test_fw_by_name();
	printf("firmware by name: ok\n");
	stall_case(STALL_BHI, "BHI: no completion");
	stall_case(STALL_READY, "the device did not enter READY");
	stall_case(STALL_M0, "power up timed out");
	stall_case(STALL_BHIE, "BHIe: no completion");
	stall_case(STALL_AMSS, "power up timed out");
	printf("stalls time out, clean up and recover: ok\n");
	test_failures();
	printf("failures (BHI, BHIe, system error, channel start, dead link): ok\n");
	test_window();
	printf("pool window: ok\n");
	test_refusals();
	printf("refusals (suspend, resume, RDDM): ok\n");
	test_ipcr();
	printf("IPCR one packet each way and beyond, driver bind and unbind: ok\n");
	test_link_refused();
	test_syserr_in_operation();
	test_async_and_poll_only();
	printf("mhi-test: %d checks, %d failed\n", checks, fails);
	return fails ? 1 : 0;
}
