/*
 * Shared apps-SMMU v2 stage-1 contexts (WP2).
 * Setup writes below retain hakios.sound.c's order and values:
 * SCTLR=0, CBA2R=1, CBAR=0x1f300, TCR2, TCR, TTBR0 lo/hi,
 * TTBR1 lo/hi=0, MAIR0, MAIR1=0, FSR, TLBIASID, TLBSYNC,
 * SCTLR=ON, S2CR=context, SMR=VALID|mask|sid.
 * Deliberate changes: guarded reads/writes; max(NUMS2CB,1) bank search;
 * failed/refused banks skipped; 500ms TLB bound; never restore CBAR/CBA2R.
 * Source: prior sound.c:679-703; iOSPlus10 0325 patch:835-922
 * (free-bank CBAR type/VMID checks), 0297 (main-owned MMIO fault guard).
 * [V] source sequences; [I] shared implementation, not phone-tested.
 * Audio now maps pages after claim: additional bounded TLB invalidations.
 * A failed disable/sync quarantines the context and its wired tables forever;
 * reusing/freeing memory still visible to hardware would be unsafe.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/hakios.smmu.h>
#include <sys/hakios.mmio.h>
#include <vm/vm_kern.h>

extern lck_mtx_t hakios_smmu_lock;
extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern uint64_t kvtophys(vm_offset_t);
extern void loader_log_putc(char);

#define SMR(n) (0x800 + 4 * (n))
#define S2CR(n) (0xc00 + 4 * (n))
#define SCTLR 0x00
#define TCR2 0x10
#define TTBR0 0x20
#define TTBR1 0x28
#define TCR 0x30
#define MAIR0 0x38
#define MAIR1 0x3c
#define FSR 0x58
#define FAR 0x60
#define FSYNR0 0x68
#define TLBIASID 0x610
#define TLBSYNC 0x7f0
#define TLBSTATUS 0x7f4
#define ON (1u | 1u<<1 | 1u<<2 | 1u<<5 | 1u<<8 | 1u<<12)
#define FSR_ALL 0xc00001feu
#define ADDRESS 0x0000fffffffff000ull
#define PAGE 4096ul
#define VALID (1u<<31)
#define MAX_CONTEXTS 8

struct hakios_smmu_context {
	bool allocated, active, quarantined, stream_written;
	unsigned sid, mask, match, bank;
	unsigned old_smr, old_s2cr;
	const char *who;
	uint64_t base, end;
	vm_offset_t tables, cb;
	unsigned long table_bytes, n2, n3;
};
static struct hakios_smmu_context contexts[MAX_CONTEXTS];
static vm_offset_t regs;
static unsigned reg_page, reg_pages, match_count, bank_count, first_bank;
/* A refused/partially claimed bank is never tried again this boot. */
static bool avoided[256], ours[256];

static void
say(const char *fmt, ...)
{
	char line[384]; va_list ap;
	va_start(ap, fmt); vsnprintf(line, sizeof(line), fmt, ap); va_end(ap);
	printf("%s", line);
	for (const char *p = line; *p; p++) loader_log_putc(*p);
}

static int
read32(vm_offset_t a, unsigned *v)
{
	return hakios_mmio_try_read32((const volatile void *)a, v);
}

static int
write32(vm_offset_t a, unsigned v)
{
	return hakios_mmio_try_write32((volatile void *)a, v);
}

static int
put(vm_offset_t a, unsigned v, unsigned mask)
{
	unsigned got; int error = write32(a, v);
	if (!error && mask) {
		error = read32(a, &got);
		if (!error && ((got ^ v) & mask)) error = EIO;
	}
	return error;
}

static int
init(void)
{
	unsigned scr, id0, id1;
	if (!regs) regs = ml_io_map(0x15000000, 0x100000);
	if (!regs || read32(regs, &scr) || read32(regs+0x20, &id0) || read32(regs+0x24, &id1)) return ENOTSUP;
	if (scr & (1u<<3)) return ENOTSUP; /* extended SID format unsupported */
	reg_page = id1 >> 31 ? 0x10000 : 0x1000;
	reg_pages = 2u << ((id1 >> 28) & 7);
	if (2 * reg_pages * reg_page != 0x100000) return ENOTSUP;
	match_count = id0 & 0xff;
	bank_count = id1 & 0xff;
	if (!bank_count || bank_count > reg_pages) return ENOTSUP;
	first_bank = (id1 >> 16) & 0xff;
	if (!first_bank) first_bank = 1;
	return 0;
}

static void
clean(struct hakios_smmu_context *c)
{
	for (vm_offset_t a = c->tables; a < c->tables+c->table_bytes; a += 64)
		__asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
	__asm__ volatile("dsb sy" ::: "memory");
}

static int
sync_context(struct hakios_smmu_context *c)
{
	int error = write32(c->cb+TLBIASID, c->bank);
	if (!error) error = write32(c->cb+TLBSYNC, 1);
	for (unsigned ms = 0; !error; ms++) {
		unsigned status;
		error = read32(c->cb+TLBSTATUS, &status);
		if (error || !(status & 1)) return error;
		if (ms == 500) return ETIMEDOUT;
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	return error;
}

static uint64_t *
pte(struct hakios_smmu_context *c, uint64_t iova)
{
	unsigned long table = 1+c->n2+((iova>>21)-(c->base>>21));
	return (uint64_t *)(c->tables+table*PAGE) + ((iova>>12)&511);
}

static int
allocate_tables(struct hakios_smmu_context *c)
{
	c->n2 = ((c->end-1)>>30)-(c->base>>30)+1;
	c->n3 = ((c->end-1)>>21)-(c->base>>21)+1;
	/* A bounded reservation, sufficient for the 64MiB Wi-Fi and audio pools. */
	if (c->n3 > 4096) return ENOMEM;
	c->table_bytes = (1+c->n2+c->n3)*PAGE;
	if (kmem_alloc(kernel_map, &c->tables, c->table_bytes,
	    KMA_KOBJECT|KMA_ZERO|KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) return ENOMEM;
	uint64_t *root = (uint64_t *)c->tables;
	for (unsigned n = 0; n < c->n2; n++)
		root[((c->base>>30)+n)&511] = kvtophys(c->tables+(1+n)*PAGE)|3;
	for (unsigned n = 0; n < c->n3; n++) {
		uint64_t addr = ((c->base>>21)+n)<<21;
		uint64_t *l2 = (uint64_t *)(c->tables+(1+(addr>>30)-(c->base>>30))*PAGE);
		l2[(addr>>21)&511] = kvtophys(c->tables+(1+c->n2+n)*PAGE)|3;
	}
	clean(c);
	return 0;
}

/* Caller holds shared lock; never return table memory to VM on uncertain stop. */
static void
release_locked(struct hakios_smmu_context *c)
{
	int error = 0;
	if (c->stream_written) {
		/* Invalidate our match before changing its route back to its original value. */
		error = put(regs+SMR(c->match), c->old_smr & ~VALID, ~0u);
		if (!error) error = put(regs+S2CR(c->match), c->old_s2cr, 0x300ff);
	}
	if (!error && c->cb) error = put(c->cb+SCTLR, 0, 1);
	if (!error && c->cb) error = sync_context(c);
	if (error) {
		c->quarantined = true;
		say("smmu: %s context %u quarantined: stop/sync error %d; tables retained\n", c->who, c->bank, error);
		return;
	}
	if (c->cb) avoided[c->bank]=false;
	if (c->tables) kmem_free(kernel_map, c->tables, c->table_bytes);
	bzero(c, sizeof(*c));
}

struct hakios_smmu_context *
hakios_smmu_claim(unsigned sid, unsigned mask, unsigned long long base,
    unsigned long size, const char *who)
{
	struct hakios_smmu_context *c = NULL;
	bool used[256] = {false}; unsigned match = ~0u, old_smr = 0, old_s2cr = 0;
	int error = EINVAL;
	if (sid > 0x7fff || mask > 0x7fff || !size || (base|size)&(PAGE-1) ||
	    base >= (1ull<<39) || size > (1ull<<39)-base) return NULL;
	lck_mtx_lock(&hakios_smmu_lock);
	if ((error = init())) goto fail;
	for (unsigned n = 0; n < match_count; n++) {
		unsigned smr, s2cr;
		if ((error = read32(regs+SMR(n), &smr)) || (error = read32(regs+S2CR(n), &s2cr))) goto fail;
		if (smr & VALID) {
			if (((s2cr>>16)&3) == 0) used[s2cr&255] = true;
			if (!(((smr ^ sid)&0x7fff) & ~((smr>>16)|mask))) { error=EBUSY; goto fail; }
		} else if (match == ~0u) { match=n; old_smr=smr; old_s2cr=s2cr; }
	}
	if (match == ~0u) { error=ENOSPC; goto fail; }
	for (unsigned n = 0; n < MAX_CONTEXTS; n++) if (!contexts[n].allocated) { c=&contexts[n]; break; }
	if (!c) { error=ENOSPC; goto fail; }
	c->allocated=true; c->sid=sid; c->mask=mask; c->base=base; c->end=base+size;
	c->who=who ? who : "anonymous"; c->match=match; c->old_smr=old_smr; c->old_s2cr=old_s2cr;
	if ((error=allocate_tables(c))) goto fail;
	for (unsigned bank = first_bank; bank < bank_count; bank++) {
		unsigned cbar, cba2r, ctl, tlb;
		vm_offset_t cb = regs+(reg_pages+bank)*reg_page;
		vm_offset_t gr1 = regs+reg_page;
		if (used[bank] || avoided[bank]) continue;
		if (read32(gr1+4*bank,&cbar) || read32(gr1+0x800+4*bank,&cba2r) ||
		    read32(cb+SCTLR,&ctl) || read32(cb+TLBSTATUS,&tlb)) { avoided[bank]=true; continue; }
		unsigned type=(cbar>>16)&3;
		if ((ctl&1) || (tlb&1) || !(ours[bank] || type==0 || (type==1 && (cbar&255)==255))) continue;
		avoided[bank]=true; /* even a partial/refused claim must not be retried */
		/* Original audio setup order, with each claim write guarded/read back. */
		if (put(cb+SCTLR,0,1) || put(gr1+0x800+4*bank,1,1) ||
		    put(gr1+4*bank,(1u<<16)|(3u<<8)|(15u<<12),0x3ffff)) continue;
		c->bank=bank; c->cb=cb; ours[bank]=true;
		break;
	}
	if (!c->cb) { error=EBUSY; goto fail; }
	uint64_t root=kvtophys(c->tables)|((uint64_t)c->bank<<48);
	if ((error=put(c->cb+TCR2,1u|(1u<<4)|(7u<<15),7)) ||
	    (error=put(c->cb+TCR,25u|(1u<<8)|(1u<<10)|(3u<<12)|(1u<<23),~0u)) ||
	    (error=put(c->cb+TTBR0,(unsigned)root,~0u)) ||
	    (error=put(c->cb+TTBR0+4,(unsigned)(root>>32),~0u)) ||
	    (error=put(c->cb+TTBR1,0,~0u)) || (error=put(c->cb+TTBR1+4,0,~0u)) ||
	    (error=put(c->cb+MAIR0,0xf404ff44,~0u)) || (error=put(c->cb+MAIR1,0,~0u)) ||
	    (error=write32(c->cb+FSR,FSR_ALL)) || (error=sync_context(c)) ||
	    (error=put(c->cb+SCTLR,ON,1))) goto fail;
	c->stream_written=true;
	if ((error=put(regs+S2CR(match),c->bank,0x300ff)) ||
	    (error=put(regs+SMR(match),VALID|(mask<<16)|sid,~0u))) goto fail;
	c->active=true;
	say("smmu: context %u for %#x (%s), IOVA %#llx + %#lx\n",c->bank,sid,c->who,base,size);
	lck_mtx_unlock(&hakios_smmu_lock);
	return c;
fail:
	say("smmu: %s claim %#x failed: %d\n",who ? who : "anonymous",sid,error);
	if (c) release_locked(c);
	lck_mtx_unlock(&hakios_smmu_lock);
	return NULL;
}

static bool
range_ok(struct hakios_smmu_context *c, uint64_t iova, unsigned long size)
{
	return c && c->active && !c->quarantined && size && !((iova|size)&(PAGE-1)) &&
	    iova >= c->base && iova < c->end && size <= c->end-iova;
}

int
hakios_smmu_map(struct hakios_smmu_context *c, unsigned long long iova,
    unsigned long long pa, unsigned long size, unsigned prot)
{
	int error=EINVAL;
	lck_mtx_lock(&hakios_smmu_lock);
	if (!range_ok(c,iova,size) || pa&(PAGE-1) || pa >= (1ull<<36) || size-1 > ((1ull<<36)-1)-pa ||
	    !prot || prot&~(HAKIOS_SMMU_READ|HAKIOS_SMMU_WRITE)) goto out;
	for (unsigned long n=0;n<size;n+=PAGE) if (*pte(c,iova+n)) { error=EEXIST; goto out; }
	/* SMMUv2 stage-1 AP[2:1]: RW=1, RO=3 (both privilege levels).
	 * Stage 1 cannot enforce write-only: reject rather than silently grant reads. */
	if (!(prot&HAKIOS_SMMU_READ)) { error=ENOTSUP; goto out; }
	uint64_t attrs=3ull|(1ull<<6)|(2ull<<8)|(1ull<<10)|(1ull<<11);
	if (!(prot&HAKIOS_SMMU_WRITE)) attrs|=1ull<<7;
	for (unsigned long n=0;n<size;n+=PAGE) *pte(c,iova+n)=(pa+n)|attrs;
	clean(c); error=sync_context(c);
	if (error) {
		for (unsigned long n=0;n<size;n+=PAGE) *pte(c,iova+n)=0;
		clean(c);
		if (sync_context(c)) { c->quarantined=true; (void)write32(c->cb+SCTLR,0); }
	}
	if (!error && c->sid==0x1801 && iova+size==c->end)
		say("sound: THE AUDIO STREAM GOES THROUGH CONTEXT %u of the SMMU, with our page tables\n",c->bank);
out:
	lck_mtx_unlock(&hakios_smmu_lock);
	return error;
}

int
hakios_smmu_unmap(struct hakios_smmu_context *c, unsigned long long iova, unsigned long size)
{
	int error=EINVAL; vm_offset_t backup=0;
	lck_mtx_lock(&hakios_smmu_lock);
	if (!range_ok(c,iova,size)) goto out;
	for (unsigned long n=0;n<size;n+=PAGE) if (!*pte(c,iova+n)) { error=ENOENT; goto out; }
	unsigned long bytes=(size/PAGE)*sizeof(uint64_t);
	if (kmem_alloc(kernel_map,&backup,bytes,KMA_KOBJECT|KMA_DATA,VM_KERN_MEMORY_IOKIT)!=KERN_SUCCESS) { error=ENOMEM; goto out; }
	for (unsigned long n=0;n<size;n+=PAGE) {
		((uint64_t *)backup)[n/PAGE]=*pte(c,iova+n); *pte(c,iova+n)=0;
	}
	clean(c); error=sync_context(c);
	if (error) {
		for (unsigned long n=0;n<size;n+=PAGE) *pte(c,iova+n)=((uint64_t *)backup)[n/PAGE];
		clean(c);
		if (sync_context(c)) { c->quarantined=true; (void)write32(c->cb+SCTLR,0); }
	}
	kmem_free(kernel_map,backup,bytes);
out:
	lck_mtx_unlock(&hakios_smmu_lock);
	return error;
}

void
hakios_smmu_faults(struct hakios_smmu_context *c)
{
	lck_mtx_lock(&hakios_smmu_lock);
	if (c && c->allocated && c->cb) {
		unsigned fsr=0,lo=0,hi=0,syn=0,global=0;
		int error=read32(c->cb+FSR,&fsr) || read32(c->cb+FAR,&lo) || read32(c->cb+FAR+4,&hi) ||
		    read32(c->cb+FSYNR0,&syn) || read32(regs+0x48,&global);
		say("smmu: %s context %u FSR %#x FAR %#llx FSYNR0 %#x SGFSR %#x read_error %d\n",
		    c->who,c->bank,fsr,((uint64_t)hi<<32)|lo,syn,global,error);
		if (!error && c->sid==0x1801 && !(fsr&FSR_ALL) && !global)
			say("sound: the SMMU has refused the DSP nothing (context %u: fsr %#x; sgfsr 0)\n",c->bank,fsr);
		if (!error) { (void)write32(c->cb+FSR,fsr&FSR_ALL); (void)write32(regs+0x48,global); }
	}
	lck_mtx_unlock(&hakios_smmu_lock);
}

void
hakios_smmu_release(struct hakios_smmu_context *c)
{
	if (!c) return;
	lck_mtx_lock(&hakios_smmu_lock);
	if (c->allocated) release_locked(c);
	lck_mtx_unlock(&hakios_smmu_lock);
}

int
hakios_smmu_bypass(unsigned sid)
{
	int error=EINVAL; unsigned free_match=~0u;
	if (sid>0x7fff) return EINVAL;
	lck_mtx_lock(&hakios_smmu_lock);
	if ((error=init())) goto out;
	for (unsigned n=0;n<MAX_CONTEXTS;n++) if (contexts[n].allocated &&
	    !((contexts[n].sid^sid)&~contexts[n].mask)) { error=EBUSY; goto out; }
	for (unsigned n=0;n<match_count;n++) {
		unsigned smr;
		if ((error=read32(regs+SMR(n),&smr))) goto out;
		if (smr&VALID) {
			if (!((smr^sid)&0x7fff&~(smr>>16))) { error=EBUSY; goto out; }
		} else if (free_match==~0u) free_match=n;
	}
	if (free_match==~0u) { error=ENOSPC; goto out; }
	unsigned old_route;
	if ((error=read32(regs+S2CR(free_match),&old_route))) goto out;
	error=put(regs+S2CR(free_match),1u<<16,0x300ff);
	if (!error) error=put(regs+SMR(free_match),VALID|sid,~0u);
	if (error) { (void)put(regs+SMR(free_match),0,~0u); (void)put(regs+S2CR(free_match),old_route,0x300ff); }
out:
	lck_mtx_unlock(&hakios_smmu_lock);
	return error;
}
