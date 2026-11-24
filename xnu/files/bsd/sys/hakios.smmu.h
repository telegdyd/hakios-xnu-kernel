/* C3: shared stage-1 contexts, SM8450 apps SMMU. */
#ifndef _SYS_HAKIOS_SMMU_H_
#define _SYS_HAKIOS_SMMU_H_

struct hakios_smmu_context;
#define HAKIOS_SMMU_READ  1u /* device may read */
#define HAKIOS_SMMU_WRITE 2u /* device may write */
#define HAKIOS_SMMU_MMIO  4u /* with READ: device memory (nGnRE), for an MSI doorbell */
#define HAKIOS_SMMU_COHERENT 8u /* write-back cacheable, inner shareable: for a dma-coherent master (camera ICP) */
#define HAKIOS_WIFI_IOVA_BASE 0xa0000000ull
#define HAKIOS_WIFI_POOL_SIZE (64ul * 1024 * 1024)
/* the ITS doorbell page, just past the pool; the context claims the pool and 2 MiB more for it */
#define HAKIOS_WIFI_DOORBELL_IOVA (HAKIOS_WIFI_IOVA_BASE + HAKIOS_WIFI_POOL_SIZE)
#define HAKIOS_WIFI_CLAIM_SIZE (HAKIOS_WIFI_POOL_SIZE + 0x200000ul)

/* NULL on failure; who is a borrowed static diagnostic string. mask is the
 * ARM SMR ignored-SID-bit mask (0 = exact SID), NOT a DMA address mask.
 * Range is [base,base+size), no overflow; page-aligned, page size 4096.
 * Serialized by existing hakios_smmu_lock. Banks start at max(NUMS2CB,1),
 * claimed by guarded write/readback. Never restore CBAR/CBA2R, even on failure.
 */
struct hakios_smmu_context *hakios_smmu_claim(unsigned sid, unsigned mask,
    unsigned long long iova_base, unsigned long iova_size, const char *who);
/* Share dedicated contiguous table pages before publishing the context and
 * reclaim them after stopping it. A failed ownership change retains the pages. */
typedef bool (*hakios_smmu_table_ownership_fn)(unsigned long long pa, unsigned long size, bool grant);
struct hakios_smmu_context *hakios_smmu_claim_with_table_ownership(unsigned sid, unsigned mask,
    unsigned long long iova_base, unsigned long iova_size, const char *who,
    hakios_smmu_table_ownership_fn ownership);
/* Route another stream (its own SMR, the same bank and page tables) to a claimed context, as stock routes every
 * camera ICP group to one context. Released with the context. 0 or errno (ENOSPC: no free stream match). */
int hakios_smmu_add_stream(struct hakios_smmu_context *ctx, unsigned sid, unsigned mask);
/* 0 or positive errno. Whole 4-KiB pages, wholly inside claimed range; map
 * rejects overlaps. prot is nonzero READ|WRITE subset, normal DMA memory; with MMIO, device memory.
 * Publish/remove PTEs with cache maintenance/barriers and TLB sync <=500 ms.
 * Failure must leave the original mappings intact. Caller quiesces DMA first.
 */
int hakios_smmu_map(struct hakios_smmu_context *ctx, unsigned long long iova,
    unsigned long long pa, unsigned long size, unsigned prot);
/* Same contract for a wired, page-aligned kernel buffer of any physical layout; READ|WRITE only. */
int hakios_smmu_map_kva(struct hakios_smmu_context *ctx, unsigned long long iova,
    unsigned long kva, unsigned long size, unsigned prot);
int hakios_smmu_unmap(struct hakios_smmu_context *ctx, unsigned long long iova, unsigned long size);
bool hakios_smmu_faults(struct hakios_smmu_context *ctx); /* print and clear, true if no faults */
/* A claimed context's SCTLR (M must stay set) and ACTLR, written and read back: to match another driver's settings
 * (the ICP probe copies stock's camera context). 0 or errno. */
int hakios_smmu_context_attributes(struct hakios_smmu_context *ctx, unsigned sctlr, unsigned actlr);
/* Read FSR, FAR and FSYNR0 without printing or clearing (lock-free: for a diagnostic's timed window). 0 or errno. */
int hakios_smmu_fault_state(struct hakios_smmu_context *ctx, unsigned *fsr, unsigned long long *far, unsigned *fsynr0);
/* NULL allowed; disable SCTLR + bounded TLB sync; no CBAR/CBA2R restore.
 * Returns 0 on release, or positive errno on quarantine.
 * Normal thread context under the shared lock, NOT a panic-time operation.
 * Panic path stops hardware DMA using C2; normal teardown releases afterward.
 */
int hakios_smmu_release(struct hakios_smmu_context *ctx);
/* Exact SID S2CR bypass for diagnosis; no active owned context may overlap.
 * No high physical address becomes a usable 32-bit IOVA by enabling bypass.
 */
int hakios_smmu_bypass(unsigned sid);

#endif
