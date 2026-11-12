/* C3: shared stage-1 contexts, SM8450 apps SMMU. */
#ifndef _SYS_HAKIOS_SMMU_H_
#define _SYS_HAKIOS_SMMU_H_

struct hakios_smmu_context;
#define HAKIOS_SMMU_READ  1u /* device may read */
#define HAKIOS_SMMU_WRITE 2u /* device may write */
#define HAKIOS_WIFI_IOVA_BASE 0xa0000000ull
#define HAKIOS_WIFI_POOL_SIZE (64ul * 1024 * 1024)

/* NULL on failure; who is a borrowed static diagnostic string. mask is the
 * ARM SMR ignored-SID-bit mask (0 = exact SID), NOT a DMA address mask.
 * Range is [base,base+size), no overflow; page-aligned, page size 4096.
 * Serialized by existing hakios_smmu_lock. Banks start at max(NUMS2CB,1),
 * claimed by guarded write/readback. Never restore CBAR/CBA2R, even on failure.
 */
struct hakios_smmu_context *hakios_smmu_claim(unsigned sid, unsigned mask,
    unsigned long long iova_base, unsigned long iova_size, const char *who);
/* 0 or positive errno. Whole 4-KiB pages, wholly inside claimed range; map
 * rejects overlaps. prot is nonzero READ|WRITE subset, normal DMA memory.
 * Publish/remove PTEs with cache maintenance/barriers and TLB sync <=500 ms.
 * Failure must leave the original mappings intact. Caller quiesces DMA first.
 */
int hakios_smmu_map(struct hakios_smmu_context *ctx, unsigned long long iova,
    unsigned long long pa, unsigned long size, unsigned prot);
int hakios_smmu_unmap(struct hakios_smmu_context *ctx, unsigned long long iova, unsigned long size);
void hakios_smmu_faults(struct hakios_smmu_context *ctx); /* print and clear */
/* NULL allowed; disable SCTLR + bounded TLB sync; no CBAR/CBA2R restore.
 * Normal thread context under the shared lock, NOT a panic-time operation.
 * Panic path stops hardware DMA using C2; normal teardown releases afterward.
 */
void hakios_smmu_release(struct hakios_smmu_context *ctx);
/* Exact SID S2CR bypass for diagnosis; no active owned context may overlap.
 * No high physical address becomes a usable 32-bit IOVA by enabling bypass.
 */
int hakios_smmu_bypass(unsigned sid);

#endif
