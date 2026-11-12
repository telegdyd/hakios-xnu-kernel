#ifndef _SYS_HAKIOS_MMIO_H_
#define _SYS_HAKIOS_MMIO_H_

/* Guarded 32-bit accesses to already-mapped, naturally aligned device memory.
 * Positive errno; failed reads leave *value unchanged. No locks or allocation.
 * Only a synchronous external abort at the exact access instruction is recovered;
 * asynchronous SError, translation faults and unrelated aborts remain fatal.
 * Implemented with the narrow recovery hook in osfmk/arm64/sleh.c.
 */
#ifdef __cplusplus
extern "C" {
#endif
int hakios_mmio_try_read32(const volatile void *address, unsigned *value);
int hakios_mmio_try_write32(volatile void *address, unsigned value);
#ifdef __cplusplus
}
#endif
#endif
