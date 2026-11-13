/* C9: the GIC's ITS (SM8450 under Gunyah; drivers/core/hakios.its.c). Message-signalled interrupts as LPIs. */
#ifndef _SYS_HAKIOS_ITS_H_
#define _SYS_HAKIOS_ITS_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A device's `events` (1..256, a power of two) events become LPIs, each delivered to processor 0. The first call switches
 * the ITS and processor 0's LPIs on (once per boot: they cannot be switched off again). `handler` is called in interrupt
 * context, the event's number given, once per message the device writes; it must not block. *doorbell is the physical
 * address the device writes the event number to (32 bits, GITS_TRANSLATER). Thread context. 0, or a positive errno:
 * ENXIO when the GIC was left with LPIs on by an earlier kernel (the caller then does without).
 */
int hakios_its_device(unsigned device_id, unsigned events, void (*handler)(void *context, unsigned event), void *context,
    unsigned long long *doorbell);
/* The device's events unmapped; returns when no handler of it runs. Its memory is kept for a later hakios_its_device(). */
void hakios_its_device_off(unsigned device_id);
/* How many of that event have arrived since the boot. */
unsigned long long hakios_its_count(unsigned device_id, unsigned event);

#ifdef __cplusplus
}
#endif
#endif
