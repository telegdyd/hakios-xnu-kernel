/*
 * More than one processor on generic (non-Apple) ARMv8 boards: see psci_smp.c.
 */
#ifndef _ARM64_PSCI_SMP_H_
#define _ARM64_PSCI_SMP_H_

#include <mach/mach_types.h>
#include <sys/cdefs.h>

__BEGIN_DECLS

/* How far a processor other than the boot processor has come (psci_smp_cpu_reached()). */
#define PSCI_SMP_CPU_ASKED      1       /* the firmware has agreed to start it */
#define PSCI_SMP_CPU_IN_KERNEL  2       /* it runs C code on the kernel's page tables (arm_init_cpu()) */
#define PSCI_SMP_CPU_RUNNING    3       /* it has joined the scheduler (psci_smp_cpu_running()) */
#define PSCI_SMP_CPU_OFF        4       /* it has switched itself off, or is about to */

extern void             psci_smp_init(void);
extern int              psci_smp_boot_cpu_number(void);   /* the boot processor's number (master_cpu is never set) */
extern kern_return_t    psci_smp_start_cpu(int cpu);
extern void             psci_smp_cpu_reached(uint32_t stage);
extern void             psci_smp_cpu_running(void);
extern void             psci_smp_cpu_off(void) __dead2;
extern void             psci_smp_cpu_unplug(void) __dead2;
/* psci_smp.c, for psci_idle.c */
extern int64_t          psci_smp_call(uint64_t function, uint64_t arg1, uint64_t arg2, uint64_t arg3);
extern uint32_t         psci_smp_stage_of(int cpu);
extern uint64_t         psci_smp_loader_entry_pa(void);
extern void             psci_smp_clean_for_reset_all(void);

/* while the display is off, the processors of screen_off_cores are switched off */
extern void             psci_smp_display(bool on);
extern void             psci_smp_set_screen_off_cores(uint64_t cores);
extern uint64_t         psci_smp_cores_up(void);
/* ... and for the drivers' loops: is it off; a pause of on_ms, or off_ms while it is off (STANDBY-PLAN.md) */
extern bool             hakios_screen_off(void);
extern void             hakios_screen_pause(uint32_t on_ms, uint32_t off_ms);
extern void             hakios_screen_pause_leeway(uint32_t on_ms, uint32_t off_ms, uint32_t leeway_ms);
extern wait_result_t    hakios_assert_wait_net(event_t event, uint32_t ms);
extern size_t           hakios_wakes_text(char *buf, size_t size);
extern size_t           hakios_idle_text(char *buf, size_t size);

/* idle states: PSCI CPU_SUSPEND from the processor's cpu_idle_notify (POWER-COLLAPSE-PLAN.md) */
extern void             psci_idle_init(void);
extern void             psci_idle_notify(void *cpu_id, boolean_t enter, uint64_t *new_timeout_ticks);     /* a processor_idle_t */
/* the IPI census (gic_v3_irq.c): an interrupt acknowledged (kind 1 the timer, 2 the signal, 0 any other); a signal sent */
extern void             psci_idle_irq(uint32_t intid, int kind);
extern void             psci_ipi_sent(uint32_t phys_id);

extern void             psci_smp_stop_others(void);
extern void             psci_smp_stop_here(void);

/* osfmk/arm64/gic_v3_irq.c */
extern void             gic_v3_irq_signal(uint32_t phys_id);
extern void             gic_v3_irq_signal_stop(uint32_t phys_id);
extern void             gic_v3_irq_quiesce(void);
extern void             gic_v3_irq_cpu_shutdown(void);
extern void             gic_v3_irq_cpu_unplug(void);
extern void             gic_v3_irq_cpu_resume(void);

__END_DECLS

#endif /* _ARM64_PSCI_SMP_H_ */
