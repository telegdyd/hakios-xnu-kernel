#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#define nitems(a) (sizeof(a) / sizeof((a)[0]))
struct timer_list { unsigned long expires; void (*function_415)(struct timer_list *); };
struct work_struct { void (*func)(struct work_struct *); };
struct delayed_work { struct work_struct work; };
struct workqueue_struct { int unused; };
struct callout { int unused; };
typedef void (*work_func_t)(struct work_struct *);
#define INIT_WORK(w, f) ((w)->func = (f))
#define jiffies (lx_jiffies())
static unsigned long long clock_ns;
static bool locked;
static unsigned soc = 8450;
static int scenario, sleeps, notifications, callbacks, dispatches;
static void *lx_os_thread_self(void) { return (void *)1; }
static jmp_buf finished;
static struct timer_list timers[3];
static struct delayed_work works[2];
static struct workqueue_struct queue;
static unsigned long long lx_os_nanoseconds(void) { return clock_ns; }
static unsigned lx_os_option(const char *name) { assert(!strcmp(name, "wifi_soc")); return soc; }
static void lx_os_lock(void) { assert(!locked); locked = true; }
static void lx_os_unlock(void) { assert(locked); locked = false; }
static void lx_tell(const char *fmt, ...) { (void)fmt; }
static void lx_os_wakeup(const void *address);
static int lx_os_sleep(const void *address, unsigned long long ns);
static bool linux_queue_work_on(int cpu, struct workqueue_struct *q, struct work_struct *w)
{
    assert(!locked && q == &queue);
    dispatches++;
    if (w->func) w->func(w);
    return true;
}
/* PRODUCTION */
static void lx_os_wakeup(const void *address)
{
    assert(locked && address == &timer_event);
    notifications++;
}
static void callback(struct timer_list *timer)
{
    assert(!locked);
    callbacks++;
    if (scenario == 2 && callbacks == 1) mod_timer(timer, jiffies);
    if (scenario == 6) mod_timer(timer, jiffies + 10);
}
static int lx_os_sleep(const void *address, unsigned long long ns)
{
    assert(locked && address == &timer_event);
    int step = sleeps++;
    if (scenario == 7) {
        assert(ns == 0 && timer_waiters == 1);
        lx_os_unlock(); mod_timer(&timers[0], jiffies + 10); lx_os_lock();
        running_timer[1] = NULL; timer_owner[1] = NULL;
        return 0;
    }
    if (scenario == 8) {
        assert(ns == 0 && timer_waiters == 1);
        lx_os_unlock(); linux_queue_work_on(0, &queue, &works[0].work); lx_os_lock();
        dispatching_work = NULL; dispatch_owner = NULL;
        return 0;
    }
    if (scenario == 0) { assert(ns == 0); longjmp(finished, 1); }
    if (scenario == 1) {
        if (step == 0) { assert(ns == 1750000 && !dispatches && !callbacks); }
        if (step == 1) { assert(ns == 1000000 && dispatches == 1 && !callbacks); }
        if (step == 2) { assert(ns == 0 && dispatches == 1 && callbacks == 1); longjmp(finished, 1); }
    }
    if (scenario == 2 && step == 0) {
        assert(ns == 0);
        lx_os_unlock();
        int before = notifications;
        assert(mod_timer(&timers[0], jiffies + 5) == 0);
        assert(notifications == before + 1);
        lx_os_lock();
        return 0;
    }
    if (scenario == 2 && step == 1) assert(ns == 4750000);
    if (scenario == 2 && step == 2) {
        assert(ns == 0 && callbacks == 2); longjmp(finished, 1);
    }
    if ((scenario == 3 || scenario == 5) && step == 0) {
        assert(ns == 1750000);
        lx_os_unlock();
        int before = notifications;
        if (scenario == 3) assert(timer_delete_sync(&timers[0]) == 1);
        else assert(mod_timer(&timers[0], 108) == 1);
        assert(notifications == before + 1);
        lx_os_lock();
        return 0;
    }
    if (scenario == 3 && step == 1) assert(ns == 9750000);
    if (scenario == 3 && step == 2) {
        assert(ns == 0 && callbacks == 0 && dispatches == 1); longjmp(finished, 1);
    }
    if (scenario == 4) {
        assert(ns == 0 && callbacks == 3 && dispatches == 2); longjmp(finished, 1);
    }
    if (scenario == 5 && step == 1) assert(ns == 7750000);
    if (scenario == 5 && step == 2) {
        assert(ns == 0 && callbacks == 1); longjmp(finished, 1);
    }
    assert(step < 4 && ns > 0);
    clock_ns += ns;
    return 1;
}
int main(void)
{
    clock_ns = 123456789;
    assert(jiffies == 123);
    clock_ns += 5000000000ull;
    assert(jiffies == 5123); /* no ticker has run */
    for (scenario = 0; scenario < 6; scenario++) {
        memset(armed, 0, sizeof(armed)); memset(later, 0, sizeof(later));
        deferred_timer = NULL;
        memset(timers, 0, sizeof(timers)); memset(works, 0, sizeof(works));
        locked = false; sleeps = notifications = callbacks = dispatches = 0;
        clock_ns = 100250000;
        for (unsigned i = 0; i < nitems(timers); i++) timers[i].function_415 = callback;
        if (scenario == 1) {
            mod_timer(&timers[0], 103);
            assert(linux_queue_delayed_work_on(0, &queue, &works[0], 2));
            assert(!linux_queue_delayed_work_on(0, &queue, &works[0], 1));
        }
        if (scenario == 3 || scenario == 5) mod_timer(&timers[0], 102);
        if (scenario == 3) {
            assert(linux_queue_delayed_work_on(0, &queue, &works[0], 1));
            int before = notifications;
            assert(linux_cancel_delayed_work(&works[0]));
            assert(notifications == before + 1);
            assert(!linux_cancel_delayed_work(&works[0]));
            assert(linux_queue_delayed_work_on(0, &queue, &works[0], 10));
        }
        if (scenario == 4) {
            for (unsigned i = 0; i < nitems(timers); i++) mod_timer(&timers[i], 99);
            for (unsigned i = 0; i < nitems(works); i++)
                assert(linux_queue_delayed_work_on(0, &queue, &works[i], 1));
            clock_ns += 2000000;
        }
        if (setjmp(finished) == 0) ticker(NULL);
        locked = false;
        printf("PASS timer scenario %d\n", scenario);
    }
    memset(armed, 0, sizeof(armed)); memset(later, 0, sizeof(later));
    clock_ns = 100250000; callbacks = notifications = 0;
    lx_timer_defer(&timers[0]);
    int before = notifications;
    mod_timer(&timers[0], 105);
    assert(notifications == before);
    /* A pending monitor timer must not put any deadline on the ticker. */
    scenario = 0;
    if (setjmp(finished) == 0) ticker(NULL);
    locked = false; scenario = 6;
    clock_ns = 104999999; lx_timer_poll(); assert(callbacks == 0);
    clock_ns = 105000000; lx_timer_poll(); assert(callbacks == 1);
    assert(timers[0].expires == 115 && notifications == before);
    clock_ns = 115000000; lx_timer_poll(); assert(callbacks == 2);
    assert(timer_delete_sync(&timers[0]) == 1);
    clock_ns = 200000000; lx_timer_poll(); assert(callbacks == 2);
    puts("PASS deferred monitor: no ticker deadline, due-only worker service, rearm and cancellation");
    soc = 0; deferred_timer = NULL; before = notifications;
    lx_timer_defer(&timers[0]);
    assert(deferred_timer == NULL && notifications == before);
    puts("PASS AHB radio keeps its normal monitor timer");
    memset(armed, 0, sizeof(armed));
    scenario = 7; sleeps = 0;
    running_timer[1] = &timers[0]; timer_owner[1] = (void *)2;
    assert(timer_delete_sync(&timers[0]) == 1 && sleeps == 1 && timer_waiters == 0);
    for (unsigned i = 0; i < nitems(armed); i++) assert(armed[i] != &timers[0]);
    running_timer[0] = &timers[0]; timer_owner[0] = (void *)1;
    assert(timer_delete_sync(&timers[0]) == 0 && sleeps == 1);
    running_timer[0] = NULL; timer_owner[0] = NULL;
    scenario = 8; sleeps = 0; dispatches = 0;
    dispatching_work = &works[0]; dispatch_owner = (void *)2;
    assert(!linux_cancel_delayed_work(&works[0]) && sleeps == 1 && dispatches == 1 && timer_waiters == 0);
    puts("PASS cancellation waits for running monitor/rearm and delayed-work dispatch; self cancellation does not deadlock");
    puts("PASS clock advances without ticker; all timer scenarios passed");
}
