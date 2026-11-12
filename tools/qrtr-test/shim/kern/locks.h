/*
 * For the host tests: what XNU's <kern/locks.h> gives drivers/wifi/hakios.qrtr.c, on pthreads.
 * The mutexes check for misuse (a lock taken twice, or released by a thread that does not hold it,
 * aborts); lck_mtx_sleep()/thread_wakeup() are one condition variable for every event.
 */
#ifndef QRTR_TEST_SHIM_LOCKS_H
#define QRTR_TEST_SHIM_LOCKS_H

#include <pthread.h>
#include <stdlib.h>

typedef struct {
	pthread_mutex_t m;
} lck_mtx_t;
typedef int lck_grp_t;
typedef void *event_t;
typedef int wait_result_t;
typedef int lck_sleep_action_t;
typedef int wait_interrupt_t;
#define LCK_SLEEP_DEFAULT 0
#define THREAD_UNINT 0

#define LCK_GRP_DECLARE(var, name) lck_grp_t var __attribute__((unused)) = 0
#define LCK_MTX_DECLARE(var, group) lck_mtx_t var = { PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP }

static pthread_cond_t shim_event = PTHREAD_COND_INITIALIZER;

static inline void
lck_mtx_lock(lck_mtx_t *l)
{
	if (pthread_mutex_lock(&l->m) != 0) {
		abort();
	}
}

static inline void
lck_mtx_unlock(lck_mtx_t *l)
{
	if (pthread_mutex_unlock(&l->m) != 0) {
		abort();
	}
}

static inline wait_result_t
lck_mtx_sleep(lck_mtx_t *l, lck_sleep_action_t action, event_t event, wait_interrupt_t interruptible)
{
	(void)action;
	(void)event;
	(void)interruptible;
	if (pthread_cond_wait(&shim_event, &l->m) != 0) {
		abort();
	}
	return 0;
}

static inline void
thread_wakeup(event_t event)
{
	(void)event;
	pthread_cond_broadcast(&shim_event);
}

#endif
