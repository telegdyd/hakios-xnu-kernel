/* For the host tests: a thread is told apart by its pthread. */
#ifndef QRTR_TEST_SHIM_THREAD_H
#define QRTR_TEST_SHIM_THREAD_H

#include <pthread.h>

typedef void *thread_t;
static inline thread_t
current_thread(void)
{
	return (thread_t)(unsigned long)pthread_self();
}

#endif
