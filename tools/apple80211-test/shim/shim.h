/* The few kernel interfaces drivers/wifi/hakios.apple80211.c uses, on the host (tools/apple80211-test). */
#ifndef APPLE80211_TEST_SHIM_H
#define APPLE80211_TEST_SHIM_H
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#ifndef EAUTH
#define EAUTH 80                        /* (Darwin's; Linux has none) */
#endif
typedef uint64_t user_addr_t;
typedef int errno_t;
#define USER_ADDR_NULL ((user_addr_t)0)
#define Z_WAITOK 0
#define Z_ZERO 1
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define __DECONST(type, var) ((type)(uintptr_t)(const void *)(var))
#define LCK_GRP_DECLARE(name, string) int name __attribute__((unused))
#define LCK_MTX_DECLARE(name, group) pthread_mutex_t name = PTHREAD_MUTEX_INITIALIZER
#define lck_mtx_lock(m) pthread_mutex_lock(m)
#define lck_mtx_unlock(m) pthread_mutex_unlock(m)
#define KEV_VENDOR_APPLE 1
#define KEV_IEEE80211_CLASS 6

static inline void *
kalloc_data(size_t size, int flags)
{
	return flags & Z_ZERO ? calloc(1, size) : malloc(size);
}

static inline void
kfree_data(void *p, size_t size)
{
	(void)size;
	free(p);
}

static inline int
copyin(user_addr_t from, void *to, size_t length)
{
	memcpy(to, (const void *)(uintptr_t)from, length);
	return 0;
}

static inline int
copyout(const void *from, user_addr_t to, size_t length)
{
	memcpy((void *)(uintptr_t)to, from, length);
	return 0;
}

static inline int
memset_s(void *p, size_t most, int c, size_t n)
{
	memset(p, c, n < most ? n : most);
	return 0;
}

static inline char *
strnstr(const char *s, const char *find, size_t most)
{
	size_t n = strlen(find);

	for (size_t i = 0; i + n <= most && s[i] != 0; i++) {
		if (strncmp(s + i, find, n) == 0) {
			return (char *)(uintptr_t)(s + i);
		}
	}
	return NULL;
}

struct kev_d_vectors {
	uint32_t data_length;
	void *data_ptr;
};
struct kev_msg {
	uint32_t vendor_code, kev_class, kev_subclass, event_code;
	struct kev_d_vectors dv[5];
};
int kev_post_msg(struct kev_msg *message);
void proc_selfname(char *name, int size);
#endif
