#include "../fakekernel.h"
#ifndef FAKE_STRNSTR
#define FAKE_STRNSTR
/* strnstr(): the first of needle in the first n bytes of haystack (BSD's; glibc has none) */
static inline char *
fake_strnstr(const char *haystack, const char *needle, size_t n)
{
	size_t length = strlen(needle);

	if (length == 0) {
		return (char *)haystack;
	}
	for (size_t i = 0; i + length <= n && haystack[i] != 0; i++) {
		if (strncmp(haystack + i, needle, length) == 0) {
			return (char *)haystack + i;
		}
	}
	return NULL;
}
#define strnstr fake_strnstr
#endif
