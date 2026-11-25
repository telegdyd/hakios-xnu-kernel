/* For the host tests: bzero(). */
#include <string.h>
#define bzero(p, n) memset((p), 0, (n))
