/* For the host tests: kalloc_data()/kfree_data() on calloc/free. */
#ifndef QRTR_TEST_SHIM_KALLOC_H
#define QRTR_TEST_SHIM_KALLOC_H

#include <stdlib.h>

#define Z_WAITOK 1
#define Z_ZERO 2
#define kalloc_data(size, flags) calloc(1, (size))
#define kfree_data(pointer, size) free(pointer)

#endif
