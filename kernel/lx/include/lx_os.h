/*
 * What the driver's object asks of the kernel it is linked into. The kernel describes that
 * itself (bsd/sys/board/lx.h of the kernel tree, which kernel/lx/build.sh names here): there
 * is one description, and both sides are compiled against it.
 * C4's PCIe, DMA pool, QRTR link and stage calls are declared there too. The
 * included C1/C2/C6 headers travel beside board/lx.h in the kernel tree;
 * do not reproduce their layouts in the LinuxKPI side.
 */
#ifndef _LX_OS_H_
#define _LX_OS_H_

#include LX_HAKIOS_LX_H

#endif
