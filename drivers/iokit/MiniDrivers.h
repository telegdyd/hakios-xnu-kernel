/*
 * Stand-ins for Apple's closed kernel extensions: just enough of each for the stock Darwin
 * user space to work on a machine without Apple hardware. The platform expert
 * (GenericARMPE) starts them once the registry is up.
 *
 *   MiniIOSurface.cpp              IOSurface: pixel buffers shared between processes
 *   MiniIOMobileFramebuffer.cpp    IOMobileFramebuffer: the display (the boot framebuffer)
 *   MiniAppleKeyStore.cpp          AppleKeyStore: the keybag of a device without a passcode
 *   MiniIONetworkStack.cpp         IONetworkStack: what configd's interface namer waits for
 */
#ifndef _IOKIT_MINIDRIVERS_H
#define _IOKIT_MINIDRIVERS_H

#include <IOKit/IOService.h>

void MiniIOSurfaceStart(IOService * provider);
void MiniIOMobileFramebufferStart(IOService * provider);
void MiniAppleKeyStoreStart(void);
void MiniIONetworkStackStart(void);
/* arch/arm64/kernel/psci_smp.c: RequestPowerChange: the processors of screen_off_cores off while the display is off */
extern "C" void psci_smp_display(bool on);

#endif /* _IOKIT_MINIDRIVERS_H */
