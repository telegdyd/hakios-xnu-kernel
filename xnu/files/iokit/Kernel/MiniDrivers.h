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

/* drivers/gpu/hakios.backlight.cpp: the panel's brightness (backlight=SOC), started under the display */
void hakios_backlight_start(IOService * display);
void hakios_backlight_display(bool on);         /* RequestPowerChange: the panel to sleep, or woken */
int hakios_dsi_dcs_write(const uint8_t * buf, uint32_t len, bool lowPower);
/* MiniIOMobileFramebuffer.cpp: the panel asleep (no TE): no flips until it is woken */
void hakios_fb_panel_asleep(bool asleep);

#endif /* _IOKIT_MINIDRIVERS_H */
