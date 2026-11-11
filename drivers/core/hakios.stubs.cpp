/*
 * The IOKit half of drivers/core/hakios.stubs.c: what the rest of the kernel calls in a driver that is left out of it,
 * for the calls that are C++ (docs/drivers-config.md).
 */
#include "MiniIOSurface.h"
#include "MiniDrivers.h"

#ifndef HAKIOS_BACKLIGHT
/* the display starts the panel's brightness: without the driver the panel stays as the boot loader left it */
void hakios_backlight_start(__unused IOService * display) {}
int hakios_dsi_dcs_write(__unused const uint8_t * buf, __unused uint32_t len, __unused bool lowPower) { return -1; }
#endif

#ifndef HAKIOS_ADRENO
/* the display's driver asks whether the GPU is done with a surface before it shows it: with no GPU nothing draws into one */
bool
MiniIOSurfaceDrawn(__unused MiniIOSurface * surface, __unused uint64_t milliseconds)
{
	return true;
}
void MiniIOSurfaceBeginRead(__unused MiniIOSurface * surface) {}
void MiniIOSurfaceEndRead(__unused MiniIOSurface * surface) {}
#endif
