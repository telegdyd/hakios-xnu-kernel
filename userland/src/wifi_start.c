/* Start the last saved Wi-Fi through MobileWiFi/wifid, never kernel boot words.
 * The flash-boot gate runs this once in System after publishing the radio.
 * No names, addresses or passwords are logged. The password remains in iOS's
 * keychain; its transient CFString is released after handing it to MobileWiFi.
 * Private API signatures: ios-reversed-headers/MobileWiFi (Cykey), as used by
 * the phone-tested diagnostic. Resolve at runtime because the public SDK omits them.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

#define LOAD(name, result, args) \
	result (*name) args = dlsym(library, #name); \
	if (name == NULL) { puts("wifi_start: native Wi-Fi API unavailable: " #name); return 1; }

static int finished, association_error;

static void
associated(CFTypeRef device, CFTypeRef network, CFDictionaryRef info, int error, const void *context)
{
	(void)device; (void)network; (void)info; (void)context;
	association_error = error;
	finished = 1;
}

static Boolean
disabled(CFDictionaryRef record)
{
	const CFStringRef keys[] = { CFSTR("AutoJoinDisabled"), CFSTR("Disabled") };
	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		CFTypeRef value = CFDictionaryGetValue(record, keys[i]);
		if (value && CFGetTypeID(value) == CFBooleanGetTypeID() && CFBooleanGetValue(value)) return true;
	}
	CFTypeRef enabled = CFDictionaryGetValue(record, CFSTR("AutoJoin"));
	return enabled && CFGetTypeID(enabled) == CFBooleanGetTypeID() && !CFBooleanGetValue(enabled);
}

int
main(void)
{
	setvbuf(stdout, NULL, _IOLBF, 0);
	void *library = dlopen("/System/Library/PrivateFrameworks/MobileWiFi.framework/MobileWiFi", RTLD_NOW);
	if (!library) { puts("wifi_start: MobileWiFi unavailable"); return 1; }
	LOAD(WiFiManagerClientCreate, CFTypeRef, (CFAllocatorRef, int));
	LOAD(WiFiManagerClientCopyDevices, CFArrayRef, (CFTypeRef));
	LOAD(WiFiManagerClientCopyNetworks, CFArrayRef, (CFTypeRef));
	LOAD(WiFiManagerClientScheduleWithRunLoop, void, (CFTypeRef, CFRunLoopRef, CFStringRef));
	LOAD(WiFiManagerClientUnscheduleFromRunLoop, void, (CFTypeRef));
	LOAD(WiFiDeviceClientGetPower, int, (CFTypeRef));
	LOAD(WiFiDeviceClientCopyCurrentNetwork, CFTypeRef, (CFTypeRef));
	LOAD(WiFiNetworkCopyRecord, CFDictionaryRef, (CFTypeRef));
	LOAD(WiFiNetworkCopyPassword, CFStringRef, (CFTypeRef));
	LOAD(WiFiNetworkRequiresPassword, Boolean, (CFTypeRef));
	LOAD(WiFiNetworkSetPassword, void, (CFTypeRef, CFStringRef));
	LOAD(WiFiDeviceClientAssociateAsync, int, (CFTypeRef, CFTypeRef,
	    void (*)(CFTypeRef, CFTypeRef, CFDictionaryRef, int, const void *), CFDictionaryRef));
	CFTypeRef manager = WiFiManagerClientCreate(NULL, 0);
	if (!manager) { puts("wifi_start: no native manager"); return 1; }
	CFArrayRef devices = NULL, networks = NULL;
	/* wifid and the kernel radio start independently; wait for both, without
	 * blocking launchd or repeatedly changing the user's power preference. */
	for (unsigned attempt = 0; attempt < 120; attempt++) {
		devices = WiFiManagerClientCopyDevices(manager);
		networks = WiFiManagerClientCopyNetworks(manager);
		if (devices && CFArrayGetCount(devices) && networks && CFArrayGetCount(networks)) break;
		if (devices) CFRelease(devices);
		if (networks) CFRelease(networks);
		devices = networks = NULL;
		usleep(500000);
	}
	int result = 0;
	if (!devices || !networks) { puts("wifi_start: no saved network or ready radio"); goto out; }
	CFTypeRef device = CFArrayGetValueAtIndex(devices, 0);
	if (!WiFiDeviceClientGetPower(device)) { puts("wifi_start: Wi-Fi is off"); goto out; }
	CFTypeRef current = WiFiDeviceClientCopyCurrentNetwork(device);
	if (current) { CFRelease(current); puts("wifi_start: already joined"); goto out; }
	/* Every eligible saved network, newest lastJoined first, until one joins: the clock
	 * restarts at 1970 on each boot, so lastJoined only orders, it cannot pick one, and a
	 * network out of range must not keep the next one from being tried. */
	enum { MAX_TRY = 32 };
	CFTypeRef order[MAX_TRY];
	CFAbsoluteTime when[MAX_TRY];
	unsigned count = 0;
	for (CFIndex i = 0; i < CFArrayGetCount(networks) && count < MAX_TRY; i++) {
		CFTypeRef network = CFArrayGetValueAtIndex(networks, i);
		CFDictionaryRef record = WiFiNetworkCopyRecord(network);
		if (record && !disabled(record)) {
			CFTypeRef date = CFDictionaryGetValue(record, CFSTR("lastJoined"));
			CFAbsoluteTime t = date && CFGetTypeID(date) == CFDateGetTypeID() ? CFDateGetAbsoluteTime(date) : -1.0e29;
			unsigned k = count++;
			for (; k > 0 && when[k - 1] < t; k--) { order[k] = order[k - 1]; when[k] = when[k - 1]; }
			order[k] = network;
			when[k] = t;
		}
		if (record) CFRelease(record);
	}
	if (!count) { puts("wifi_start: no eligible saved network"); goto out; }
	WiFiManagerClientScheduleWithRunLoop(manager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
	result = -1;
	for (unsigned n = 0; n < count && result != 0; n++) {
		CFTypeRef wanted = order[n];
		/* wifid may hold the password where this client cannot read it (a Settings join);
		 * then the request goes without one and wifid looks it up itself. */
		CFStringRef password = WiFiNetworkCopyPassword(wanted);
		if (password) { WiFiNetworkSetPassword(wanted, password); CFRelease(password); }
		else if (WiFiNetworkRequiresPassword(wanted))
			printf("wifi_start: saved network %u of %u: password not readable here; wifid asked to use its own\n", n + 1, count);
		finished = 0;
		association_error = 0;
		result = WiFiDeviceClientAssociateAsync(device, wanted, associated, NULL);
		if (result == 0) {
			for (unsigned attempt = 0; !finished && attempt < 120; attempt++)
				CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.5, false);
			result = finished ? association_error : -1;
		}
		printf("wifi_start: saved network %u of %u: %s (%d)\n", n + 1, count, result == 0 ? "joined" : "not joined", result);
	}
	WiFiManagerClientUnscheduleFromRunLoop(manager);
	printf("wifi_start: native saved-network join: %s (%d)\n", result == 0 ? "ok" : "FAILED", result);
out:
	if (devices) CFRelease(devices);
	if (networks) CFRelease(networks);
	CFRelease(manager);
	return result != 0;
}
