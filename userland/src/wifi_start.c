/* Join the saved Wi-Fi networks through MobileWiFi/wifid, never kernel boot words.
 * wifid here does not auto-join by itself (its iOS 16 known-network store is never
 * made), so this job stays running in System and does it: at boot, after Wi-Fi is
 * turned back on, and after the link is lost. It only asks while Wi-Fi is on and has
 * been unjoined for two looks in a row, and backs off when no saved network joins.
 * No names, addresses or passwords are logged. A password readable here is handed to
 * MobileWiFi; one stored by a Settings join is not, and wifid then uses its own.
 * Private API signatures: ios-reversed-headers/MobileWiFi (Cykey), as used by
 * the phone-tested diagnostic. Resolve at runtime because the public SDK omits them.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>

#define LOAD(name, result, args) \
	name = (result (*) args)dlsym(library, #name); \
	if (name == NULL) { puts("wifi_start: native Wi-Fi API unavailable: " #name); return 1; }

static CFTypeRef (*WiFiManagerClientCreate)(CFAllocatorRef, int);
static CFArrayRef (*WiFiManagerClientCopyDevices)(CFTypeRef);
static CFArrayRef (*WiFiManagerClientCopyNetworks)(CFTypeRef);
static void (*WiFiManagerClientScheduleWithRunLoop)(CFTypeRef, CFRunLoopRef, CFStringRef);
static void (*WiFiManagerClientUnscheduleFromRunLoop)(CFTypeRef);
static int (*WiFiDeviceClientGetPower)(CFTypeRef);
static CFTypeRef (*WiFiDeviceClientCopyCurrentNetwork)(CFTypeRef);
static CFDictionaryRef (*WiFiNetworkCopyRecord)(CFTypeRef);
static CFStringRef (*WiFiNetworkCopyPassword)(CFTypeRef);
static Boolean (*WiFiNetworkRequiresPassword)(CFTypeRef);
static void (*WiFiNetworkSetPassword)(CFTypeRef, CFStringRef);
static int (*WiFiDeviceClientAssociateAsync)(CFTypeRef, CFTypeRef,
    void (*)(CFTypeRef, CFTypeRef, CFDictionaryRef, int, const void *), CFDictionaryRef);

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

static Boolean
joined(CFTypeRef device)
{
	CFTypeRef current = WiFiDeviceClientCopyCurrentNetwork(device);
	if (current) CFRelease(current);
	return current != NULL;
}

/* Every eligible saved network, newest lastJoined first, until one joins: the clock
 * restarts at 1970 on each boot, so lastJoined only orders, it cannot pick one, and a
 * network out of range must not keep the next one from being tried.
 * 0: joined (by us, or by someone else meanwhile); 1: nothing joined; -1: nothing to try. */
static int
join_saved(CFTypeRef manager, CFTypeRef device)
{
	CFArrayRef networks = WiFiManagerClientCopyNetworks(manager);
	enum { MAX_TRY = 32 };
	CFTypeRef order[MAX_TRY];
	CFAbsoluteTime when[MAX_TRY];
	unsigned count = 0;
	for (CFIndex i = 0; networks && i < CFArrayGetCount(networks) && count < MAX_TRY; i++) {
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
	if (!count) {
		if (networks) CFRelease(networks);
		return -1;
	}
	int result = -1;
	for (unsigned n = 0; n < count && result != 0; n++) {
		if (!WiFiDeviceClientGetPower(device) || joined(device)) { result = 0; break; }
		CFTypeRef wanted = order[n];
		CFStringRef password = WiFiNetworkCopyPassword(wanted);
		if (password) { WiFiNetworkSetPassword(wanted, password); CFRelease(password); }
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
	CFRelease(networks);
	return result == 0 ? 0 : 1;
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
	/* The client's view of the device (attached, power, network) is kept current by wifid's notifications, which
	 * arrive only while its run loop runs: so the waits below run it, and it stays scheduled for the session. */
	WiFiManagerClientScheduleWithRunLoop(manager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);

	/* unjoined: consecutive 5 s looks with Wi-Fi on and no network (two before asking, so a
	 * join someone else has started is left alone; at boot it may ask at once). */
	unsigned unjoined = 1, backoff = 0, wait = 0;
	int was = -2;	/* last reported state: -1 off, 0 unjoined, 1 joined */
	for (;;) {
		CFArrayRef devices = WiFiManagerClientCopyDevices(manager);
		CFTypeRef device = devices && CFArrayGetCount(devices) ? CFArrayGetValueAtIndex(devices, 0) : NULL;
		int state = !device || !WiFiDeviceClientGetPower(device) ? -1 : joined(device) ? 1 : 0;
		if (state != was && state != 0) {
			puts(state < 0 ? "wifi_start: Wi-Fi is off or the radio is not up" : "wifi_start: joined");
		}
		was = state;
		if (state != 0) {
			unjoined = 0;
			backoff = wait = 0;
		} else if (++unjoined >= 2 && wait == 0) {
			int r = join_saved(manager, device);
			if (r == 0) {
				backoff = 0;
			} else {
				/* nothing of ours in range (or nothing saved): 30 s, doubling to 5 min */
				backoff = backoff ? (backoff * 2 > 60 ? 60 : backoff * 2) : 6;
				wait = backoff;
				if (r < 0) puts("wifi_start: no eligible saved network");
			}
			unjoined = 0;
		} else if (wait) {
			wait--;
		}
		if (devices) CFRelease(devices);
		if (CFRunLoopRunInMode(kCFRunLoopDefaultMode, 5.0, false) == kCFRunLoopRunFinished) {
			sleep(5);       /* (no source in the mode: never spin) */
		}
	}
}
