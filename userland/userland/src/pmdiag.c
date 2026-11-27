/*
 * pmdiag: why the system does not sleep (COMMON-PORT.md "System sleep"). Prints powerd's assertions by process and
 * their totals (IOPMCopyAssertionsByProcess, IOPMCopyAssertionsStatus: what pmset -g assertions shows on a Mac, which
 * iOS has no tool for), and with -k the kernel's message buffer (proc_kmsgbuf, what dmesg prints; Darwin 27 has no
 * dmesg here): with io=0x20000 it holds the root domain's own lines (PMRD:).
 *
 *   pmdiag        assertions
 *   pmdiag -k     assertions, then the kernel's buffer
 *   pmdiag -w S   a wake scheduled through powerd S seconds from now (IOPMSchedulePowerEvent, "wake": what an alarm
 *                 asks for; powerd hands the root domain WakeByCalendarDate), then the list below
 *   pmdiag -l     the power events powerd has scheduled (IOPMCopyScheduledPowerEvents)
 */
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* IOKit/pwr_mgt/IOPMLib.h (not in the iOS SDK's headers) and libproc.h */
typedef int IOReturn;
extern IOReturn IOPMCopyAssertionsByProcess(CFDictionaryRef *assertionsByPID);
extern IOReturn IOPMCopyAssertionsStatus(CFDictionaryRef *assertionsStatus);
extern int proc_kmsgbuf(void *buffer, uint32_t buffersize);
extern IOReturn IOPMSchedulePowerEvent(CFDateRef time_to_wake, CFStringRef my_id, CFStringRef type);
extern CFArrayRef IOPMCopyScheduledPowerEvents(void);

static void
show(const char *title, CFTypeRef value)
{
	CFStringRef text;
	char *line;
	CFIndex size;

	if (value == NULL) {
		printf("%s: none\n", title);
		return;
	}
	text = CFCopyDescription(value);
	size = CFStringGetMaximumSizeForEncoding(CFStringGetLength(text), kCFStringEncodingUTF8) + 1;
	line = malloc((size_t)size);
	if (line != NULL && CFStringGetCString(text, line, size, kCFStringEncodingUTF8)) {
		printf("%s:\n%s\n", title, line);
	}
	free(line);
	CFRelease(text);
}

int
main(int argc, char **argv)
{
	CFDictionaryRef by = NULL, status = NULL;
	IOReturn r;

	if (argc > 2 && strcmp(argv[1], "-w") == 0) {
		double secs = atof(argv[2]);
		CFDateRef when = CFDateCreate(NULL, CFAbsoluteTimeGetCurrent() + secs);

		r = IOPMSchedulePowerEvent(when, CFSTR("com.hakios.pmdiag"), CFSTR("wake"));
		printf("IOPMSchedulePowerEvent(now + %.0f s, wake): 0x%x\n", secs, r);
		CFRelease(when);
	}
	if (argc > 1 && (strcmp(argv[1], "-l") == 0 || strcmp(argv[1], "-w") == 0)) {
		CFArrayRef events = IOPMCopyScheduledPowerEvents();

		show("scheduled power events", events);
		if (events != NULL) CFRelease(events);
		return 0;
	}
	r = IOPMCopyAssertionsByProcess(&by);
	printf("IOPMCopyAssertionsByProcess: 0x%x\n", r);
	show("assertions by process", by);
	r = IOPMCopyAssertionsStatus(&status);
	printf("IOPMCopyAssertionsStatus: 0x%x\n", r);
	show("assertions in force", status);
	if (by != NULL) CFRelease(by);
	if (status != NULL) CFRelease(status);

	if (argc > 1 && strcmp(argv[1], "-k") == 0) {
		static char buffer[1024 * 1024];
		int n = proc_kmsgbuf(buffer, sizeof(buffer));

		printf("--- kernel buffer (%d bytes) ---\n", n);
		if (n > 0) fwrite(buffer, 1, (size_t)n, stdout);
	}
	return 0;
}
