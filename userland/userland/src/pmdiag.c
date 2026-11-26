/*
 * pmdiag: why the system does not sleep (COMMON-PORT.md "System sleep"). Prints powerd's assertions by process and
 * their totals (IOPMCopyAssertionsByProcess, IOPMCopyAssertionsStatus: what pmset -g assertions shows on a Mac, which
 * iOS has no tool for), and with -k the kernel's message buffer (proc_kmsgbuf, what dmesg prints; Darwin 27 has no
 * dmesg here): with io=0x20000 it holds the root domain's own lines (PMRD:).
 *
 *   pmdiag        assertions
 *   pmdiag -k     assertions, then the kernel's buffer
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
