/* C1: shared Wi-Fi stage logger. No hardware access or allocation. */
#include <libkern/libkern.h>
#include <sys/hakios.wifi.h>

extern void loader_log_line(const char *line);

void
hakios_wifi_stage(unsigned n, const char *name, int ok, const char *fmt, ...)
{
	char detail[256], line[352];
	va_list args;

	va_start(args, fmt);
	vsnprintf(detail, sizeof(detail), fmt, args);
	va_end(args);
	for (char *c = detail; *c; c++) {
		if (*c == '\r' || *c == '\n') {
			*c = ' ';
		}
	}
	snprintf(line, sizeof(line), "wifi: STAGE %u %s: %s %s\n", n, name, ok ? "ok" : "FAILED", detail);
	printf("%s", line);
	loader_log_line(line);
}
