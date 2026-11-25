/* Wave-0 contract: round-1 stage numbers and names are a log-file ABI. */
#ifndef _SYS_HAKIOS_WIFI_H_
#define _SYS_HAKIOS_WIFI_H_

/* n/name must be one row below; no new stage numbers for substeps or audio.
 * Stage 12 is completed by the host test; stage 13 by the owner, never inferred
 * from an earlier driver stage. Audio regression lines keep their existing names.
 */
#define HAKIOS_WIFI_STAGES(X) \
	X(1, rails) \
	X(2, link) \
	X(3, endpoint) \
	X(4, smmu) \
	X(5, probe) \
	X(6, mhi) \
	X(7, qrtr) \
	X(8, qmi) \
	X(9, wmi) \
	X(10, scan) \
	X(11, join) \
	X(12, network) \
	X(13, settings)

/* One line: wifi: STAGE <n> <name>: ok <detail> or FAILED <reason>.
 * Always printf + loader_log_line, even with quiet Wi-Fi logging. Thread
 * context; detail is bounded to 255 bytes, CR/LF replaced with spaces.
 * Callers pass a literal name from the table and a printf format (possibly "").
 */
void hakios_wifi_stage(unsigned n, const char *name, int ok, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

#endif
