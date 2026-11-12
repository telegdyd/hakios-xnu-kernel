/*
 * The OnePlus 10 Pro's Wi-Fi (wifi_soc=8450): the radio is a QCA6490 on the first PCIe controller, driven by ath11k's PCI path
 * (tools/mk-ath11k.sh with wifi/lx/src/lx_pci.c and lx_mhi.c), not a DSP of the SoC. hakios.wifi.c is redwood's and keeps its
 * own steps; when the boot-arg wifi_soc is given it hands the start, the stop and the panic's way back to this file, which
 * runs the chain of WIFI-PLAN.md (stages are the numbers of sys/hakios.wifi.h, a line each, always said):
 *
 *   1 rails 2 link 3 endpoint   hakios_pcie_up()       C2, drivers/wifi/hakios.pcie.c (it says these three itself)
 *   4 smmu                      hakios_smmu_claim()    C3, SID 0x1c01, the IOVA pool (HAKIOS_WIFI_IOVA_BASE) mapped
 *   5 probe 6 mhi               lx_start()             ath11k's probe, then MHI: BHI, BHIe, EE AMSS  (lx_mhi.c says them)
 *   7 qrtr                      the IPCR link's HELLO and the WLFW server (hakios.lx.c's watcher says it)
 *   8 qmi 9 wmi                 ath11k's own exchange with the firmware and its WMI "unified ready" (lx_bus.c, lx_80211.c)
 *   10 scan 11 join             the station boundary (hakios.sta.h), as redwood's join_given()
 *   (12 is the host's: DHCP, ping, HTTPS; 13 is the owner's.)
 *
 * What is in the first half of this file is decisions with no kernel in them, so that tools/wifi-int-test/ builds this very
 * file on the host and runs them: the words (C7, WIFI-PLAN.md section 6) and the refusal of what is not understood, before any
 * hardware is touched; the table of firmware files (C8); and the chain itself with its teardown, over a table of what it
 * does to the hardware (struct hakios_wifi_pci_ops). The second half (KERNEL) is that table filled in.
 *
 * A stage that fails prints its FAILED line and everything that was made is undone in reverse (the driver, the radio's memory,
 * the SMMU context, the PCIe link: C2's rails, WLAN_EN). A panic's way back to the loader only does the last of those,
 * hakios_pcie_down(), which takes no lock and allocates nothing.
 *
 * Nothing here prints a MAC address, the passphrase, or the name of the network.
 *
 * The words (C7, WIFI-PLAN.md section 6), each read by its exact text; one that is not understood fails the stage it belongs to
 * before any hardware is touched. redwood's meanings of the words it has are not changed (without wifi_soc nothing of this runs):
 *
 *   wifi_soc=8450      the OnePlus 10 Pro's radio, on PCIe0: this file runs the Wi-Fi and redwood's WPSS steps are left out
 *   wifi_join=1        after the scan, join the network of wifi_ssid and wifi_psk (or of fw/network); default 0
 *   wifi_keep=0        take everything down again when the chain is over; default 1, with wifi=1: keep it while the system runs
 *   wifi_mac=A         an override of the radio's address, six hex bytes with colons (no group address, not zeros), used when the
 *                      firmware has no valid per-radio address of its own: ath11k's order, then WMI READY's address, then a derived one
 *   wifi_ssid=NAME     the network to join, 1 to 32 bytes without spaces (a name with one goes in fw/network); with wifi_psk
 *   wifi_psk=PASS      its WPA2 passphrase, 8 to 63 bytes; it is said in no line of any log
 *   pcie=1             only the link test, C2 up and down without Wi-Fi (hakios.pcie.c); it excludes wifi=1
 *   pcie_gen=1         the highest link generation, 1 or 2; default 2
 *   wifi_msi=1         one message-signalled interrupt vector instead of the 32 that are enabled before MHI powers up (lx_pci.c)
 *   wifi_board=NAME    the board data file of fw/ for ath11k's board.bin, a plain name; default: chosen by the QMI board id
 *   wifi_delay=S       the chain waits S seconds before stage 1, so that a reset it causes comes after the USB log is up; default 0
 * and, on this path, wifi_smmu=translate (default) or bypass (a diagnosis: exact-SID bypass for SID 0x1c01), a string.
 */
#ifdef KERNEL
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/hakios.input.h>
#include <sys/hakios.pcie.h>
#include <sys/hakios.qrtr.h>
#include <sys/hakios.smmu.h>
#include <sys/hakios.sta.h>
#include <sys/systm.h>
#else
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#endif
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/hakios.wifi.h>

/* ---- the words (C7) ------------------------------------------------------------------------------------ */

/* Each word's text as the kernel was given it; has_x says it was given. Buffers one longer than the longest good value, so
 * that a value that was cut off is told from one that fits (a full buffer is too long). */
struct hakios_wifi_raw {
	bool has_wifi, has_soc, has_join, has_keep, has_mac, has_ssid, has_psk, has_pcie, has_gen, has_smmu, has_msi, has_dump,
	    has_board;
	char wifi[16], soc[16], join[16], keep[16], mac[40], ssid[40], psk[72], pcie[16], gen[16], smmu[24], msi[16], dump[16],
	    board[72];
};

struct hakios_wifi_words {
	bool wifi, join, keep, mhi_dump, pcie_standalone, bypass;
	unsigned soc, pcie_gen, msi;
	bool has_mac, has_board;
	uint8_t mac[6];
	unsigned ssid_length, psk_length;
	char ssid[33], psk[64];                 /* the pair: both or neither */
	char board[65];                         /* the validated basename of wifi_board */
};

/* Which stage a refused word belongs to, the word, and why. */
struct hakios_wifi_word_error {
	unsigned stage;
	const char *word, *why;
};

/* (XNU has strnstr and no strstr) */
static const char *
text_find(const char *text, const char *needle)
{
	size_t n = strlen(needle);

	for (; *text != 0; text++) {
		if (strncmp(text, needle, n) == 0) {
			return text;
		}
	}
	return n == 0 ? text : NULL;
}

static bool
flag_word(const char *text, bool *value)
{
	if ((text[0] == '0' || text[0] == '1') && text[1] == 0) {
		*value = text[0] == '1';
		return true;
	}
	return false;
}

static bool
printable(const char *text, size_t length)
{
	for (size_t i = 0; i < length; i++) {
		if ((unsigned char)text[i] < 0x20 || (unsigned char)text[i] > 0x7e) {
			return false;
		}
	}
	return true;
}

/*
 * Six bytes in hexadecimal with colons between (one or two digits each, as redwood's wifi_mac always was). 0; 1 when it is not
 * that; 2 when it is a group address or all zeros, which is no station's. *bytes is written only on 0.
 */
int
hakios_wifi_mac_parse(const char *text, uint8_t *bytes)
{
	unsigned count = 0, digits = 0, value = 0;
	uint8_t found[6];
	bool good = true;

	for (const char *c = text;; c++) {
		int digit = *c >= '0' && *c <= '9' ? *c - '0' : *c >= 'a' && *c <= 'f' ? *c - 'a' + 10 : *c >= 'A' && *c <= 'F' ?
		    *c - 'A' + 10 : -1;

		if (digit >= 0 && digits < 2) {
			value = value << 4 | (unsigned)digit;
			digits++;
		} else if ((*c == ':' || *c == 0) && digits != 0 && count < 6) {
			found[count++] = (uint8_t)value;
			digits = value = 0;
		} else {
			good = false;
		}
		if (*c == 0 || !good) {
			break;
		}
	}
	if (!good || count != 6) {
		return 1;
	}
	if ((found[0] & 1) || (found[0] | found[1] | found[2] | found[3] | found[4] | found[5]) == 0) {
		return 2;
	}
	memcpy(bytes, found, sizeof(found));
	return 0;
}

#define REFUSE(stage_, word_, why_) do { error->stage = (stage_); error->word = (word_); error->why = (why_); return EINVAL; } while (0)

/*
 * 0, or EINVAL with the first word that is not understood. Words that were not given have their defaults (wifi_smmu translate,
 * pcie_gen 2, wifi_msi 32, the flags 0, wifi_keep 1). The order of the checks is the order of the stages, so the first FAILED
 * line is the earliest stage a bad word affects. Nothing here touches hardware.
 */
int
hakios_wifi_words_parse(const struct hakios_wifi_raw *raw, struct hakios_wifi_words *words, struct hakios_wifi_word_error *error)
{
	bool value;

	memset(words, 0, sizeof(*words));
	words->pcie_gen = 2;
	words->msi = 32;
	words->keep = true;
	error->stage = 0;
	error->word = error->why = "";
	if (raw->has_wifi && !flag_word(raw->wifi, &words->wifi)) {
		REFUSE(1, "wifi", "not 0 or 1");
	}
	if (raw->has_soc && !(strcmp(raw->soc, "8450") == 0)) {
		REFUSE(1, "wifi_soc", "not 8450 (absent: redwood's WPSS path)");
	}
	words->soc = raw->has_soc ? 8450 : 0;
	if (raw->has_pcie && !flag_word(raw->pcie, &words->pcie_standalone)) {
		REFUSE(1, "pcie", "not 0 or 1");
	}
	if (words->pcie_standalone && words->wifi) {
		REFUSE(1, "pcie", "pcie=1 is the standalone link test and excludes wifi=1");
	}
	if (raw->has_gen) {
		if ((raw->gen[0] != '1' && raw->gen[0] != '2') || raw->gen[1] != 0) {
			REFUSE(2, "pcie_gen", "not 1 or 2");
		}
		words->pcie_gen = (unsigned)(raw->gen[0] - '0');
	}
	if (raw->has_smmu) {
		if (strcmp(raw->smmu, "translate") == 0) {
			words->bypass = false;
		} else if (strcmp(raw->smmu, "bypass") == 0) {
			words->bypass = true;
		} else {
			REFUSE(4, "wifi_smmu", "not translate or bypass");
		}
	}
	if (raw->has_msi) {
		if (strcmp(raw->msi, "1") == 0) {
			words->msi = 1;
		} else if (strcmp(raw->msi, "32") == 0) {
			words->msi = 32;
		} else {
			REFUSE(5, "wifi_msi", "not 1 or 32");
		}
	}
	if (raw->has_dump && !flag_word(raw->dump, &words->mhi_dump)) {
		REFUSE(6, "wifi_mhi_dump", "not 0 or 1");
	}
	if (raw->has_board) {
		size_t length = strlen(raw->board);

		if (length == 0 || length >= sizeof(words->board) || length >= sizeof(raw->board) - 1) {
			REFUSE(8, "wifi_board", "empty or longer than 64 bytes");
		}
		if (strchr(raw->board, '/') != NULL || strchr(raw->board, '\\') != NULL || text_find(raw->board, "..") != NULL) {
			REFUSE(8, "wifi_board", "a file name of fw/ only: no /, \\ or ..");
		}
		for (size_t i = 0; i < length; i++) {
			char c = raw->board[i];

			if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '.' || c == '_' ||
			    c == '-' || c == '+')) {
				REFUSE(8, "wifi_board", "only letters, digits and . _ - +");
			}
		}
		memcpy(words->board, raw->board, length + 1);
		words->has_board = true;
	}
	if (raw->has_mac) {
		int rc = hakios_wifi_mac_parse(raw->mac, words->mac);

		if (rc == 1) {
			REFUSE(9, "wifi_mac", "not six colon-separated hex bytes");
		} else if (rc == 2) {
			REFUSE(9, "wifi_mac", "a group address or all zeros is no station's");
		}
		words->has_mac = true;
	}
	if (raw->has_join && !flag_word(raw->join, &words->join)) {
		REFUSE(11, "wifi_join", "not 0 or 1");
	}
	if (raw->has_keep) {
		if (!flag_word(raw->keep, &value)) {
			REFUSE(11, "wifi_keep", "not 0 or 1");
		}
		words->keep = value;
	}
	if (raw->has_ssid != raw->has_psk) {
		REFUSE(11, raw->has_ssid ? "wifi_psk" : "wifi_ssid", "wifi_ssid and wifi_psk come together");
	}
	if (raw->has_ssid) {
		size_t sl = strlen(raw->ssid), pl = strlen(raw->psk);

		if (sl < 1 || sl > 32 || sl >= sizeof(raw->ssid) - 1 || !printable(raw->ssid, sl)) {
			REFUSE(11, "wifi_ssid", "1 to 32 printable bytes");
		}
		if (pl < 8 || pl > 63 || !printable(raw->psk, pl)) {
			REFUSE(11, "wifi_psk", "8 to 63 printable bytes");
		}
		memcpy(words->ssid, raw->ssid, sl);
		words->ssid_length = (unsigned)sl;
		memcpy(words->psk, raw->psk, pl);
		words->psk_length = (unsigned)pl;
	}
	return 0;
}

/* ---- the firmware files (C8) ------------------------------------------------------------------------------ */

static bool
board_file_name(const char *name)
{
	static const char *const prefixes[] = { "bdwlan.", "bdwlang." };

	for (unsigned p = 0; p < 2; p++) {
		size_t n = strlen(prefixes[p]);

		if (strncmp(name, prefixes[p], n) != 0) {
			continue;
		}
		name += n;
		if (strcmp(name, "elf") == 0) {
			return true;
		}
		return name[0] == 'e' && ((name[1] >= '0' && name[1] <= '9') || (name[1] >= 'a' && name[1] <= 'f')) &&
		       ((name[2] >= '0' && name[2] <= '9') || (name[2] >= 'a' && name[2] <= 'f')) && name[3] == 0;
	}
	return false;
}

static bool
starts(const char *text, const char *prefix)
{
	return strncmp(text, prefix, strlen(prefix)) == 0;
}

/*
 * The file of fw/ that stands for the one ath11k asked for by Linux's name. 0 with the basename in out; or
 *   ENOENT   there is no such file here and ath11k is to take its other way (firmware-N.bin, board-2.bin) or fail (a file
 *            that is missing from the image);
 *   ENODEV   board.bin, and no wifi_board: the caller chooses by the QMI board id (wifi/lx/src/lx_policy.h) and asks again with
 *            that file's own name, which is accepted as it is;
 *   EINVAL   a name that is no file's.
 * amss.bin is amss20.bin when the request is for WCN6855's hardware 2.x (hw2.0 and hw2.1: SoC major 2, which is this chip's)
 * and amss.bin otherwise; "mhi.bin", the name upstream's comment gives it, is the same file. A wifi_board that is not there is
 * ENOENT at the read, never another board.
 */
int
hakios_wifi_fw_name(const char *request, const char *board_override, char *out, size_t size)
{
	const char *base;
	const char *name = NULL;

	if (request == NULL || request[0] == 0 || strchr(request, '\\') != NULL || text_find(request, "..") != NULL) {
		return EINVAL;
	}
	base = strrchr(request, '/');
	base = base == NULL ? request : base + 1;
	if (starts(base, "firmware-") || starts(base, "board-")) {
		return ENOENT;
	}
	if (strcmp(base, "amss.bin") == 0 || strcmp(base, "mhi.bin") == 0) {
		const char *hw = text_find(request, "/WCN6855/hw");

		/* (ath11k/WCN6855/hw2.0/ and hw2.1/: SoC major 2; any other family or major gets amss.bin) */
		name = hw != NULL && hw[11] == '2' && hw[12] == '.' ? "amss20.bin" : "amss.bin";
	} else if (strcmp(base, "m3.bin") == 0 || strcmp(base, "regdb.bin") == 0) {
		name = base;
	} else if (strcmp(base, "board.bin") == 0) {
		if (board_override == NULL || board_override[0] == 0) {
			return ENODEV;
		}
		name = board_override;
	} else if (board_file_name(base)) {
		name = base;
	} else {
		return ENOENT;
	}
	if (strlen(name) + 1 > size) {
		return EINVAL;
	}
	memcpy(out, name, strlen(name) + 1);
	return 0;
}

/* ---- the chain ----------------------------------------------------------------------------------------- */

/* What the chain does to the machine, so that a test can put a fake in its place and look at the order. 0 or an errno. */
struct hakios_wifi_pci_ops {
	void *ctx;
	void (*lock)(void *ctx);                        /* may be NULL */
	void (*unlock)(void *ctx);
	void (*wait_ms)(void *ctx, unsigned ms);
	void (*stage)(void *ctx, unsigned n, int ok, const char *detail);
	bool (*file_there)(void *ctx, const char *name);        /* a file of fw/ */
	bool (*network_there)(void *ctx);                       /* fw/network, if no wifi_ssid was given */
	int (*pcie_up)(void *ctx);                      /* stages 1-3, said by C2; on failure C2 has said which */
	void (*pcie_down)(void *ctx);
	int (*smmu_up)(void *ctx, bool bypass);
	void (*smmu_down)(void *ctx);
	int (*pool_up)(void *ctx);
	void (*pool_down)(void *ctx);
	int (*driver_start)(void *ctx);
	void (*driver_stop)(void *ctx);
	int (*scan)(void *ctx, unsigned *bss);
	int (*join)(void *ctx, int *last_error);
};

#define UP_PCIE         1u
#define UP_SMMU         2u
#define UP_POOL         4u
#define UP_DRIVER       8u

struct hakios_wifi_pci_run {
	unsigned up;                                    /* what is up, of the four above */
};

/* The stages that have been said in this run: 0 not yet, 1 ok, 2 FAILED. */
static unsigned char stage_seen[14];
static const struct hakios_wifi_pci_ops *stage_ops;     /* where the lx layer's stage lines go; set for a run */

static const char *
stage_name(unsigned n)
{
	switch (n) {
#define NAME(n_, name_) case n_: return #name_;
		HAKIOS_WIFI_STAGES(NAME)
#undef NAME
	}
	return "?";
}

static void
ops_lock(const struct hakios_wifi_pci_ops *ops)
{
	if (ops->lock != NULL) {
		ops->lock(ops->ctx);
	}
}

static void
ops_unlock(const struct hakios_wifi_pci_ops *ops)
{
	if (ops->unlock != NULL) {
		ops->unlock(ops->ctx);
	}
}

/* The line of one stage, once per run (the lock is held). */
static void
say_stage_locked(const struct hakios_wifi_pci_ops *ops, unsigned n, int ok, const char *detail)
{
	if (n < 1 || n > 13 || stage_seen[n] != 0) {
		return;
	}
	stage_seen[n] = ok ? 1 : 2;
	ops->stage(ops->ctx, n, ok, detail);
}

/* For the lx layer and the kernel's watchers (through hakios.lx.c's lx_os_stage): a stage said in some thread. */
void
hakios_wifi_pci_report(unsigned n, int ok, const char *detail)
{
	const struct hakios_wifi_pci_ops *ops = stage_ops;

	if (ops == NULL || n < 1 || n > 13) {
		return;
	}
	if (ok) {
		/* A stage that works is evidence of the ones before it: wait a moment for their own lines (several threads say them),
		 * then say what is missing, in order, as inferred, so that the log never has a stage before the one it needs. */
		for (unsigned i = 0; i < 3000; i++) {
			bool missing = false;

			ops_lock(ops);
			for (unsigned m = 5; m < n; m++) {
				missing = missing || stage_seen[m] == 0;
			}
			ops_unlock(ops);
			if (!missing) {
				break;
			}
			ops->wait_ms(ops->ctx, 1);
		}
	}
	ops_lock(ops);
	for (unsigned m = 5; ok && m < n; m++) {
		if (stage_seen[m] == 0) {
			say_stage_locked(ops, m, 1, "inferred: a later stage worked");
		}
	}
	say_stage_locked(ops, n, ok, detail);
	ops_unlock(ops);
}

/* A stage another party has said (C2 does 1-3 itself): noted, not said again. */
void
hakios_wifi_pci_mark(unsigned n, int ok)
{
	if (n >= 1 && n <= 13 && stage_seen[n] == 0) {
		stage_seen[n] = ok ? 1 : 2;
	}
}

unsigned
hakios_wifi_pci_seen(unsigned n)
{
	return n >= 1 && n <= 13 ? stage_seen[n] : 0;
}

/* The chain is about to run: no stage has been said. */
void
hakios_wifi_pci_begin(const struct hakios_wifi_pci_ops *ops)
{
	memset(stage_seen, 0, sizeof(stage_seen));
	stage_ops = ops;
}

static void
fail_stage(const struct hakios_wifi_pci_ops *ops, unsigned n, const char *detail)
{
	ops_lock(ops);
	say_stage_locked(ops, n, 0, detail);
	ops_unlock(ops);
}

static void
ok_stage(const struct hakios_wifi_pci_ops *ops, unsigned n, const char *detail)
{
	ops_lock(ops);
	say_stage_locked(ops, n, 1, detail);
	ops_unlock(ops);
}

/*
 * Undo what is up, in reverse of the order it was made in: the driver (ath11k's removal: MHI reset and C2's power-down are inside
 * it, so the radio no longer masters memory), the radio's memory (out of the SMMU context), the context, the link. Idempotent;
 * a part that is not up is left alone.
 */
void
hakios_wifi_pci_teardown(const struct hakios_wifi_pci_ops *ops, struct hakios_wifi_pci_run *run)
{
	if (run->up & UP_DRIVER) {
		run->up &= ~UP_DRIVER;
		ops->driver_stop(ops->ctx);
	}
	if (run->up & UP_POOL) {
		run->up &= ~UP_POOL;
		ops->pool_down(ops->ctx);
	}
	if (run->up & UP_SMMU) {
		run->up &= ~UP_SMMU;
		ops->smmu_down(ops->ctx);
	}
	if (run->up & UP_PCIE) {
		run->up &= ~UP_PCIE;
		ops->pcie_down(ops->ctx);
	}
}

/*
 * The chain. 0 when every stage asked for is ok; otherwise the number of the stage that failed (its FAILED line is said and
 * what was made is undone; 1 stands for "one of 1-3": C2 says which). On success everything stays up for the caller to keep
 * or to tear down. Before any hardware: the files the run will need must be in the small image, and the network of a join
 * must be given (a stage's own FAILED line, with nothing powered).
 */
int
hakios_wifi_pci_start(const struct hakios_wifi_words *w, const struct hakios_wifi_pci_ops *ops, struct hakios_wifi_pci_run *run)
{
	static const struct {
		const char *request;
		unsigned stage;
	} needed[] = {
		{ "ath11k/WCN6855/hw2.1/amss.bin", 6 }, { "ath11k/WCN6855/hw2.1/m3.bin", 8 }, { "ath11k/WCN6855/hw2.1/regdb.bin", 8 },
	};
	char detail[160], file[80];
	unsigned bss = 0, n;
	int error, last_error = 0;
	bool any_failed;

	hakios_wifi_pci_begin(ops);
	run->up = 0;
	for (unsigned i = 0; i < sizeof(needed) / sizeof(needed[0]); i++) {
		if (hakios_wifi_fw_name(needed[i].request, NULL, file, sizeof(file)) != 0 || !ops->file_there(ops->ctx, file)) {
			snprintf(detail, sizeof(detail), "fw/%s is not in the small image: nothing is powered", file);
			fail_stage(ops, needed[i].stage, detail);
			return (int)needed[i].stage;
		}
	}
	if (w->has_board && !ops->file_there(ops->ctx, w->board)) {
		snprintf(detail, sizeof(detail), "wifi_board names fw/%s, which is not in the small image: nothing is powered", w->board);
		fail_stage(ops, 8, detail);
		return 8;
	}
	if (w->join && w->ssid_length == 0 && !ops->network_there(ops->ctx)) {
		fail_stage(ops, 11, "wifi_join=1 and no network: wifi_ssid and wifi_psk, or fw/network: nothing is powered");
		return 11;
	}

	if ((error = ops->pcie_up(ops->ctx)) != 0) {
		return 1;                       /* (C2 undid itself and said which of 1-3) */
	}
	run->up |= UP_PCIE;
	hakios_wifi_pci_mark(1, 1);             /* (C2 said these three) */
	hakios_wifi_pci_mark(2, 1);
	hakios_wifi_pci_mark(3, 1);

	if ((error = ops->smmu_up(ops->ctx, w->bypass)) != 0) {
		snprintf(detail, sizeof(detail), "smmu %s for SID 0x1c01: errno %d", w->bypass ? "bypass" : "context", error);
		fail_stage(ops, 4, detail);
		hakios_wifi_pci_teardown(ops, run);
		return 4;
	}
	run->up |= UP_SMMU;
	if ((error = ops->pool_up(ops->ctx)) != 0) {
		snprintf(detail, sizeof(detail), "the radio's memory pool (64 MiB at IOVA 0xa0000000, in the 32-bit window): errno %d",
		    error);
		fail_stage(ops, 4, detail);
		hakios_wifi_pci_teardown(ops, run);
		return 4;
	}
	run->up |= UP_POOL;
	snprintf(detail, sizeof(detail), "smmu: %s for 0x1c01, 64 MiB pool at IOVA 0xa0000000 mapped", w->bypass ?
	    "bypass (diagnostic)" : "stage-1 context");
	ok_stage(ops, 4, detail);

	error = ops->driver_start(ops->ctx);
	if (error != 0) {
		/* The first stage between the probe and the station that nobody said is where it stopped (lx_start undid its own part). */
		any_failed = false;
		for (n = 5; n <= 10; n++) {
			any_failed = any_failed || hakios_wifi_pci_seen(n) == 2;
		}
		for (n = 5; n <= 10 && !any_failed; n++) {
			if (hakios_wifi_pci_seen(n) == 0) {
				snprintf(detail, sizeof(detail), "the driver's start returned errno %d", error);
				fail_stage(ops, n, detail);
				break;
			}
		}
		hakios_wifi_pci_teardown(ops, run);
		for (n = 5; n <= 10; n++) {
			if (hakios_wifi_pci_seen(n) == 2) {
				return (int)n;
			}
		}
		return 5;
	}
	run->up |= UP_DRIVER;
	for (n = 5; n <= 9; n++) {
		if (hakios_wifi_pci_seen(n) == 0) {
			ok_stage(ops, n, "inferred: the driver registered its radio");
		}
	}

	if ((error = ops->scan(ops->ctx, &bss)) != 0 || bss == 0) {
		if (error != 0) {
			snprintf(detail, sizeof(detail), "the scan failed: errno %d", error);
		} else {
			snprintf(detail, sizeof(detail), "scan: 0 BSS: the radio heard nothing (regulatory domain, or the antenna's supply)");
		}
		fail_stage(ops, 10, detail);
		hakios_wifi_pci_teardown(ops, run);
		return 10;
	}
	snprintf(detail, sizeof(detail), "scan: %u BSS", bss);
	ok_stage(ops, 10, detail);

	if (w->join) {
		if ((error = ops->join(ops->ctx, &last_error)) != 0) {
			snprintf(detail, sizeof(detail), "the network was not joined: errno %d, last error %d", error, last_error);
			fail_stage(ops, 11, detail);
			hakios_wifi_pci_teardown(ops, run);
			return 11;
		}
		ok_stage(ops, 11, "sta: joined, keys in");
	}
	return 0;
}

#ifdef KERNEL
/* ---- the kernel's side ----------------------------------------------------------------------------------- */

/* hakios.wifi.c, hakios.lx.c and the image of the small system: declared where they are used, as hakios.wifi.c does. */
extern void hakios_wifi_whole_mode(bool loud);
extern void hakios_wifi_say_kept(void);
extern const struct hakios_qrtr_ops hakios_qrtr_router_ops;
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern void hakios_lx_pcie_adopt(const struct hakios_pcie_link *link);
extern void hakios_lx_pcie_forget(void);
extern void hakios_lx_dma_context(struct hakios_smmu_context *context, bool bypass);
extern void hakios_lx_dma_release(void);
extern kern_return_t thread_terminate(thread_t thread);

#define WIFI_SID        0x1c01u

static struct {
	bool selected;                          /* wifi_soc was given: this file has the Wi-Fi, hakios.wifi.c's steps are not run */
	struct hakios_wifi_words words;
	struct hakios_wifi_pci_run run;
	struct hakios_pcie_link link;
	struct hakios_smmu_context *smmu;
	volatile bool running;                  /* the chain's thread is at work */
} pci;

static LCK_GRP_DECLARE(pci_group, "hakios-wifi-pci");
static LCK_MTX_DECLARE(pci_stage_lock, &pci_group);     /* what is said as a stage, and that it is said once */
static LCK_MTX_DECLARE(pci_run_lock, &pci_group);       /* the teardown */

static void
wipe(void *memory, size_t size)
{
	volatile unsigned char *p = memory;

	while (size--) {
		*p++ = 0;
	}
}

static void
op_lock(__unused void *ctx)
{
	lck_mtx_lock(&pci_stage_lock);
}

static void
op_unlock(__unused void *ctx)
{
	lck_mtx_unlock(&pci_stage_lock);
}

static void
op_wait(__unused void *ctx, unsigned ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static void
op_stage(__unused void *ctx, unsigned n, int ok, const char *detail)
{
	hakios_wifi_stage(n, stage_name(n), ok, "%s", detail);
}

/* pcie_pause=MS (hakios.pcie.c) here too: a line before each look into the small image, then that long a wait */
static void
look_pause(const char *name)
{
	unsigned pause = 0;

	if (PE_parse_boot_argn("pcie_pause", &pause, sizeof(pause)) && pause != 0) {
		hakios_wifi_tell("wifi: chain: looking for fw/%s in the small image\n", name);
		delay_for_interval(pause > 10000 ? 10000 : pause, NSEC_PER_MSEC);
	}
}

static bool
op_file_there(__unused void *ctx, const char *name)
{
	uint64_t size = 0;
	bool there;

	look_pause(name);
	there = hakios_image_file("fw", name, &size, NULL) != NULL;
	look_pause(there ? "(found)" : "(not found)");
	return there;
}

static bool
op_network_there(__unused void *ctx)
{
	uint64_t size = 0;

	look_pause("network");
	return hakios_image_file("fw", "network", &size, NULL) != NULL;
}

static int
op_pcie_up(__unused void *ctx)
{
	int error = hakios_pcie_up(HAKIOS_PCIE_WIFI_RC, &pci.link);

	if (error == 0) {
		hakios_lx_pcie_adopt(&pci.link);
	}
	return error;
}

static void
op_pcie_down(__unused void *ctx)
{
	hakios_lx_pcie_forget();
	hakios_pcie_down(HAKIOS_PCIE_WIFI_RC);
}

static int
op_smmu_up(__unused void *ctx, bool bypass)
{
	int error;

	if (bypass) {
		if ((error = hakios_smmu_bypass(WIFI_SID)) == 0) {
			hakios_lx_dma_context(NULL, true);
		}
		return error;
	}
	/* mask 1: 0x1c00 and 0x1c01, so that the chip's DMA matches even if the BDF-to-SID table (hakios.pcie.c) did not
	 * take and it carries the root port's SID (iOSPlus10 0325) */
	pci.smmu = hakios_smmu_claim(WIFI_SID, 1, HAKIOS_WIFI_IOVA_BASE, HAKIOS_WIFI_POOL_SIZE, "wifi");
	if (pci.smmu == NULL) {
		return EBUSY;                   /* (the claim said why) */
	}
	hakios_lx_dma_context(pci.smmu, false);
	return 0;
}

static void
op_smmu_down(__unused void *ctx)
{
	hakios_lx_dma_context(NULL, false);
	if (pci.smmu != NULL) {
		hakios_smmu_release(pci.smmu);
		pci.smmu = NULL;
	}
	/* (an exact-SID bypass has no undo in C3: it stays set until the next boot) */
}

static int
op_pool_up(__unused void *ctx)
{
	return hakios_lx_prepare();
}

static void
op_pool_down(__unused void *ctx)
{
	hakios_lx_dma_release();
}

static int
op_driver_start(__unused void *ctx)
{
	return hakios_lx_start();
}

static void
op_driver_stop(__unused void *ctx)
{
	hakios_lx_stop();
}

/* A passive scan through the station boundary, and how many access points it heard. */
static int
op_scan(__unused void *ctx, unsigned *bss)
{
	struct hakios_sta_scan what;
	struct hakios_sta_bss *one;
	uint64_t generation = hakios_sta_scan_generation();
	int error;

	bzero(&what, sizeof(what));
	what.passive = true;
	if (!hakios_sta_present()) {
		return ENXIO;
	}
	if ((error = hakios_sta_ask_scan(&what)) != 0) {
		return error;
	}
	/* wifid can queue another scan immediately from SCAN_DONE. Global busy
	 * may then never go false; wait for a completed result snapshot instead. */
	for (unsigned waited = 0; hakios_sta_scan_generation() == generation && waited < 600; waited++) {
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	if (hakios_sta_scan_generation() == generation) {
		return ETIMEDOUT;
	}
	if ((one = kalloc_type(struct hakios_sta_bss, Z_WAITOK | Z_ZERO)) == NULL) {
		return ENOMEM;
	}
	*bss = 0;
	while (*bss < 1024 && hakios_sta_heard(*bss, one)) {
		(*bss)++;                       /* (counted; nobody's name or address is said) */
	}
	kfree_type(struct hakios_sta_bss, one);
	return 0;
}

/* The network (the words' or fw/network), asked of the station as redwood's join_given() does; joined, or why not. */
static int
op_join(__unused void *ctx, int *last_error)
{
	static struct hakios_sta_join what;
	struct hakios_sta_state *state;
	unsigned name_length = sizeof(what.ssid), passphrase_length = sizeof(what.passphrase);
	int error;

	bzero(&what, sizeof(what));
	if (!hakios_wifi_network(what.ssid, &name_length, what.passphrase, &passphrase_length)) {
		*last_error = 0;
		return ENOENT;
	}
	what.ssid_length = (uint8_t)name_length;
	what.passphrase_length = (uint8_t)passphrase_length;
	what.security = HAKIOS_STA_WPA2_PSK;
	error = hakios_sta_ask_join_standing(&what);
	wipe(&what, sizeof(what));
	for (unsigned waited = 0; error == 0 && hakios_sta_busy() && waited < 400; waited++) {
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	if (error != 0) {
		*last_error = 0;
		return error;
	}
	if ((state = kalloc_type(struct hakios_sta_state, Z_WAITOK | Z_ZERO)) == NULL) {
		return ENOMEM;
	}
	hakios_sta_state(state);
	*last_error = state->last_error;
	error = state->phase == HAKIOS_STA_JOINED ? 0 : state->last_error != 0 ? state->last_error : ETIMEDOUT;
	kfree_type(struct hakios_sta_state, state);
	return error;
}

static const struct hakios_wifi_pci_ops real_ops = {
	.ctx = NULL, .lock = op_lock, .unlock = op_unlock, .wait_ms = op_wait, .stage = op_stage, .file_there = op_file_there,
	.network_there = op_network_there, .pcie_up = op_pcie_up, .pcie_down = op_pcie_down, .smmu_up = op_smmu_up,
	.smmu_down = op_smmu_down, .pool_up = op_pool_up, .pool_down = op_pool_down, .driver_start = op_driver_start,
	.driver_stop = op_driver_stop, .scan = op_scan, .join = op_join,
};

static void
fetch(struct hakios_wifi_raw *raw)
{
	bzero(raw, sizeof(*raw));
	raw->has_wifi = PE_parse_boot_arg_str("wifi", raw->wifi, sizeof(raw->wifi));
	raw->has_soc = PE_parse_boot_arg_str("wifi_soc", raw->soc, sizeof(raw->soc));
	raw->has_join = PE_parse_boot_arg_str("wifi_join", raw->join, sizeof(raw->join));
	raw->has_keep = PE_parse_boot_arg_str("wifi_keep", raw->keep, sizeof(raw->keep));
	raw->has_mac = PE_parse_boot_arg_str("wifi_mac", raw->mac, sizeof(raw->mac));
	raw->has_ssid = PE_parse_boot_arg_str("wifi_ssid", raw->ssid, sizeof(raw->ssid));
	raw->has_psk = PE_parse_boot_arg_str("wifi_psk", raw->psk, sizeof(raw->psk));
	raw->has_pcie = PE_parse_boot_arg_str("pcie", raw->pcie, sizeof(raw->pcie));
	raw->has_gen = PE_parse_boot_arg_str("pcie_gen", raw->gen, sizeof(raw->gen));
	raw->has_smmu = PE_parse_boot_arg_str("wifi_smmu", raw->smmu, sizeof(raw->smmu));
	raw->has_msi = PE_parse_boot_arg_str("wifi_msi", raw->msi, sizeof(raw->msi));
	raw->has_dump = PE_parse_boot_arg_str("wifi_mhi_dump", raw->dump, sizeof(raw->dump));
	raw->has_board = PE_parse_boot_arg_str("wifi_board", raw->board, sizeof(raw->board));
}

static void
pci_teardown(void)
{
	lck_mtx_lock(&pci_run_lock);
	hakios_wifi_pci_teardown(&real_ops, &pci.run);
	lck_mtx_unlock(&pci_run_lock);
}

static void
pci_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint64_t began, now;
	uint32_t wait_s;
	int stage;

	/* wifi_delay=S: nothing is touched before S seconds have gone by (at most an hour) */
	if ((wait_s = hakios_wifi_option("wifi_delay")) != 0) {
		if (wait_s > 3600) {
			wait_s = 3600;
		}
		hakios_wifi_tell("wifi: wifi_delay=%u: stage 1 begins in %u s\n", wait_s, wait_s);
		for (uint32_t left = wait_s; left > 0;) {
			uint32_t now_wait = left > 10 ? 10 : left;

			delay_for_interval(now_wait * 1000, NSEC_PER_MSEC);
			left -= now_wait;
			absolutetime_to_nanoseconds(mach_absolute_time(), &now);
			hakios_wifi_tell("wifi: wifi_delay: %u s left (uptime %llu s)\n", left, now / 1000000000ull);
		}
		hakios_wifi_tell("wifi: wifi_delay over: stage 1 begins now\n");
	}
	absolutetime_to_nanoseconds(mach_absolute_time(), &began);
	if (wait_s != 0) {
		hakios_wifi_tell("wifi: chain: the firmware files and the network are looked for, then C2 up\n");
	}
	stage = hakios_wifi_pci_start(&pci.words, &real_ops, &pci.run);
	/*
	 * Once more after a failure between the probe and the radio's registration (stages 5-9): on a warm reboot the
	 * chip has been seen to reach mission mode and never complete IPCR's channel start (2 of ~14 boots). The failed
	 * run has undone everything, rails and WLAN_EN included; the chip stays unpowered for a second first.
	 */
	if (stage >= 5 && stage <= 9) {
		hakios_wifi_tell("wifi: stage %d failed: the radio is powered off and brought up once more in 1 s\n", stage);
		delay_for_interval(1000, NSEC_PER_MSEC);
		stage = hakios_wifi_pci_start(&pci.words, &real_ops, &pci.run);
		hakios_wifi_tell("wifi: the second bring-up: %s\n", stage == 0 ? "ok" : "failed too");
	}
	if (stage == 0) {
		absolutetime_to_nanoseconds(mach_absolute_time(), &now);
		hakios_wifi_tell("wifi: THE RADIO IS UP (QCA6490 over PCIe0), %llu.%llu s after the kernel began on it%s\n",
		    (now - began) / 1000000000ull, (now - began) / 100000000ull % 10, pci.words.keep ?
		    "; it stays up while the system runs" : "; wifi_keep=0: it is taken down again now");
		if (!pci.words.keep) {
			pci_teardown();
		}
	} else {
		hakios_wifi_tell("wifi: THE RADIO DID NOT COME UP: stage %u failed (what was made is undone, in reverse). The last of what "
		    "was said on the way:\n", (unsigned)stage);
		hakios_wifi_say_kept();
	}
	/* (both halves of the network: a name left without its passphrase would still count as given, and
	 * hakios_wifi_network() would then never fall back to fw/network) */
	wipe(pci.words.psk, sizeof(pci.words.psk));
	pci.words.psk_length = 0;
	wipe(pci.words.ssid, sizeof(pci.words.ssid));
	pci.words.ssid_length = 0;
	pci.running = false;
	thread_terminate(current_thread());
}

/*
 * Called first by hakios_wifi_init(): true when wifi_soc is given, which is this file's path (and redwood's steps are not
 * run, whatever else is given); false when it is absent. With wifi=1 the chain starts on a thread of its own, beside the
 * system's start; words that are not understood are said as their stage's FAILED line and nothing is powered.
 */
extern int hakios_lx_pool_reserve(bool bypass);         /* hakios.lx.c */

bool
hakios_wifi_pci_init(void)
{
	struct hakios_wifi_raw raw;
	struct hakios_wifi_word_error error;
	thread_t thread;
	char soc[16];
	bool whole_wifi;

	if (!PE_parse_boot_arg_str("wifi_soc", soc, sizeof(soc))) {
		return false;
	}
	pci.selected = true;
	fetch(&raw);
	whole_wifi = raw.has_wifi && strcmp(raw.wifi, "0") != 0;
	if (!whole_wifi) {
		wipe(&raw, sizeof(raw));
		return true;                    /* (wifi=0 or absent: nothing runs; pcie=1 is C2's own) */
	}
	if (hakios_wifi_words_parse(&raw, &pci.words, &error) != 0) {
		hakios_wifi_stage(error.stage, stage_name(error.stage), 0, "%s: %s", error.word, error.why);
		wipe(&raw, sizeof(raw));
		wipe(&pci.words, sizeof(pci.words));
		return true;
	}
	wipe(&raw, sizeof(raw));
	hakios_qmi_set_transport(&hakios_qrtr_router_ops);
	hakios_wifi_whole_mode(hakios_wifi_option("wifi_loud") != 0);
	hakios_lx_net_early();
	/* the DMA pool now, while 64 MiB in one piece are still to be had (stage 4 only maps it) */
	if (hakios_lx_pool_reserve(pci.words.bypass) != 0) {
		hakios_wifi_tell("wifi: no 64 MiB in one piece for the radio at the start: stage 4 will try again\n");
	}
	pci.running = true;
	if (kernel_thread_start(pci_thread, NULL, &thread) != KERN_SUCCESS) {
		pci.running = false;
		hakios_wifi_tell("wifi: no thread: the radio is not brought up\n");
		return true;
	}
	thread_deallocate(thread);
	hakios_wifi_tell("wifi: the radio (PCIe path, wifi_soc=8450) is being brought up beside the system's start\n");
	return true;
}

bool
hakios_wifi_pci_selected(void)
{
	return pci.selected;
}

/* Where a run ends in order: the chain is let finish (or fail), and then undone in reverse. */
bool
hakios_wifi_pci_stop(void)
{
	if (!pci.selected) {
		return false;
	}
	for (unsigned ms = 0; pci.running && ms < 90000; ms += 10) {
		delay_for_interval(10, NSEC_PER_MSEC);
	}
	pci_teardown();
	return true;
}

/* A panic's way back to the loader: the radio is powered down, with no lock and no allocation (C2's hakios_pcie_down). */
bool
hakios_wifi_pci_quiesce(void)
{
	if (!pci.selected) {
		return false;
	}
	hakios_pcie_down(HAKIOS_PCIE_WIFI_RC);
	return true;
}

/*
 * What ath11k asked for by Linux's name, in the small image's fw/: 0 with the file in place; ENOENT (not there, or not offered:
 * ath11k takes its other way); ENODEV (board.bin and no wifi_board: the lx layer chooses by the QMI board id).
 */
int
hakios_wifi_pci_firmware(const char *name, const uint8_t **data, uint64_t *size)
{
	char file[80];
	int error = hakios_wifi_fw_name(name, pci.words.has_board ? pci.words.board : NULL, file, sizeof(file));

	if (error != 0) {
		return error == EINVAL ? ENOENT : error;
	}
	*data = hakios_image_file("fw", file, size, NULL);
	if (*data == NULL) {
		hakios_wifi_say("wifi: no fw/%s in the small image (the driver asked for %s)\n", file, name);
		return ENOENT;
	}
	hakios_wifi_say("wifi: fw/%s is what the driver gets for %s\n", file, name);
	return 0;
}

/* The words' network, if the pair was given; false: the caller reads fw/network. The passphrase is never said. */
bool
hakios_wifi_pci_network(uint8_t *name, unsigned *name_length, uint8_t *passphrase, unsigned *passphrase_length)
{
	if (!pci.selected || pci.words.ssid_length == 0 || pci.words.ssid_length > *name_length ||
	    pci.words.psk_length > *passphrase_length) {
		return false;
	}
	memcpy(name, pci.words.ssid, pci.words.ssid_length);
	memcpy(passphrase, pci.words.psk, pci.words.psk_length);
	*name_length = pci.words.ssid_length;
	*passphrase_length = pci.words.psk_length;
	hakios_wifi_say("wifi: which network to join: a name of %u bytes and a passphrase of %u bytes were given as words\n",
	    *name_length, *passphrase_length);
	return true;
}
#endif /* KERNEL */
