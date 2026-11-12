/*
 * Host test of WP6's decisions: the production files are compiled in (drivers/wifi/hakios.wifi.pci.c without KERNEL, and
 * wifi/lx/src/lx_policy.h), with only the machine replaced by a fake that writes down what the chain does to it.
 *
 *   words        parsing, defaults, and the refusal of every word that is not understood (with the stage it belongs to)
 *   firmware     the table of C8: amss -> amss20, m3, regdb, the ENOENT of firmware-N / board-2, board.bin and the override
 *   mac          ath11k's order, the source named, the locally administered fallback, no address ever said
 *   chain        the order of the stages and the teardown after each stage's failure
 */
#include "../../drivers/wifi/hakios.wifi.pci.c"
#include "../../wifi/lx/src/lx_policy.h"

#include <assert.h>
#include <ctype.h>

static int checks, fails;
#define CHECK(cond) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_STR(a, b) do { checks++; if (strcmp((a), (b)) != 0) { fails++; printf("FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__, (a), (b)); } } while (0)

/* ---------------------------------------------------------------- words */

static struct hakios_wifi_raw raw;
static struct hakios_wifi_words words;
static struct hakios_wifi_word_error werr;

static void
reset(void)
{
	memset(&raw, 0, sizeof(raw));
}
#define SET(field, flag, text) do { raw.flag = true; snprintf(raw.field, sizeof(raw.field), "%s", text); } while (0)

static int
parse(void)
{
	return hakios_wifi_words_parse(&raw, &words, &werr);
}

static void
refused(unsigned stage, const char *word)
{
	int rc = parse();

	CHECK(rc == EINVAL);
	CHECK(werr.stage == stage);
	CHECK_STR(werr.word, word);
	CHECK(werr.why[0] != 0);
}

static void
test_words(void)
{
	char long_text[200];

	/* nothing given: the defaults of the table */
	reset();
	CHECK(parse() == 0);
	CHECK(!words.wifi && !words.join && words.keep && !words.mhi_dump && !words.pcie_standalone && !words.bypass);
	CHECK(words.soc == 0 && words.pcie_gen == 2 && words.msi == 32 && !words.has_mac && !words.has_board && words.ssid_length == 0);

	/* round 1: the words the plan names */
	reset();
	SET(wifi, has_wifi, "1"); SET(soc, has_soc, "8450"); SET(join, has_join, "1"); SET(keep, has_keep, "1");
	SET(board, has_board, "bdwlang.b0e"); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, "correct horse 42");
	CHECK(parse() == 0);
	CHECK(words.wifi && words.soc == 8450 && words.join && words.keep && words.has_board);
	CHECK_STR(words.board, "bdwlang.b0e");
	CHECK(words.ssid_length == 7 && strcmp(words.ssid, "HomeNet") == 0 && words.psk_length == 16 && strcmp(words.psk, "correct horse 42") == 0);

	/* every knob, valid */
	reset();
	SET(wifi, has_wifi, "1"); SET(soc, has_soc, "8450"); SET(gen, has_gen, "1"); SET(smmu, has_smmu, "bypass"); SET(msi, has_msi, "1");
	SET(dump, has_dump, "1"); SET(keep, has_keep, "0"); SET(mac, has_mac, "02:11:22:33:44:55"); SET(pcie, has_pcie, "0");
	CHECK(parse() == 0);
	CHECK(words.pcie_gen == 1 && words.bypass && words.msi == 1 && words.mhi_dump && !words.keep && words.has_mac &&
	    words.mac[0] == 0x02 && words.mac[5] == 0x55 && !words.pcie_standalone);
	reset();
	SET(smmu, has_smmu, "translate"); SET(gen, has_gen, "2"); SET(msi, has_msi, "32");
	CHECK(parse() == 0 && !words.bypass && words.pcie_gen == 2 && words.msi == 32);
	reset();
	SET(pcie, has_pcie, "1");                       /* the standalone diagnostic, without wifi=1 */
	CHECK(parse() == 0 && words.pcie_standalone && !words.wifi);

	/* refusals, each with the stage it belongs to, and before anything else happens (the parser has no hardware to touch) */
	reset(); SET(wifi, has_wifi, "2"); refused(1, "wifi");
	reset(); SET(wifi, has_wifi, ""); refused(1, "wifi");
	reset(); SET(soc, has_soc, "7280"); refused(1, "wifi_soc");
	reset(); SET(soc, has_soc, "08450"); refused(1, "wifi_soc");
	reset(); SET(soc, has_soc, "8450x"); refused(1, "wifi_soc");
	reset(); SET(pcie, has_pcie, "yes"); refused(1, "pcie");
	reset(); SET(wifi, has_wifi, "1"); SET(pcie, has_pcie, "1"); refused(1, "pcie");     /* mutually exclusive */
	reset(); SET(gen, has_gen, "3"); refused(2, "pcie_gen");
	reset(); SET(gen, has_gen, "0"); refused(2, "pcie_gen");
	reset(); SET(gen, has_gen, "12"); refused(2, "pcie_gen");
	reset(); SET(gen, has_gen, ""); refused(2, "pcie_gen");
	reset(); SET(smmu, has_smmu, "1"); refused(4, "wifi_smmu");        /* redwood's numeric forms are not this path's */
	reset(); SET(smmu, has_smmu, "2"); refused(4, "wifi_smmu");
	reset(); SET(smmu, has_smmu, "Bypass"); refused(4, "wifi_smmu");
	reset(); SET(smmu, has_smmu, "translate "); refused(4, "wifi_smmu");
	reset(); SET(smmu, has_smmu, ""); refused(4, "wifi_smmu");
	reset(); SET(msi, has_msi, "2"); refused(5, "wifi_msi");
	reset(); SET(msi, has_msi, "0"); refused(5, "wifi_msi");
	reset(); SET(msi, has_msi, "16"); refused(5, "wifi_msi");
	reset(); SET(msi, has_msi, "032"); refused(5, "wifi_msi");
	reset(); SET(dump, has_dump, "2"); refused(6, "wifi_mhi_dump");
	reset(); SET(dump, has_dump, "on"); refused(6, "wifi_mhi_dump");
	reset(); SET(join, has_join, "2"); refused(11, "wifi_join");
	reset(); SET(keep, has_keep, "maybe"); refused(11, "wifi_keep");

	/* wifi_board: a plain file name of fw/ only */
	reset(); SET(board, has_board, "../bdwlang.b0e"); refused(8, "wifi_board");
	reset(); SET(board, has_board, "a/b"); refused(8, "wifi_board");
	reset(); SET(board, has_board, "/bdwlang.b0e"); refused(8, "wifi_board");
	reset(); SET(board, has_board, "a\\b"); refused(8, "wifi_board");
	reset(); SET(board, has_board, "bdwlang..b0e"); refused(8, "wifi_board");
	reset(); SET(board, has_board, ".."); refused(8, "wifi_board");
	reset(); SET(board, has_board, ""); refused(8, "wifi_board");
	reset(); SET(board, has_board, "bad name!"); refused(8, "wifi_board");
	reset(); SET(board, has_board, "caf\xc3\xa9"); refused(8, "wifi_board");
	memset(long_text, 'a', sizeof(raw.board) - 1);   /* as long as the buffer holds: cut off by whoever fetched it */
	long_text[sizeof(raw.board) - 1] = 0;
	reset(); SET(board, has_board, long_text); refused(8, "wifi_board");
	memset(long_text, 'a', 65);
	long_text[65] = 0;
	reset(); SET(board, has_board, long_text); refused(8, "wifi_board");              /* 65 bytes: too long for a name */
	memset(long_text, 'a', 64);
	long_text[64] = 0;
	reset(); SET(board, has_board, long_text); CHECK(parse() == 0);                    /* 64 bytes: the longest */

	/* wifi_mac: an override only; never a group address or zeros */
	reset(); SET(mac, has_mac, "01:00:5e:00:00:01"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "00:00:00:00:00:00"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "02:00:00:00:00"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "02:00:00:00:00:00:01"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "zz:00:00:00:00:01"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "02:00:00:00:00:01:"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "02-00-00-00-00-01"); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, ""); refused(9, "wifi_mac");
	reset(); SET(mac, has_mac, "AA:bb:CC:dd:EE:f0"); CHECK(parse() == 0 && words.mac[0] == 0xaa && words.mac[5] == 0xf0);

	/* the pair: both or neither; sizes; printable; and the passphrase is never in what is said about it */
	reset(); SET(ssid, has_ssid, "HomeNet"); refused(11, "wifi_psk");
	reset(); SET(psk, has_psk, "secretsecret"); refused(11, "wifi_ssid");
	reset(); SET(ssid, has_ssid, ""); SET(psk, has_psk, "secretsecret"); refused(11, "wifi_ssid");
	memset(long_text, 'n', 33);
	long_text[33] = 0;
	reset(); SET(ssid, has_ssid, long_text); SET(psk, has_psk, "secretsecret"); refused(11, "wifi_ssid");
	memset(long_text, 'n', 32);
	long_text[32] = 0;
	reset(); SET(ssid, has_ssid, long_text); SET(psk, has_psk, "secretsecret"); CHECK(parse() == 0 && words.ssid_length == 32);
	reset(); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, "short77"); refused(11, "wifi_psk");
	CHECK(strstr(werr.why, "short77") == NULL && strstr(werr.word, "short77") == NULL);
	memset(long_text, 'p', 64);
	long_text[64] = 0;
	reset(); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, long_text); refused(11, "wifi_psk");
	memset(long_text, 'p', 63);
	long_text[63] = 0;
	reset(); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, long_text); CHECK(parse() == 0 && words.psk_length == 63);
	reset(); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, "bad\tpassphrase"); refused(11, "wifi_psk");
	CHECK(strstr(werr.why, "passphrase") == NULL);
	reset(); SET(ssid, has_ssid, "Home\nNet"); SET(psk, has_psk, "secretsecret"); refused(11, "wifi_ssid");

	/* the earliest stage a bad word touches comes first when several are bad */
	reset(); SET(gen, has_gen, "9"); SET(smmu, has_smmu, "no"); SET(msi, has_msi, "3"); refused(2, "pcie_gen");

	/* a refusal leaves nothing half-parsed behind: the output is zero apart from the defaults */
	reset(); SET(wifi, has_wifi, "1"); SET(msi, has_msi, "7");
	CHECK(parse() == EINVAL && words.psk_length == 0 && words.ssid_length == 0);
}

/* ---------------------------------------------------------------- the MAC rules of redwood's wifi_mac: one parser */

static void
test_mac_parse(void)
{
	uint8_t out[6] = { 9, 9, 9, 9, 9, 9 };

	CHECK(hakios_wifi_mac_parse("02:00:00:00:00:01", out) == 0 && out[0] == 2 && out[5] == 1);
	memset(out, 9, 6);
	CHECK(hakios_wifi_mac_parse("01:02:03:04:05:06", out) == 2 && out[0] == 9);      /* group bit: refused, nothing written */
	CHECK(hakios_wifi_mac_parse("2:2:3:4:5:6", out) == 0 && out[0] == 2 && out[1] == 2);   /* one digit a byte, as always */
	CHECK(hakios_wifi_mac_parse("002:00:00:00:00:01", out) == 1);
	CHECK(hakios_wifi_mac_parse("", out) == 1);
	CHECK(hakios_wifi_mac_parse(":::::", out) == 1);
}

/* ---------------------------------------------------------------- firmware names (C8) */

static const char *
fw(const char *request, const char *override, int *rc)
{
	static char out[80];

	memset(out, 0, sizeof(out));
	*rc = hakios_wifi_fw_name(request, override, out, sizeof(out));
	return out;
}

static void
test_firmware(void)
{
	int rc;
	char tiny[8], name[24];

	CHECK_STR(fw("ath11k/WCN6855/hw2.0/amss.bin", NULL, &rc), "amss20.bin"); CHECK(rc == 0);
	CHECK_STR(fw("ath11k/WCN6855/hw2.1/amss.bin", NULL, &rc), "amss20.bin"); CHECK(rc == 0);
	CHECK_STR(fw("ath11k/WCN6855/hw2.1/mhi.bin", NULL, &rc), "amss20.bin"); CHECK(rc == 0);     /* upstream's comment's name */
	CHECK_STR(fw("ath11k/QCA6390/hw2.0/amss.bin", NULL, &rc), "amss.bin"); CHECK(rc == 0);       /* another family: no major 2 rule */
	CHECK_STR(fw("ath11k/WCN6855/hw1.1/amss.bin", NULL, &rc), "amss.bin"); CHECK(rc == 0);       /* another major */
	CHECK_STR(fw("amss.bin", NULL, &rc), "amss.bin"); CHECK(rc == 0);
	CHECK_STR(fw("ath11k/WCN6855/hw2.1/m3.bin", NULL, &rc), "m3.bin"); CHECK(rc == 0);
	CHECK_STR(fw("ath11k/WCN6855/hw2.0/regdb.bin", NULL, &rc), "regdb.bin"); CHECK(rc == 0);
	/* the files ath11k asks for first and can do without: ENOENT, so that it takes its separate-file way */
	fw("ath11k/WCN6855/hw2.1/firmware-2.bin", NULL, &rc); CHECK(rc == ENOENT);
	fw("ath11k/WCN6855/hw2.1/firmware-5.bin", NULL, &rc); CHECK(rc == ENOENT);
	fw("ath11k/WCN6855/hw2.1/board-2.bin", NULL, &rc); CHECK(rc == ENOENT);
	fw("ath11k/WCN6855/hw2.1/board-2.bin", "bdwlang.b0e", &rc); CHECK(rc == ENOENT);   /* an override is for board.bin only */
	fw("ath11k/WCN6855/hw2.1/unknown.bin", NULL, &rc); CHECK(rc == ENOENT);
	/* board.bin: the override, or ENODEV (the caller chooses by the QMI board id), never a guess */
	fw("ath11k/WCN6855/hw2.1/board.bin", NULL, &rc); CHECK(rc == ENODEV);
	fw("ath11k/WCN6855/hw2.1/board.bin", "", &rc); CHECK(rc == ENODEV);
	CHECK_STR(fw("ath11k/WCN6855/hw2.1/board.bin", "bdwlang.b0e", &rc), "bdwlang.b0e"); CHECK(rc == 0);
	CHECK_STR(fw("board.bin", "bdwlan.elf", &rc), "bdwlan.elf"); CHECK(rc == 0);
	/* the automatic names, when the caller asks for them by their own name: only the shapes of cnss2's rule */
	CHECK_STR(fw("bdwlang.elf", NULL, &rc), "bdwlang.elf"); CHECK(rc == 0);
	CHECK_STR(fw("bdwlan.elf", NULL, &rc), "bdwlan.elf"); CHECK(rc == 0);
	CHECK_STR(fw("bdwlang.e0e", NULL, &rc), "bdwlang.e0e"); CHECK(rc == 0);
	CHECK_STR(fw("bdwlan.eff", NULL, &rc), "bdwlan.eff"); CHECK(rc == 0);
	fw("bdwlang.b0e", NULL, &rc); CHECK(rc == ENOENT);                  /* (that one is reached through wifi_board) */
	fw("bdwlan.e0", NULL, &rc); CHECK(rc == ENOENT);
	fw("bdwlan.e0g", NULL, &rc); CHECK(rc == ENOENT);
	fw("bdwlan.e0e1", NULL, &rc); CHECK(rc == ENOENT);
	fw("bdwlanx.elf", NULL, &rc); CHECK(rc == ENOENT);
	/* names that are no file's */
	fw(NULL, NULL, &rc); CHECK(rc == EINVAL);
	fw("", NULL, &rc); CHECK(rc == EINVAL);
	fw("../../etc/amss.bin", NULL, &rc); CHECK(rc == EINVAL);
	fw("ath11k\\WCN6855\\amss.bin", NULL, &rc); CHECK(rc == EINVAL);
	CHECK(hakios_wifi_fw_name("ath11k/WCN6855/hw2.1/amss.bin", NULL, tiny, sizeof(tiny)) == EINVAL);   /* does not fit */

	/* the QMI board-id rule (lx_policy.h) gives names that this table accepts as they are */
	CHECK(lx_board_auto_name(0xff, 0x12, name, sizeof(name)) == 11 && strcmp(name, "bdwlang.elf") == 0);
	CHECK(lx_board_auto_name(0x0e, 0x12, name, sizeof(name)) == 11 && strcmp(name, "bdwlang.e0e") == 0);
	CHECK(lx_board_auto_name(0xff, 0x02, name, sizeof(name)) == 10 && strcmp(name, "bdwlan.elf") == 0);
	CHECK(lx_board_auto_name(0x2b, 0x00, name, sizeof(name)) == 10 && strcmp(name, "bdwlan.e2b") == 0);
	CHECK(lx_board_auto_name(0x100, 0x00, name, sizeof(name)) == 10 && strcmp(name, "bdwlan.elf") == 0);
	CHECK(lx_board_auto_name(0x0e, 0x12, tiny, sizeof(tiny)) == 0);                       /* does not fit: nothing */
	for (unsigned id = 0; id < 0x100; id++) {
		for (unsigned chip = 0; chip < 2; chip++) {
			char again[80];

			CHECK(lx_board_auto_name(id, chip ? 0x12 : 0x02, name, sizeof(name)) != 0);
			CHECK(hakios_wifi_fw_name(name, NULL, again, sizeof(again)) == 0 && strcmp(again, name) == 0);
		}
	}
}

/* ---------------------------------------------------------------- MAC sources (lx_policy.h) */

static void
test_mac_sources(void)
{
	struct lx_mac_inputs in;
	uint8_t answer[6], other[6], zero[6] = { 0 };
	bool valid;
	char text[160];

	memset(&in, 0, sizeof(in));
	memset(answer, 0xee, 6);

	/* 1. the firmware's per-radio address wins, and ath11k is not given another to choose */
	in.radio_valid = true; in.platform_valid = true; memcpy(in.platform, "\x02\x01\x02\x03\x04\x05", 6);
	memcpy(in.ready, "\x02\x09\x09\x09\x09\x09", 6);
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_FIRMWARE_RADIO && !valid && answer[0] == 0xee);
	/* 2. else wifi_mac, given to ath11k to take */
	in.radio_valid = false;
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_WIFI_MAC && valid && memcmp(answer, in.platform, 6) == 0);
	/* 3. else WMI READY's: ath11k's own, nothing to answer */
	in.platform_valid = false;
	memset(answer, 0xee, 6);
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_WMI_READY && !valid && answer[0] == 0xee);
	/* a wifi_mac that is zero or a group address is no source */
	in.platform_valid = true; memset(in.platform, 0, 6);
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_WMI_READY && !valid);
	memcpy(in.platform, "\x01\x00\x5e\x00\x00\x01", 6);
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_WMI_READY && !valid);
	/* 4. all zero: a derived address, locally administered, never multicast, never zero, the same every time */
	in.platform_valid = false; memset(in.ready, 0, 6);
	in.seed[0] = 0x1234abcd; in.seed[1] = 0x12; in.seed[2] = 0x400c1211; in.seed[3] = 0xff;
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_DERIVED && valid);
	CHECK((answer[0] & 0x02) != 0 && (answer[0] & 0x01) == 0 && memcmp(answer, zero, 6) != 0);
	memcpy(other, answer, 6);
	memset(answer, 0, 6);
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_DERIVED && valid && memcmp(answer, other, 6) == 0);        /* stable */
	in.seed[0] ^= 1;                                                      /* another chip: another address */
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_DERIVED && memcmp(answer, other, 6) != 0);
	memset(in.seed, 0, sizeof(in.seed));                                  /* even with no identity at all */
	CHECK(lx_mac_choose(&in, answer, &valid) == LX_MAC_DERIVED && valid && (answer[0] & 3) == 2 && memcmp(answer, zero, 6) != 0);
	/* the derivation over many seeds: always locally administered, unicast */
	for (uint32_t i = 0; i < 5000; i++) {
		uint32_t seed[4] = { i * 2654435761u, i, ~i, i << 7 };

		lx_mac_derive(seed, answer);
		CHECK((answer[0] & 3) == 2);
	}
	/* the sources are named; the names carry no address */
	CHECK_STR(lx_mac_source_name(LX_MAC_FIRMWARE_RADIO), "firmware per-radio address");
	CHECK_STR(lx_mac_source_name(LX_MAC_WIFI_MAC), "wifi_mac");
	CHECK_STR(lx_mac_source_name(LX_MAC_WMI_READY), "wmi ready address");
	CHECK_STR(lx_mac_source_name(LX_MAC_DERIVED), "derived locally administered address");
	snprintf(text, sizeof(text), "%s%s%s%s", lx_mac_source_name(LX_MAC_FIRMWARE_RADIO), lx_mac_source_name(LX_MAC_WIFI_MAC),
	    lx_mac_source_name(LX_MAC_WMI_READY), lx_mac_source_name(LX_MAC_DERIVED));
	CHECK(strchr(text, ':') == NULL);
}

/* ---------------------------------------------------------------- the chain with a fake machine */

static struct {
	char events[4096];                      /* what was done to the machine, in order, comma separated */
	char stage_log[2048];                   /* "n+" or "n-" for each stage line said, in order */
	char detail[14][200];
	unsigned waits;
	int fail_pcie, fail_smmu, fail_pool, fail_driver, fail_scan, fail_join, bss;
	int driver_script;                      /* what the "lx layer" says during driver_start: see fake_driver_start */
	bool have_amss, have_m3, have_regdb, have_board, have_network;
	bool bypass_seen;
	unsigned lock_depth, stage_while_unlocked;
} fake;

static void
event(const char *what)
{
	snprintf(fake.events + strlen(fake.events), sizeof(fake.events) - strlen(fake.events), "%s%s", fake.events[0] ? "," : "", what);
}

static void
f_lock(void *ctx)
{
	fake.lock_depth++;
}

static void
f_unlock(void *ctx)
{
	fake.lock_depth--;
}

static void
f_wait(void *ctx, unsigned ms)
{
	fake.waits++;
}

static void
f_stage(void *ctx, unsigned n, int ok, const char *detail)
{
	if (fake.lock_depth == 0) {
		fake.stage_while_unlocked++;
	}
	snprintf(fake.stage_log + strlen(fake.stage_log), sizeof(fake.stage_log) - strlen(fake.stage_log), "%u%c", n, ok ? '+' : '-');
	snprintf(fake.detail[n], sizeof(fake.detail[n]), "%s", detail);
	/* the name the kernel prints is the table's: every stage 1-13 has one */
	CHECK(strcmp(stage_name(n), "?") != 0);
}

static bool
f_file_there(void *ctx, const char *name)
{
	if (strcmp(name, "amss20.bin") == 0) {
		return fake.have_amss;
	}
	if (strcmp(name, "m3.bin") == 0) {
		return fake.have_m3;
	}
	if (strcmp(name, "regdb.bin") == 0) {
		return fake.have_regdb;
	}
	return fake.have_board;
}

static bool
f_network_there(void *ctx)
{
	return fake.have_network;
}

static int
f_pcie_up(void *ctx)
{
	event("pcie_up");
	if (fake.fail_pcie) {
		return fake.fail_pcie;
	}
	/* C2 says its three stages itself, through C1, not through the chain */
	return 0;
}

static void
f_pcie_down(void *ctx)
{
	event("pcie_down");
}

static int
f_smmu_up(void *ctx, bool bypass)
{
	event(bypass ? "smmu_up(bypass)" : "smmu_up");
	fake.bypass_seen = bypass;
	return fake.fail_smmu;
}

static void
f_smmu_down(void *ctx)
{
	event("smmu_down");
}

static int
f_pool_up(void *ctx)
{
	event("pool_up");
	return fake.fail_pool;
}

static void
f_pool_down(void *ctx)
{
	event("pool_down");
}

/* What the lx layer reports from inside lx_start(), by script: the stages it says and in which order, then the answer. */
static int
f_driver_start(void *ctx)
{
	event("driver_start");
	switch (fake.driver_script) {
	case 0:         /* the whole of it, in order */
		hakios_wifi_pci_report(5, 1, "probe"); hakios_wifi_pci_report(6, 1, "mhi: EE AMSS"); hakios_wifi_pci_report(7, 1, "qrtr");
		hakios_wifi_pci_report(8, 1, "qmi"); hakios_wifi_pci_report(9, 1, "wmi");
		break;
	case 1:         /* nothing said at all, and it works */
		break;
	case 2:         /* the probe fails before MHI is prepared: nobody says anything */
		break;
	case 3:         /* the probe is fine and MHI stalls: stage 6 FAILED */
		hakios_wifi_pci_report(5, 1, "probe"); hakios_wifi_pci_report(6, 0, "stall");
		break;
	case 4:         /* up to the QRTR link, then nothing: the first stage nobody said is 8 */
		hakios_wifi_pci_report(5, 1, "probe"); hakios_wifi_pci_report(6, 1, "mhi"); hakios_wifi_pci_report(7, 1, "qrtr");
		break;
	case 5:         /* only the last one is said: the missing ones are made up for, in order, once */
		hakios_wifi_pci_report(9, 1, "wmi");
		hakios_wifi_pci_report(9, 1, "wmi again");
		hakios_wifi_pci_report(6, 1, "late mhi");
		break;
	case 6:         /* duplicates: a later power-up says the same stage again */
		hakios_wifi_pci_report(5, 1, "probe"); hakios_wifi_pci_report(5, 1, "probe"); hakios_wifi_pci_report(6, 1, "mhi");
		hakios_wifi_pci_report(6, 1, "mhi"); hakios_wifi_pci_report(7, 1, "q"); hakios_wifi_pci_report(8, 1, "q");
		hakios_wifi_pci_report(9, 1, "w");
		break;
	}
	return fake.fail_driver;
}

static void
f_driver_stop(void *ctx)
{
	event("driver_stop");
}

static int
f_scan(void *ctx, unsigned *bss)
{
	event("scan");
	*bss = (unsigned)fake.bss;
	return fake.fail_scan;
}

static int
f_join(void *ctx, int *last)
{
	event("join");
	*last = fake.fail_join ? 15 : 0;
	return fake.fail_join;
}

static const struct hakios_wifi_pci_ops fake_ops = {
	.ctx = NULL, .lock = f_lock, .unlock = f_unlock, .wait_ms = f_wait, .stage = f_stage, .file_there = f_file_there,
	.network_there = f_network_there, .pcie_up = f_pcie_up, .pcie_down = f_pcie_down, .smmu_up = f_smmu_up,
	.smmu_down = f_smmu_down, .pool_up = f_pool_up, .pool_down = f_pool_down, .driver_start = f_driver_start,
	.driver_stop = f_driver_stop, .scan = f_scan, .join = f_join,
};

static void
fresh(void)
{
	memset(&fake, 0, sizeof(fake));
	fake.have_amss = fake.have_m3 = fake.have_regdb = fake.have_board = fake.have_network = true;
	fake.bss = 19;
	hakios_wifi_pci_begin(NULL);
	stage_ops = NULL;
}

static struct hakios_wifi_pci_run run;

static int
chain(const struct hakios_wifi_words *w)
{
	memset(&run, 0, sizeof(run));
	return hakios_wifi_pci_start(w, &fake_ops, &run);
}

static struct hakios_wifi_words
good_words(void)
{
	struct hakios_wifi_words w;

	reset();
	SET(wifi, has_wifi, "1"); SET(soc, has_soc, "8450"); SET(join, has_join, "1"); SET(keep, has_keep, "1");
	SET(board, has_board, "bdwlang.b0e"); SET(ssid, has_ssid, "HomeNet"); SET(psk, has_psk, "correct horse 42");
	CHECK(hakios_wifi_words_parse(&raw, &w, &werr) == 0);
	return w;
}

static void
test_chain(void)
{
	struct hakios_wifi_words w = good_words();

	/* everything works: stages in order, once each; nothing torn down while the radio is kept */
	fresh();
	CHECK(chain(&w) == 0);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan,join");
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+11+");          /* (1-3 are C2's own lines; the chain only notes them) */
	CHECK(hakios_wifi_pci_seen(1) == 1 && hakios_wifi_pci_seen(2) == 1 && hakios_wifi_pci_seen(3) == 1);
	CHECK(run.up == (UP_PCIE | UP_SMMU | UP_POOL | UP_DRIVER));
	CHECK(strstr(fake.detail[10], "scan: 19 BSS") != NULL);
	CHECK(fake.stage_while_unlocked == 0 && fake.lock_depth == 0);   /* every line is said under the lock that orders them */
	CHECK(fake.waits == 0);
	/* ... and torn down in reverse of what was made (driver, memory, context, link), once, and again for nothing */
	hakios_wifi_pci_teardown(&fake_ops, &run);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan,join,driver_stop,pool_down,smmu_down,pcie_down");
	hakios_wifi_pci_teardown(&fake_ops, &run);
	CHECK(strstr(fake.events, "pcie_down,") == NULL && run.up == 0);

	/* no join asked for: stage 11 is not said, and join is not called */
	fresh();
	w.join = false;
	CHECK(chain(&w) == 0);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan");
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+");
	w.join = true;

	/* wifi_smmu=bypass reaches the context step as a diagnostic, and says so */
	fresh();
	w.bypass = true;
	CHECK(chain(&w) == 0 && fake.bypass_seen && strstr(fake.detail[4], "bypass") != NULL);
	w.bypass = false;

	/* the lx layer's stage lines: duplicates once; a missing one made up before a later one, in order */
	fresh(); fake.driver_script = 6;
	CHECK(chain(&w) == 0);
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+11+");
	fresh(); fake.driver_script = 5;
	CHECK(chain(&w) == 0);
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+11+");            /* 5-8 inferred, in order, before 9 */
	CHECK(strstr(fake.detail[7], "inferred") != NULL && strstr(fake.detail[9], "wmi") != NULL && fake.waits > 0);
	fresh(); fake.driver_script = 1;
	CHECK(chain(&w) == 0);
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+11+");            /* a driver that said nothing but worked: all inferred */

	/* nothing powered when what the run needs is missing: the stage's own FAILED line, no hardware step */
	fresh(); fake.have_amss = false;
	CHECK(chain(&w) == 6 && strcmp(fake.events, "") == 0 && strcmp(fake.stage_log, "6-") == 0 && run.up == 0);
	fresh(); fake.have_m3 = false;
	CHECK(chain(&w) == 8 && strcmp(fake.events, "") == 0 && strcmp(fake.stage_log, "8-") == 0);
	fresh(); fake.have_regdb = false;
	CHECK(chain(&w) == 8 && strcmp(fake.events, "") == 0);
	fresh(); fake.have_board = false;                           /* wifi_board names a file that is not there: never another */
	CHECK(chain(&w) == 8 && strcmp(fake.events, "") == 0 && strstr(fake.detail[8], "wifi_board") != NULL);
	w.has_board = false;
	fresh(); fake.have_board = false;                           /* (no override: the board is chosen by id later) */
	CHECK(chain(&w) == 0);
	w.has_board = true;
	fresh(); fake.have_network = false;                         /* wifi_join=1 and no network at all */
	{
		struct hakios_wifi_words nonet = w;

		memset(nonet.ssid, 0, sizeof(nonet.ssid)); nonet.ssid_length = 0;
		CHECK(chain(&nonet) == 11 && strcmp(fake.events, "") == 0 && strcmp(fake.stage_log, "11-") == 0);
		fresh(); fake.have_network = true;                      /* ... but fw/network will do */
		CHECK(chain(&nonet) == 0);
	}

	/* a failure at each stage prints its FAILED line and undoes exactly what was made, in reverse */
	fresh(); fake.fail_pcie = EIO;                              /* C2 undid itself and said which of 1-3 */
	CHECK(chain(&w) == 1);
	CHECK_STR(fake.events, "pcie_up");
	CHECK_STR(fake.stage_log, "");
	CHECK(run.up == 0);

	fresh(); fake.fail_smmu = EBUSY;
	CHECK(chain(&w) == 4);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pcie_down");
	CHECK_STR(fake.stage_log, "4-");

	fresh(); fake.fail_pool = ENOMEM;
	CHECK(chain(&w) == 4);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,smmu_down,pcie_down");   /* (the pool never came up: not taken down) */
	CHECK_STR(fake.stage_log, "4-");
	CHECK(strstr(fake.detail[4], "pool") != NULL);

	fresh(); fake.driver_script = 2; fake.fail_driver = ENODEV;       /* ath11k's probe fails before MHI: stage 5 */
	CHECK(chain(&w) == 5);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,pool_down,smmu_down,pcie_down");
	CHECK_STR(fake.stage_log, "4+5-");

	fresh(); fake.driver_script = 3; fake.fail_driver = ETIMEDOUT;    /* MHI stalls: stage 6, said by the lx layer, once */
	CHECK(chain(&w) == 6);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,pool_down,smmu_down,pcie_down");
	CHECK_STR(fake.stage_log, "4+5+6-");

	fresh(); fake.driver_script = 4; fake.fail_driver = EIO;          /* no QMI: the first stage nobody said, 8 */
	CHECK(chain(&w) == 8);
	CHECK_STR(fake.stage_log, "4+5+6+7+8-");
	CHECK(strstr(fake.detail[8], "errno 5") != NULL);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,pool_down,smmu_down,pcie_down");

	fresh(); fake.fail_scan = ETIMEDOUT;                               /* the driver is up and has to come down first */
	CHECK(chain(&w) == 10);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan,driver_stop,pool_down,smmu_down,pcie_down");
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10-");

	fresh(); fake.bss = 0;                                             /* it heard nothing */
	CHECK(chain(&w) == 10);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan,driver_stop,pool_down,smmu_down,pcie_down");
	CHECK(strstr(fake.detail[10], "0 BSS") != NULL);

	fresh(); fake.fail_join = ETIMEDOUT;
	CHECK(chain(&w) == 11);
	CHECK_STR(fake.events, "pcie_up,smmu_up,pool_up,driver_start,scan,join,driver_stop,pool_down,smmu_down,pcie_down");
	CHECK_STR(fake.stage_log, "4+5+6+7+8+9+10+11-");
	CHECK(strstr(fake.detail[11], "last error 15") != NULL);

	/* a stage is said once even when the chain and the lx layer both think they should */
	fresh(); fake.driver_script = 3; fake.fail_driver = EIO;
	chain(&w);
	CHECK(strchr(fake.stage_log, '5') == strrchr(fake.stage_log, '5'));

	/* the passphrase is in none of the lines said, whatever happened */
	for (unsigned script = 0; script < 7; script++) {
		fresh(); fake.driver_script = (int)script;
		fake.fail_join = script == 6 ? EIO : 0;
		chain(&w);
		for (unsigned n = 1; n <= 13; n++) {
			CHECK(strstr(fake.detail[n], "correct horse") == NULL && strstr(fake.detail[n], "HomeNet") == NULL);
		}
	}
	/* ... and no MAC-like text either (six hex pairs with colons) */
	for (unsigned n = 1; n <= 13; n++) {
		const char *d = fake.detail[n];

		for (size_t i = 0; d[i] != 0 && strlen(d + i) >= 17; i++) {
			bool mac = true;

			for (unsigned k = 0; k < 17 && mac; k++) {
				mac = (k % 3 == 2) ? d[i + k] == ':' : isxdigit((unsigned char)d[i + k]) != 0;
			}
			CHECK(!mac);
		}
	}
}

/* the stage table is the header's: names of the stages the chain says */
static void
test_names(void)
{
	CHECK_STR(stage_name(1), "rails"); CHECK_STR(stage_name(4), "smmu"); CHECK_STR(stage_name(5), "probe");
	CHECK_STR(stage_name(6), "mhi"); CHECK_STR(stage_name(7), "qrtr"); CHECK_STR(stage_name(8), "qmi");
	CHECK_STR(stage_name(9), "wmi"); CHECK_STR(stage_name(10), "scan"); CHECK_STR(stage_name(11), "join");
	CHECK_STR(stage_name(13), "settings"); CHECK_STR(stage_name(14), "?");
}

int
main(void)
{
	test_words();
	printf("words: ok\n");
	test_mac_parse();
	printf("wifi_mac parser: ok\n");
	test_firmware();
	printf("firmware table and board rule: ok\n");
	test_mac_sources();
	printf("MAC sources: ok\n");
	test_names();
	test_chain();
	printf("chain, stage order and teardown: ok\n");
	printf("wifi-int-test: %d checks, %d failed\n", checks, fails);
	return fails != 0;
}
