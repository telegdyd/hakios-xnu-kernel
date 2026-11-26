/*
 * The fastboot reboot driver (drivers/board/reboot.c) and kern.hakios.loader_request dispatch
 * (drivers/board/board.c), run as a host program against the PMIC arbiter model (spmi_model.h)
 * and a mocked IMEM word.
 *
 *   reboot_driver_test SCENARIO      one scenario a process
 */
#include <unistd.h>

#include "fakekernel.h"

extern char log_text[];

#include "spmi_model.h"

static struct spmi_peripheral sdam_model, other_model;
static uint32_t imem_word;
static uint64_t imem_phys;
static unsigned imem_reads, imem_writes;

static int bus_held;
void hakios_spmi_lock(void) { bus_held++; }
void hakios_spmi_unlock(void) { bus_held--; }

static bool interrupts_enabled = true;
boolean_t ml_set_interrupts_enabled(boolean_t enable)
{
	boolean_t prev = interrupts_enabled ? TRUE : FALSE;

	interrupts_enabled = enable != FALSE;
	return prev;
}

static unsigned s3908_quiesces, buttons_quiesces, adsp_stops, psci_calls;
static uint64_t last_psci_fn;
static bool psci_saw_irqs_off, psci_saw_quiesced;
static int last_loader_mode, loader_calls;

vm_offset_t ml_io_map(vm_offset_t phys, vm_size_t size __unused) { return phys; }

uint32_t hakios_test_rd(vm_offset_t address)
{
	if (imem_phys != 0 && address == imem_phys) {
		imem_reads++;
		return imem_word;
	}
	return spmi_model_rd(address);
}

void hakios_test_wr(vm_offset_t address, uint32_t value)
{
	if (imem_phys != 0 && address == imem_phys) {
		imem_writes++;
		imem_word = value;
		return;
	}
	spmi_model_wr(address, value);
}

#pragma GCC diagnostic ignored "-Wformat"
#define KERNEL 1
#define HAKIOS_MMIO_TEST 1
#include "board/board.c"
#include "board/reboot.c"

void hakios_s3908_quiesce(void) { s3908_quiesces++; }
void hakios_buttons_quiesce(void) { buttons_quiesces++; }
void hakios_adsp_stop(void) { adsp_stops++; }
void loader_request_return(int mode)
{
	loader_calls++;
	last_loader_mode = mode;
}

int64_t psci_smp_call(uint64_t fn, uint64_t a1 __unused, uint64_t a2 __unused, uint64_t a3 __unused)
{
	psci_calls++;
	last_psci_fn = fn;
	psci_saw_irqs_off = !interrupts_enabled;
	psci_saw_quiesced = s3908_quiesces > 0 && buttons_quiesces > 0 && adsp_stops > 0;
	return 0;
}

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; fprintf(stderr, "FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
	fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } } while (0)

static void
setup_phone_keys_v7(uint32_t owner)
{
	const struct hakios_board *b;

	board = "phone-keys";
	b = hakios_board();
	CHECK(b != NULL && b->reboot.sdam == 0x271, "phone-keys has reboot { }");
	if (b == NULL) {
		return;
	}
	spmi.core = b->spmi.core;
	spmi.cnfg = b->spmi.cnfg;
	spmi.channels = b->spmi.channels;
	spmi.ee = b->spmi.ee;
	spmi_version_7(b->spmi.observer, 0, 0, 16);
	sdam_model.id = b->reboot.sdam;
	sdam_model.reg[b->reboot.offset] = 0x01;
	other_model.id = 0x272;
	spmi_channel(2, &other_model, spmi.ee);
	spmi_channel(4, &sdam_model, 1);
	spmi_channel(6, &sdam_model, owner);
	imem_phys = b->reboot.imem;
	imem_word = 0;
}

static void
v7_fastboot_ok(void)
{
	int err;

	setup_phone_keys_v7(2);
	err = hakios_fastboot_prepare();
	CHECK(err == 0, "prepare rc=%d", err);
	CHECK(sdam_model.reg[0x48] == 0x05, "SDAM reason 0x%02x (expected 0x05)", sdam_model.reg[0x48]);
	CHECK(imem_writes == 1 && imem_word == 0x77665500u, "IMEM cookie 0x%08x (writes %u)", imem_word, imem_writes);
	CHECK(bus_held == 0 && spmi.elsewhere == 0 && spmi.past_the_table == 0, "clean SPMI bus");
	CHECK(logged("reboot: fastboot: reboot reason 0x1 -> 0x5 (bootloader, SDAM 0x271 ch 6), IMEM cookie set"),
	    "log: %s", log_text);

	hakios_fastboot_reset();
	CHECK(psci_calls == 1 && last_psci_fn == 0x84000009u, "PSCI SYSTEM_RESET called (%u, %#llx)",
	    psci_calls, (unsigned long long)last_psci_fn);
	CHECK(psci_saw_irqs_off && psci_saw_quiesced, "irqs off (%d) and quiesced (%d) before PSCI",
	    psci_saw_irqs_off, psci_saw_quiesced);
}

static void
wrong_owner(void)
{
	int err;

	setup_phone_keys_v7(1);
	spmi_channel(6, &sdam_model, 1);
	err = hakios_fastboot_prepare();
	CHECK(err == EPERM, "expected EPERM, got %d", err);
	CHECK(spmi.writes == 0 && imem_writes == 0 && sdam_model.reg[0x48] == 0x01, "nothing written");
	CHECK(bus_held == 0, "bus balanced");
}

static void
no_channel(void)
{
	const struct hakios_board *b;
	int err;

	board = "phone-keys";
	b = hakios_board();
	CHECK(b != NULL, "board");
	spmi.core = b->spmi.core;
	spmi.cnfg = b->spmi.cnfg;
	spmi.channels = b->spmi.channels;
	spmi.ee = b->spmi.ee;
	spmi_version_7(b->spmi.observer, 0, 0, 16);
	other_model.id = 0x272;
	spmi_channel(2, &other_model, spmi.ee);
	imem_phys = b->reboot.imem;
	err = hakios_fastboot_prepare();
	CHECK(err == ENODEV, "expected ENODEV, got %d", err);
	CHECK(spmi.writes == 0 && imem_writes == 0, "nothing written");
}

static void
no_reboot_section(void)
{
	int err;

	board = "phone-cmd";
	err = hakios_fastboot_prepare();
	CHECK(err == ENOTSUP, "expected ENOTSUP, got %d", err);
	CHECK(spmi.transfers == 0 && imem_writes == 0, "nothing touched");
}

static void
drop_sdam_write(struct spmi_peripheral *p __unused, uint8_t reg __unused, uint8_t value __unused)
{
	/* ignore write so readback mismatches */
}

static void
readback_mismatch(void)
{
	int err;

	setup_phone_keys_v7(2);
	sdam_model.written = drop_sdam_write;
	err = hakios_fastboot_prepare();
	CHECK(err == EIO, "expected EIO, got %d", err);
	CHECK(imem_writes == 0 && imem_word == 0, "IMEM cookie not written when SDAM readback fails");
	CHECK(bus_held == 0, "bus balanced");
}

static void
dropped_transfer_retry(void)
{
	int err;

	setup_phone_keys_v7(2);
	spmi.drop_reads = 1;
	spmi.drop_writes = 1;
	err = hakios_fastboot_prepare();
	CHECK(err == 0, "retry succeeded (rc=%d)", err);
	CHECK(sdam_model.reg[0x48] == 0x05 && imem_word == 0x77665500u, "reason & cookie written after retry");
	CHECK(bus_held == 0, "bus balanced");
}

static void
sysctl_dispatch(void)
{
	int err;

	setup_phone_keys_v7(2);
	err = try_as_root("loader_request", 2);
	CHECK(err == 0 && loader_calls == 1 && last_loader_mode == 3,
	    "mode 2 promoted to 3 (err=%d calls=%u mode=%d)", err, loader_calls, last_loader_mode);
	CHECK(sdam_model.reg[0x48] == 0x05 && imem_word == 0x77665500u, "reason & cookie written on mode 2->3");

	boot_args = "reboot_fastboot=0";
	sdam_model.reg[0x48] = 0x01;
	imem_word = 0;
	err = try_as_root("loader_request", 2);
	CHECK(err == 0 && loader_calls == 2 && last_loader_mode == 2,
	    "reboot_fastboot=0 keeps mode 2 (err=%d calls=%u mode=%d)", err, loader_calls, last_loader_mode);
	CHECK(sdam_model.reg[0x48] == 0x01 && imem_word == 0, "no fastboot write when reboot_fastboot=0");

	err = try_as_root("loader_request", 3);
	CHECK(err == 0 && loader_calls == 3 && last_loader_mode == 3,
	    "explicit mode 3 works even with reboot_fastboot=0 (err=%d calls=%u mode=%d)", err, loader_calls, last_loader_mode);
	CHECK(sdam_model.reg[0x48] == 0x05 && imem_word == 0x77665500u, "reason & cookie written on explicit mode 3");

	err = try_as_root("loader_request", 4);
	CHECK(err == EINVAL, "mode 4 rejected with EINVAL (%d)", err);
}

int
main(int argc, char **argv)
{
	static const struct { const char *name; void (*run)(void); } scenarios[] = {
		{ "v7_fastboot_ok", v7_fastboot_ok },
		{ "wrong_owner", wrong_owner },
		{ "no_channel", no_channel },
		{ "no_reboot_section", no_reboot_section },
		{ "readback_mismatch", readback_mismatch },
		{ "dropped_transfer_retry", dropped_transfer_retry },
		{ "sysctl_dispatch", sysctl_dispatch },
	};

	if (argc < 2) {
		for (unsigned i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
			printf("%s\n", scenarios[i].name);
		}
		return 0;
	}
	for (unsigned i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
		if (strcmp(argv[1], scenarios[i].name) == 0) {
			scenarios[i].run();
			printf("%-24s %s\n", argv[1], failures ? "FAILED" : "ok");
			return failures != 0;
		}
	}
	return 2;
}
