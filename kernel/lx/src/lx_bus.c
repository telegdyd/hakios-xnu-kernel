/*
 * The radio's bus, for as long as ath11k's own (ahb.c, pcic.c) is not built: what stands between
 * the driver's core and this phone's WCN6750. It does what ahb.c's probe does for this chip, in
 * its order, with what the files built so far need of it -- the chip's registers through the
 * three fixed windows, and a thread that looks at the copy engines every millisecond where the
 * driver would have their interrupts -- and it is where the driver's calls into the parts not
 * yet built are given something to do.
 *
 * Where this stage ends: qmi.c has its exchange with the firmware, core.c starts the firmware
 * in mission mode and has ce.c and hal.c set up the copy engines' rings, htc.c connects its
 * services over them and wmi.c has the firmware say what the radio is ("service ready") and,
 * given the driver's configuration, that it is ready for commands ("unified ready"); mac.c
 * registers the radio with what stands in for mac80211 (lx_80211.c) and sets the data path's
 * rings up. What the firmware said is then put in the log. With the boot-arg wifi_scan=1 the
 * station of lx_80211.c has the radio scan, and the driver is taken down again the way its own
 * removal does it (ath11k_core_deinit()); with wifi_join=1 the station joins the network the
 * kernel was given, and the driver stays up, carrying the kernel's frames, until lx_stop().
 *
 * The kernel's boot-arg wifi_lx says how far the radio's registers are touched: 1, not at all
 * (every write is said and dropped, every read is 0: the driver's own exchange and nothing
 * else); 2, for real.
 *
 * What the phone said, 13 October: all of that happens, 270 ms from the start of the bus to
 * "unified ready", with the copy engines looked at once a millisecond, and the scan hears the
 * networks around (board/wifi.c, step_driver(), has the account).
 */
#define LX_QMI_AS_IT_IS				/* (this file is where the driver's requests go past) */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/skbuff.h>

#include "core.h"
#include "ce.h"
#include "debug.h"
#include "hal.h"
#include "hif.h"
#include "htc.h"
#include "pcic.h"
#include "qmi.h"

#include <lx_os.h>
#include "lx.h"
#include "lx_policy.h"
#ifdef CONFIG_ATH11K_PCI
#include <linux/pci.h>
#endif

/* The normal ath11k WoW command sends these tuples alongside its IPv4 ARP offload. */
void lx_bus_wow_ns(struct ieee80211_vif *vif, const u8 addresses[][16], unsigned count);
void
lx_bus_wow_ns(struct ieee80211_vif *vif, const u8 addresses[][16], unsigned count)
{
	struct ath11k_vif *arvif = ath11k_vif_to_arvif(vif);
	struct ath11k_arp_ns_offload *offload = &arvif->arp_ns_offload;

	count = min_t(unsigned, count, ARRAY_SIZE(offload->ipv6_addr));
	memset(offload->ipv6_addr, 0, sizeof(offload->ipv6_addr));
	memset(offload->self_ipv6_addr, 0, sizeof(offload->self_ipv6_addr));
	memset(offload->ipv6_type, ATH11K_IPV6_UC_TYPE, sizeof(offload->ipv6_type));
	memcpy(offload->mac_addr, vif->addr, ETH_ALEN);
	for (unsigned i = 0; i < count; i++) {
		memcpy(offload->ipv6_addr[i], addresses[i], 16);
		offload->self_ipv6_addr[i][0] = 0xff;
		offload->self_ipv6_addr[i][1] = 0x02;
		offload->self_ipv6_addr[i][11] = 0x01;
		offload->self_ipv6_addr[i][12] = 0xff;
		memcpy(offload->self_ipv6_addr[i] + 13, addresses[i] + 13, 3);
	}
	offload->ipv6_count = count;
}

static struct {
	unsigned how;				/* wifi_lx */
	bool mission;				/* the firmware has been asked for mission mode, and for no other since */
	bool closed;				/* the driver has stopped the bus: the registers are not touched again */
	unsigned reads, writes, said, refused;
	u32 last, last_value;			/* the register written last, and with what ... */
	unsigned again;				/* ... and how many writes to it in a row have not been said */
	volatile bool looking;			/* the thread that looks at the copy engines is wanted ... */
	volatile bool looks;			/* ... and runs */
	volatile bool data;			/* ... and looks at the data path's rings as well */
	unsigned same;				/* it has written a ring's pointer what it held so many times */

	/* the radio's interrupts (wifi_irq=1): what the copy engines and the data path's groups are taken care of on */
	bool irq;
	struct mutex service;			/* one thing at a time looks at the rings: an interrupt's or the thread */
	int ce_vector[CE_COUNT_MAX];		/* a copy engine's vector, -1: it has none (ce.c: CE_ATTR_DIS_INTR) */
	int group_vector[ATH11K_EXT_IRQ_GRP_NUM_MAX];
	unsigned long long late;		/* work the thread found that no interrupt had brought */
} bus;

/*
 * The radio's interrupt vectors, as pcic.c has them for this chip: 28 of the node's 32, ten for the
 * copy engines from vector 0 and eighteen for the data path's groups from vector 10.
 */
static struct ath11k_msi_user msi_users[] = {
	{ .name = "CE", .num_vectors = 10, .base_vector = 0 },
	{ .name = "DP", .num_vectors = 18, .base_vector = 10 },
};
static void irq_say(void);

static const struct ath11k_msi_config msi_config = {
	.total_vectors = 28,
	.total_users = 2,
	.users = msi_users,
	.hw_rev = ATH11K_HW_WCN6750_HW10,
};

/* ---- what the driver says ---------------------------------------------------------------------- */

static void
driver_says(const char *kind, const char *format, va_list list)
{
	char line[320];
	size_t length;

	vsnprintf(line, sizeof(line), format, list);
	length = strlen(line);
	/* (Linux ends a line for whoever has not) */
	printf("ath11k%s: %s%s", kind, line, length != 0 && line[length - 1] == '\n' ? "" : "\n");
}

void
ath11k_info(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_says("", format, list);
	va_end(list);
}

/* (the driver's warnings and errors are said whatever happens to the rest) */
static void
driver_complains(const char *kind, const char *format, va_list list)
{
	char line[320];

	vsnprintf(line, sizeof(line), format, list);
	lx_tell("ath11k %s: %s%s", kind, line, line[0] != 0 && line[strlen(line) - 1] == '\n' ? "" : "\n");
}

void
ath11k_warn(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_complains("WARNS", format, list);
	va_end(list);
}

void
ath11k_err(struct ath11k_base *ab, const char *format, ...)
{
	va_list list;

	va_start(list, format);
	driver_complains("ERROR", format, list);
	va_end(list);
}

/* doze (lx_80211.c): the firmware's reason for each WoW wake */
void lx_doze_reason(int reason, const char *name);
void lx_doze_note(const char *format);
extern volatile bool lx_doze_in_call;	/* lx_80211.c: inside the doze's suspend, resume or undo */

/*
 * After the driver's probe, before QMI starts the firmware: no firmware log over WMI (hw_params.fw_wmi_diag_event,
 * QMI WLAN_INI enablefwlog). Its WMI_DIAG events, which the driver only traces, came a few a second and each is a
 * message to the host: in M3 an interrupt that ended every doze (run50, run51). wifi_fwlog=1 keeps it.
 */
#ifdef CONFIG_ATH11K_PCI
void *lx_pci_base(void);
void lx_bus_probed(void);
void
lx_bus_probed(void)
{
	struct ath11k_base *ab = lx_pci_base();

	if (ab != NULL && !lx_os_option("wifi_fwlog")) ab->hw_params.fw_wmi_diag_event = false;
}

/*
 * The flags of WMI_WOW_ENABLE (ath11k_wmi_wow_enable sends 0; kernel/lx/ath11k-clock-source.py has it ask here). How the
 * radio asks for the host during WoW is the firmware's choice by the link's state (amss20.bin's own lines: "PERST GPIO
 * line is HIGH. Not asserting WAKE#, CHECK D3 State", "D3 Hot State, PERST HIGH. Issue PME#", "Endpoint assert WAKE#
 * since PERST is LOW"): in DRV sleep PERST# stays high and the endpoint in D0, so without being told it asserts nothing
 * (run53..60: no WAKE#, no MSI, every wake the host's). Qualcomm's fw-api wmi_unified.h: WMI_WOW_FLAG_ENABLE_DRV_PCIE_
 * L1SS_SLEEP = 0x10, "Enable L1SS sleep for PCIE DRV case" -- the radio's L1ss sleep mode, whose wake is an MSI
 * through the DSP-held link ("Pagedone MSI for L1SS Sleep sent to HOST"). With the link off (PERST# low) it asserts
 * WAKE#, as it is: 0 then.
 */
#define LX_WMI_WOW_FLAG_ENABLE_DRV_PCIE_L1SS_SLEEP 0x10u
/* the vector of the radio's wake MSI given in the host capabilities; ~0u: none */
unsigned lx_wake_msi_vector = ~0u;
unsigned lx_wow_flags_sent;
u32 lx_wow_enable_flags(void);
u32
lx_wow_enable_flags(void)
{
	lx_wow_flags_sent = lx_os_pcie_sleep_mode() == 1 && lx_wake_msi_vector != ~0u ? LX_WMI_WOW_FLAG_ENABLE_DRV_PCIE_L1SS_SLEEP : 0;
	return lx_wow_flags_sent;
}

/*
 * The receive frames ath11k dropped, by its own counts (soc_stats, as debugfs.c's soc_dp_stats says them): the error ring,
 * buffers of no known pool, each RXDMA and REO error code (dp_rx.c drops all but TKIP MIC and a missing REO queue). Said
 * as "; rx dropped by the driver: ..." into text; codes with a count only. (run69: the LAN's broadcasts never reached
 * the host while its multicasts, same group key, did.)
 */
int lx_bus_rx_errors(char *text, size_t size);
int
lx_bus_rx_errors(char *text, size_t size)
{
	struct ath11k_base *ab = lx_pci_base();
	struct ath11k_soc_dp_stats *s;
	int at;

	if (ab == NULL || size == 0) return 0;
	s = &ab->soc_stats;
	at = snprintf(text, size, "; rx dropped by the driver: error ring %u, invalid rbm %u, reo errors", s->err_ring_pkts,
	    s->invalid_rbm);
	for (unsigned i = 0; i < HAL_REO_DEST_RING_ERROR_CODE_MAX && at < (int)size - 16; i++)
		if (s->reo_error[i]) at += snprintf(text + at, size - (size_t)at, " %u:%u", i, s->reo_error[i]);
	at += snprintf(text + at, size - (size_t)at, ", rxdma errors");
	for (unsigned i = 0; i < HAL_REO_ENTR_RING_RXDMA_ECODE_MAX && at < (int)size - 16; i++)
		if (s->rxdma_error[i]) at += snprintf(text + at, size - (size_t)at, " %u:%u", i, s->rxdma_error[i]);
	at += snprintf(text + at, size - (size_t)at, ", hal reo %u %u %u %u", s->hal_reo_error[0], s->hal_reo_error[1],
	    s->hal_reo_error[2], s->hal_reo_error[3]);
	return at < (int)size ? at : (int)size - 1;
}

/* The firmware's receive data filter on each interface (wifi_rx_filter): mode 0 off, else WMI_HW_DATA_FILTER_* bits. */
int lx_bus_rx_filter(unsigned mode);
int
lx_bus_rx_filter(unsigned mode)
{
	struct ath11k_base *ab = lx_pci_base();
	struct ath11k *ar;
	struct ath11k_vif *arvif;
	int ret = -ENODEV, r;

	if (ab == NULL || (ar = ath11k_ab_to_ar(ab, 0)) == NULL) return -ENODEV;
	mutex_lock(&ar->conf_mutex);
	list_for_each_entry(arvif, &ar->arvifs, list) {
		r = ath11k_wmi_hw_data_filter_cmd(ar, arvif->vdev_id, mode, mode != 0);
		if (ret == -ENODEV || (r != 0 && ret == 0)) ret = r;
	}
	mutex_unlock(&ar->conf_mutex);
	return ret;
}

/*
 * After ath11k_wow_op_suspend() has failed: what it leaves set undone, step for step (its own unwinding covers the WoW
 * enable and wakeup events only; mac80211 would rebuild the hardware). The hw data filter, the ARP/NS offload and the
 * keepalive: off as ath11k_wow_op_resume() turns them off (sent whatever step failed: each is idempotent). The data
 * path's and copy engines' interrupts: on again only when hif_suspend failed (mhi_was_asked), the one failure that comes
 * after ath11k_hif_irq_disable() and ath11k_hif_ce_irq_disable(). 0, or the first error.
 */
int lx_bus_wow_undo(bool mhi_was_asked);
int
lx_bus_wow_undo(bool mhi_was_asked)
{
	struct ath11k_base *ab = lx_pci_base();
	struct ath11k *ar;
	struct ath11k_vif *arvif;
	int ret = 0, r;

	if (ab == NULL || (ar = ath11k_ab_to_ar(ab, 0)) == NULL) {
		return -ENODEV;
	}
	if (mhi_was_asked) {
		ath11k_hif_ce_irq_enable(ab);
		ath11k_hif_irq_enable(ab);
	}
	mutex_lock(&ar->conf_mutex);
	list_for_each_entry(arvif, &ar->arvifs, list) {
		if ((r = ath11k_wmi_hw_data_filter_cmd(ar, arvif->vdev_id, 0, false)) != 0 && ret == 0) ret = r;
		if (arvif->vdev_type != WMI_VDEV_TYPE_STA) continue;
		if ((r = ath11k_wmi_arp_ns_offload(ar, arvif, false)) != 0 && ret == 0) ret = r;
		if ((r = ath11k_mac_vif_set_keepalive(arvif, WMI_STA_KEEPALIVE_METHOD_NULL_FRAME,
		    WMI_STA_KEEPALIVE_INTERVAL_DISABLE)) != 0 && ret == 0) ret = r;
	}
	mutex_unlock(&ar->conf_mutex);
	return ret;
}
#endif
void
__ath11k_dbg(struct ath11k_base *ab, enum ath11k_debug_mask mask, const char *format, ...)
{
	va_list list;

	if (strcmp(format, "event diag") == 0) {
		return;		/* (the firmware log's events: not said) */
	}
	if (strncmp(format, "wow wakeup host reason", 22) == 0) {
		va_start(list, format);
		int reason = va_arg(list, int);
		const char *name = va_arg(list, const char *);
		va_end(list);
		lx_doze_reason(reason, name);
	}
	if (mask == ATH11K_DBG_WMI && lx_doze_in_call && !lx_os_option("wifi_wmi_debug")) {
		lx_doze_note(format);	/* (counted for the wake's line instead: lx_80211.c doze_seen_say) */
		return;		/* (a doze's WoW setup and teardown, some fifty commands each time, would bury the driver's warnings) */
	}
	va_start(list, format);
	driver_says(" (debug)", format, list);
	va_end(list);
}

void
ath11k_dbg_dump(struct ath11k_base *ab, enum ath11k_debug_mask mask, const char *msg, const char *prefix, const void *buf,
    size_t len)
{
}

/* ---- the chip's registers ---------------------------------------------------------------------- */

/*
 * As ahb.c reaches them (ath11k_ahb_get_window_start_wcn6750()): the first 512K of the block
 * the firmware names are the chip's first 512K; the second 512K show its data path's
 * registers, the third its copy engines'.
 */
static u32
window(struct ath11k_base *ab, u32 offset)
{
	if (offset < ATH11K_PCI_WINDOW_START) {
		return (offset);
	}
	if ((offset ^ HAL_SEQ_WCSS_UMAC_OFFSET) < ATH11K_PCI_WINDOW_RANGE_MASK) {
		return (ATH11K_PCI_WINDOW_START + (offset & ATH11K_PCI_WINDOW_RANGE_MASK));
	}
	if ((offset ^ HAL_SEQ_WCSS_UMAC_CE0_SRC_REG(ab)) < ATH11K_PCI_WINDOW_RANGE_MASK) {
		return (2 * ATH11K_PCI_WINDOW_START + (offset & ATH11K_PCI_WINDOW_RANGE_MASK));
	}
	return (0);				/* (refused() has kept such a one out) */
}

/*
 * The registers answer while the firmware is in mission mode and at no other time: an access
 * outside it ends the kernel (board/wifi.c, step 16). The driver asks for that mode
 * before it first touches them; once it has stopped the bus, on its way to asking for the mode
 * "off", nothing is let through any more.
 */
static bool
refused(struct ath11k_base *ab, u32 offset)
{
	bool known = offset < ATH11K_PCI_WINDOW_START || (offset ^ HAL_SEQ_WCSS_UMAC_OFFSET) < ATH11K_PCI_WINDOW_RANGE_MASK ||
	    (offset ^ HAL_SEQ_WCSS_UMAC_CE0_SRC_REG(ab)) < ATH11K_PCI_WINDOW_RANGE_MASK;

	if (known && bus.mission && !bus.closed) {
		return (false);
	}
	if (bus.refused++ < 8) {
		lx_tell("lx: the chip's %#x is reached for %s: NOT DONE\n", offset, !known ? "which no window shows" : bus.closed ?
		    "after the bus was stopped" : "while the firmware is not in mission mode");
	}
	return (true);
}

/*
 * The accesses are said one by one, up to a number of lines; of writes to one register in a row
 * (a ring's pointer, moved once for every buffer) the first three, and then how many followed.
 */
#define SAID_FIRST	600

static void
say_repeats(void)
{
	if (bus.again != 0) {
		printf("lx: ... and %u more to the chip's %#x, the last %#x\n", bus.again, bus.last, bus.last_value);
		bus.again = 0;
	}
}

static bool
said_write(u32 offset, u32 value)
{
	static struct {
		u32 offset, value;
	} pointers[24];				/* while the copy engines are looked at: what each pointer was moved to last */
	static unsigned row;

	if (bus.looks) {
		for (unsigned i = 0; i < nitems(pointers); i++) {
			if (pointers[i].offset == offset || pointers[i].offset == 0) {
				bool moved = pointers[i].offset == 0 || pointers[i].value != value;

				pointers[i].offset = offset;
				pointers[i].value = value;
				bus.same += !moved;
				return (moved && bus.said < SAID_FIRST);
			}
		}
	}
	row = offset == bus.last ? row + 1 : 0;
	if (row >= 3) {
		bus.again++;
		bus.last_value = value;
		return (false);
	}
	say_repeats();
	bus.last = offset;
	return (bus.said < SAID_FIRST);
}

static u32
bus_read32(struct ath11k_base *ab, u32 offset)
{
	u32 value = 0;

	if (refused(ab, offset)) {
		return (0);
	}
	bus.reads++;
	if (bus.how >= 2 && ab->mem != NULL) {
		value = ioread32((u8 *)ab->mem + window(ab, offset));
	}
	if (bus.said < SAID_FIRST) {
		say_repeats();
		bus.last = ~0u;
		bus.said++;
		printf("lx: the chip's %#x reads %#x%s\n", offset, value, bus.how >= 2 ? "" : " (NOT READ: wifi_lx=1)");
	}
	return (value);
}

static void
bus_write32(struct ath11k_base *ab, u32 offset, u32 value)
{
	if (refused(ab, offset)) {
		return;
	}
	bus.writes++;
	if (said_write(offset, value)) {
		bus.said++;
		printf("lx: %s the chip's %#x (at +%#x of its block): %#x\n", bus.how >= 2 ? "about to write" : "NOT WRITTEN (wifi_lx=1):",
		    offset, window(ab, offset), value);
	}
	if (bus.how >= 2 && ab->mem != NULL) {
		__asm __volatile("dsb sy");	/* (what was written to a ring is in memory before the radio is told of it) */
		iowrite32(value, (u8 *)ab->mem + window(ab, offset));
	}
}

/*
 * In place of the radio's interrupts: each copy engine is looked at as its interrupt would have
 * ath11k do (ce.c: ath11k_ce_per_engine_service(), which takes what has been received to its
 * owner and posts fresh buffers), and once the driver has "enabled the data path's interrupts"
 * so is each of the groups its rings are in (dp.c: ath11k_dp_service_srng(), which is what the
 * driver's NAPI poll calls: frames received, frames that have been sent, errors, the reorder
 * block's answers). Once a millisecond while nothing comes; again at once while a group has
 * used up what it was allowed in one go.
 *
 * With the radio's interrupts (wifi_irq=1) the rings are looked at when one arrives, on the
 * kernel's thread for them (irq_ce(), irq_group() below), and this thread looks only every 100 ms,
 * to count what the interrupts left behind and to take care of it.
 */
#define DATA_BUDGET	64

/*
 * What the data path threw away and why, where it has: frames the hardware would not pass (rxdma_error: by its
 * error code, HAL_REO_ENTR_RING_RXDMA_ECODE_ -- 3 deciphering failed, 4 Michael's check, 5 not enciphered) and frames
 * the reorder engine would not (reo_error: HAL_REO_DEST_RING_ERROR_CODE_ -- 12 a packet number out of order).
 */
static void
errors_say(struct ath11k_base *ab)
{
	const struct ath11k_soc_dp_stats *st = &ab->soc_stats;
	char line[400];
	int at = 0;

	for (int i = 0; i < HAL_REO_ENTR_RING_RXDMA_ECODE_MAX; i++) {
		if (st->rxdma_error[i] != 0 && at < (int)sizeof(line) - 40) {
			at += snprintf(line + at, sizeof(line) - at, " rxdma %d: %u", i, st->rxdma_error[i]);
		}
	}
	for (int i = 0; i < HAL_REO_DEST_RING_ERROR_CODE_MAX; i++) {
		if (st->reo_error[i] != 0 && at < (int)sizeof(line) - 40) {
			at += snprintf(line + at, sizeof(line) - at, " reo %d: %u", i, st->reo_error[i]);
		}
	}
	lx_tell("lx: frames the data path threw away: %u through the error ring, %u with a wrong owner;%s\n", st->err_ring_pkts,
	    st->invalid_rbm, at != 0 ? line : " none by the hardware or the reorder engine");
}

static void
bus_looks(void *argument)
{
	struct ath11k_base *ab = argument;
	unsigned long long said = lx_os_nanoseconds(), every = 120000000000ull;

	bus.looks = true;
	while (bus.looking) {
		bool more = false;

		if (lx_os_nanoseconds() - said > every) {	/* after two minutes, then every five: how the station and memory stand */
			said = lx_os_nanoseconds();
			every = 300000000000ull;
			lx_station_say();
			errors_say(ab);
			lx_memory_say();
			if (bus.irq) {
				irq_say();
			}
		}

		if (bus.irq) {
			mutex_lock(&bus.service);
		}
		for (unsigned i = 0; i < ab->hw_params.ce_count; i++) {
			ath11k_ce_per_engine_service(ab, i);
		}
		for (unsigned i = 0; bus.data && i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
			int done = ath11k_dp_service_srng(ab, &ab->ext_irq_grp[i], DATA_BUDGET);

			more |= done >= DATA_BUDGET;
			bus.late += bus.irq && done > 0;
		}
		if (bus.irq) {
			mutex_unlock(&bus.service);
		}
		if (!more) {
			lx_pause_ms(bus.irq ? 100 : 1);
		}
	}
	bus.looks = false;
}

/*
 * The copy engines are looked at no more, and no register is touched from here on. This is what
 * the driver's "interrupts off" comes to here, and it comes in time: on its way down the driver
 * says that after its last word to the firmware over a copy engine and before it asks for the
 * mode "off".
 */
static void
bus_quiet(void)
{
	if (bus.irq && bus.looking) {
		irq_say();
	}
	if (bus.irq) {
		lx_os_irq_detach_all();
	}
	bus.data = false;
	bus.looking = false;
	while (bus.looks) {
		lx_pause_ms(1);
	}
	say_repeats();
}

/*
 * A request of the driver's to the firmware, on its way out (linux/soc/qcom/qmi.h, ours). One
 * matters here: the mode. With any mode but mission the radio's registers stop answering, and
 * the driver does not always let go of the copy engines first -- where its start fails late,
 * core.c asks for the mode "off" and stops the bus after that (ath11k_core_stop()). So the
 * copy engines are let go of here, before the request leaves.
 */
int
lx_qmi_send_request(struct qmi_handle *qmi, struct sockaddr_qrtr *sq, struct qmi_txn *txn, int msg_id, size_t largest,
    const struct qmi_elem_info *ei, const void *structure)
{
#ifdef CONFIG_ATH11K_PCI
	/*
	 * Host capabilities on the OnePlus 10 Pro: ath11k tells a WCN6855 that it runs on a non-Qualcomm platform and is to
	 * use its internal sleep clock (nm_modem: HOST_CSTATE, SLEEP_CLOCK_SELECT_INTERNAL, PCIE_GLOBAL_RESET, PME_D3COLD).
	 * Here the host is a Qualcomm SoC that feeds the chip its sleep clock, and Android's cnss2 sends no nm_modem at all
	 * (iOSPlus10 0263, t21: HOST_CAP without it, all of QMI passed). Round 1 (2026-11-12): with it, SYS_ERR right after
	 * RESPOND_MEM. So the word is left out, as cnss2 does; wifi_hostcap_ath=1 sends ath11k's own.
	 */
	if (msg_id == QMI_WLANFW_HOST_CAP_REQ_V01 && lx_os_radio_model() == 1 && !lx_os_option("wifi_hostcap_ath")) {
		struct qmi_wlanfw_host_cap_req_msg_v01 req = *(const struct qmi_wlanfw_host_cap_req_msg_v01 *)structure;

		lx_tell("lx: host capabilities as Android's cnss2 sends them: nm_modem %#x left out (wifi_hostcap_ath=1 keeps it)\n",
		    req.nm_modem_valid ? req.nm_modem : 0);
		req.nm_modem_valid = 0;
		req.nm_modem = 0;
		/*
		 * The vector the radio is to ask for the host with (cnss2: cnss_pci_get_wake_msi, "WAKE MSI base data is %d"):
		 * ath11k's MSI layout has a WAKE user (pcic.c, vector 13 of 32) but never says it to the firmware, nor listens
		 * on it. In DRV sleep (PERST# high, D0) the firmware asserts neither WAKE# nor PME#: this MSI is its way, and
		 * without it, with WMI_WOW_FLAG_ENABLE_DRV_PCIE_L1SS_SLEEP, it crashed in WoW (run61: MHI SYS_ERR in a doze).
		 * lx_pci.c's poll takes the vector as the doze's wake.
		 */
		{
			struct ath11k_base *ab = lx_pci_base();
			int vectors = 0;
			u32 data = 0, base = 0;

			if (ab != NULL && ath11k_pcic_get_user_msi_assignment(ab, "WAKE", &vectors, &data, &base) == 0 && vectors > 0) {
				req.wake_msi_valid = 1;
				req.wake_msi = data;
				lx_wake_msi_vector = base;
				lx_tell("lx: host capabilities: the radio's wake MSI is vector %u (data %u)\n", base, data);
			}
		}
		return (qmi_send_request(qmi, sq, txn, msg_id, largest, ei, &req));
	}
#endif
	if (msg_id == QMI_WLANFW_WLAN_MODE_REQ_V01) {
		u32 mode = ((const struct qmi_wlanfw_wlan_mode_req_msg_v01 *)structure)->mode;

		if (mode != ATH11K_FIRMWARE_MODE_NORMAL && bus.looking) {
			printf("lx: the driver asks for mode %u while the copy engines are still looked at: they are let go of first\n",
			    mode);
		}
		if (mode != ATH11K_FIRMWARE_MODE_NORMAL) {
			bus_quiet();
		}
		bus.mission = mode == ATH11K_FIRMWARE_MODE_NORMAL;
		lx_tell("lx: the firmware is asked for mode %u (0: mission, 4: off, 7: calibration)\n", mode);
#ifdef CONFIG_ATH11K_PCI
		/*
		 * WIFI-PLAN stage 8 (PCIe path): qmi.c has been through host capabilities, the firmware's memory, the board data,
		 * the regulatory database and M3, and has the firmware's "ready"; asking for mission mode is core.c's next move.
		 */
		if (mode == ATH11K_FIRMWARE_MODE_NORMAL && lx_os_radio_model() == 1) {
			struct ath11k_base *ab = lx_pci_base();

			if (ab != NULL) {
				lx_os_stage(8, "qmi", 1, "qmi: host capabilities, memory, board data (board id %#x), regdb, M3, fw ready",
				    ab->qmi.target.board_id);
			}
		}
#endif
	}
	return (qmi_send_request(qmi, sq, txn, msg_id, largest, ei, structure));
}

/* ---- the radio's interrupts ---------------------------------------------------------------------- */

/* A copy engine's interrupt (pcic.c: ath11k_pcic_ce_interrupt_handler() and its tasklet). */
static void
irq_ce(void *argument)
{
	struct ath11k_ce_pipe *pipe = argument;

	mutex_lock(&bus.service);
	ath11k_ce_per_engine_service(pipe->ab, pipe->pipe_num);
	mutex_unlock(&bus.service);
	lx_os_irq_enable(bus.ce_vector[pipe->pipe_num]);
}

/* A data path group's (pcic.c: ath11k_pcic_ext_interrupt_handler() and its NAPI poll, until it has less to do than allowed). */
static void
irq_group(void *argument)
{
	struct ath11k_ext_irq_grp *group = argument;
	int done;

	mutex_lock(&bus.service);
	do {
		done = bus.data ? ath11k_dp_service_srng(group->ab, group, DATA_BUDGET) : 0;
	} while (done >= DATA_BUDGET);
	mutex_unlock(&bus.service);
	if (bus.data) {
		lx_os_irq_enable(bus.group_vector[group->grp_id]);
	}
}

/* pcic.c: ath11k_pcic_config_irq(), for this chip: which vector is whose, and the functions for them, all masked */
static int
irq_attach(struct ath11k_base *ab)
{
	unsigned index = 0;
	int ret;

	for (unsigned i = 0; i < CE_COUNT_MAX; i++) {
		bus.ce_vector[i] = -1;
	}
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		bus.group_vector[i] = -1;
	}
	for (unsigned i = 0; i < ab->hw_params.ce_count; i++) {
		if (ath11k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR) {
			continue;
		}
		bus.ce_vector[i] = (int)(index++ % msi_users[0].num_vectors + msi_users[0].base_vector);
		if ((ret = lx_os_irq_attach((unsigned)bus.ce_vector[i], irq_ce, &ab->ce.ce_pipe[i])) != 0) {
			return (ret);
		}
	}
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		const struct ath11k_hw_ring_mask *mask = ab->hw_params.ring_mask;

		ab->ext_irq_grp[i].ab = ab;
		ab->ext_irq_grp[i].grp_id = i;
		if (!(mask->tx[i] || mask->rx[i] || mask->rx_err[i] || mask->rx_wbm_rel[i] || mask->reo_status[i] ||
		    mask->rxdma2host[i] || mask->host2rxdma[i] || mask->rx_mon_status[i])) {
			continue;
		}
		bus.group_vector[i] = (int)(i % msi_users[1].num_vectors + msi_users[1].base_vector);
		if ((ret = lx_os_irq_attach((unsigned)bus.group_vector[i], irq_group, &ab->ext_irq_grp[i])) != 0) {
			return (ret);
		}
	}
	return (0);
}

/* What came, by vector: for the log. */
static void
irq_say(void)
{
	char line[400] = "";

	for (unsigned v = 0; v < msi_config.total_vectors; v++) {
		unsigned long long count = lx_os_irq_count(v);

		if (count != 0) {
			snprintf(line + strlen(line), sizeof(line) - strlen(line), " %u: %llu,", v, count);
		}
	}
	lx_tell("lx: the radio's interrupts by vector (copy engines 0-9, the data path's groups 10-27):%s the thread found %llu "
	    "times work no interrupt had brought\n", line[0] ? line : " NONE", bus.late);
}

static int
bus_get_user_msi_vector(struct ath11k_base *ab, char *user, int *count, u32 *user_base_data, u32 *base_vector)
{
	for (unsigned i = 0; i < nitems(msi_users); i++) {
		if (strcmp(user, msi_users[i].name) == 0) {
			*count = (int)msi_users[i].num_vectors;
			*base_vector = msi_users[i].base_vector;
			*user_base_data = msi_users[i].base_vector + ab->pci.msi.ep_base_data;
			return (0);
		}
	}
	return (-EINVAL);
}

static void
bus_get_msi_address(struct ath11k_base *ab, u32 *low, u32 *high)
{
	*low = ab->pci.msi.addr_lo;
	*high = ab->pci.msi.addr_hi;
}

/* pcic.c: ath11k_pcic_get_ce_msi_idx(): the copy engines with interrupts, counted */
static void
bus_get_ce_msi_idx(struct ath11k_base *ab, u32 ce_id, u32 *index)
{
	u32 n = 0;

	for (u32 i = 0; i < ab->hw_params.ce_count && i != ce_id; i++) {
		n += !(ath11k_ce_get_attr_flags(ab, i) & CE_ATTR_DIS_INTR);
	}
	*index = n;
}

/* pcic.c: ath11k_pcic_start(), with the thread above where it enables the interrupts */
static int
bus_start(struct ath11k_base *ab)
{
	int ret;

	set_bit(ATH11K_FLAG_DEVICE_INIT_DONE, &ab->dev_flags);
	printf("lx: the driver starts the bus: buffers for the copy engines that receive (%u reads and %u writes of the chip's "
	    "registers so far; the configuration the firmware was sent named %u shadow registers)\n", bus.reads, bus.writes,
	    ab->qmi.ce_cfg.shadow_reg_v2_len);
	ath11k_ce_rx_post_buf(ab);
	say_repeats();
	lx_tell("lx: the rings are set up and the copy engines have their buffers: the bus is started\n");
	/*
	 * (here and not when the driver is set up: the firmware is asked for calibration in between, and
	 * the copy engines are let go of for that, interrupts and all; see lx_qmi_send_request())
	 */
	if (bus.irq && (ret = irq_attach(ab)) != 0) {
		lx_tell("lx: wifi_irq: the radio's interrupts cannot be had (%d): its rings are looked at every 100 ms only\n", ret);
		lx_os_irq_detach_all();
	}
	if (bus.irq) {
		for (unsigned i = 0; i < CE_COUNT_MAX; i++) {
			if (bus.ce_vector[i] >= 0) {
				lx_os_irq_enable((unsigned)bus.ce_vector[i]);
			}
		}
	}
	bus.looking = true;
	if (lx_os_thread("wifi-rings", bus_looks, ab) != 0) {
		bus.looking = false;
		return (-ENOMEM);
	}
	return (0);
}

/* pcic.c: ath11k_pcic_stop() */
static void
bus_stop(struct ath11k_base *ab)
{
	bus_quiet();
	bus.closed = true;
	timer_delete_sync(&ab->rx_replenish_retry);
	ath11k_ce_cleanup_pipes(ab);
	printf("lx: the driver stops the bus (%u reads and %u writes of the chip's registers in all, %u of the writes a ring's "
	    "pointer written what it held)\n", bus.reads, bus.writes, bus.same);
}

/* pcic.c: ath11k_pcic_map_service_to_pipe(): which copy engines carry a service, by the chip's table */
static int
bus_map_service_to_pipe(struct ath11k_base *ab, u16 service, u8 *up, u8 *down)
{
	bool has_up = false, has_down = false;

	for (u32 i = 0; i < ab->hw_params.svc_to_ce_map_len; i++) {
		const struct service_to_pipe *entry = &ab->hw_params.svc_to_ce_map[i];
		u32 direction = __le32_to_cpu(entry->pipedir);

		if (__le32_to_cpu(entry->service_id) != service) {
			continue;
		}
		if (direction == PIPEDIR_IN || direction == PIPEDIR_INOUT) {
			*down = (u8)__le32_to_cpu(entry->pipenum);
			has_down = true;
		}
		if (direction == PIPEDIR_OUT || direction == PIPEDIR_INOUT) {
			*up = (u8)__le32_to_cpu(entry->pipenum);
			has_up = true;
		}
	}
	return (has_up && has_down ? 0 : -ENOENT);
}

static int
bus_power_up(struct ath11k_base *ab)
{
	return (-lx_os_rproc_boot());
}

static void
bus_power_down(struct ath11k_base *ab, bool is_suspend)
{
}

/* pcic.c: ath11k_pcic_ext_irq_enable(): the data path's rings are set up, and are looked at from here on */
static void
bus_irq_enable(struct ath11k_base *ab)
{
	for (unsigned i = 0; i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		ab->ext_irq_grp[i].ab = ab;
		ab->ext_irq_grp[i].grp_id = i;
	}
	bus.data = true;
	for (unsigned i = 0; bus.irq && i < ATH11K_EXT_IRQ_GRP_NUM_MAX; i++) {
		if (bus.group_vector[i] >= 0) {
			lx_os_irq_enable((unsigned)bus.group_vector[i]);
		}
	}
}

static void
bus_irq_disable(struct ath11k_base *ab)
{
	bus_quiet();
}

static const struct ath11k_hif_ops bus_ops = {
	.read32 = bus_read32,
	.write32 = bus_write32,
	.irq_enable = bus_irq_enable,
	.irq_disable = bus_irq_disable,
	.start = bus_start,
	.stop = bus_stop,
	.map_service_to_pipe = bus_map_service_to_pipe,
	.power_up = bus_power_up,
	.power_down = bus_power_down,
	/* (no interrupt vectors are offered: the rings are then set up without any, see ce.c) */
};

/* ... and with the radio's interrupts: the vectors, and where the radio writes to raise one */
static const struct ath11k_hif_ops bus_ops_irq = {
	.read32 = bus_read32,
	.write32 = bus_write32,
	.irq_enable = bus_irq_enable,
	.irq_disable = bus_irq_disable,
	.start = bus_start,
	.stop = bus_stop,
	.map_service_to_pipe = bus_map_service_to_pipe,
	.power_up = bus_power_up,
	.power_down = bus_power_down,
	.get_user_msi_vector = bus_get_user_msi_vector,
	.get_msi_address = bus_get_msi_address,
	.get_ce_msi_idx = bus_get_ce_msi_idx,
};

/* ---- the device tree, of which there is none ------------------------------------------------------ */

int
of_property_read_string(const struct device_node *np, const char *name, const char **out)
{
	return (-EINVAL);
}

bool
of_machine_is_compatible(const char *compatible)
{
	return (false);
}

int
of_reserved_mem_region_to_resource(const struct device_node *np, unsigned int index, struct resource *res)
{
	return (-ENODEV);
}

/*
 * The radio's own address, which the driver asks its device for before it falls back on what the
 * firmware says (mac.c: ath11k_mac_register()); this radio's firmware says none. The kernel has
 * it, if it was given one. Nothing here says the address.
 */
static enum lx_mac_source mac_source;
static bool mac_source_known;

int
linuxkpi_device_get_mac_address(struct device *dev, char *address)
{
#ifdef CONFIG_ATH11K_PCI
	if (lx_os_radio_model() == 1) {
		/*
		 * The PCIe path. ath11k asks after WMI's READY, and keeps its own order (per-radio addresses of the firmware, then
		 * what is answered here, then READY's address): so this answers the kernel's wifi_mac, if there is one and the
		 * firmware's per-radio addresses are not valid, and a derived locally administered address when all three are zero
		 * (lx_policy.h). Which of them ended up the radio's is kept for stage 9: the source, never the address.
		 */
		struct ath11k_base *ab = lx_pci_base();
		struct lx_mac_inputs in;
		uint8_t answer[6];
		bool valid;

		if (ab != NULL) {
			memset(&in, 0, sizeof(in));
			in.radio_valid = ab->pdevs_macaddr_valid;
			in.platform_valid = lx_os_radio_address(in.platform) != 0;
			memcpy(in.ready, ab->mac_addr, 6);
			in.seed[0] = lx_mhi_serial();
			in.seed[1] = ab->qmi.target.chip_id;
			in.seed[2] = ab->qmi.target.soc_id;
			in.seed[3] = ab->qmi.target.board_id;
			mac_source = lx_mac_choose(&in, answer, &valid);
			mac_source_known = true;
			explicit_bzero(in.platform, sizeof(in.platform));
			if (valid) {
				memcpy(address, answer, 6);
				explicit_bzero(answer, sizeof(answer));
				return (0);
			}
			return (-ENOENT);
		}
	}
#endif
	return (lx_os_radio_address((unsigned char *)address) ? 0 : -ENOENT);
}

/*
 * lx_base.c, for a request of board.bin that the kernel answered with ENODEV (no wifi_board): the board data file chosen by the
 * QMI board id, which is known by now (lx_policy.h, cnss2's rule). Its own answer: 0, or ENOENT (never another board's file).
 */
int
lx_firmware_by_board(const void **data, unsigned long *size)
{
#ifdef CONFIG_ATH11K_PCI
	struct ath11k_base *ab = lx_pci_base();
	char board[24];

	if (ab != NULL && lx_board_auto_name(ab->qmi.target.board_id, ab->qmi.target.chip_id, board, sizeof(board)) != 0) {
		lx_tell("lx: no wifi_board: the board data is chosen by the QMI board id %#x and chip id %#x: %s\n",
		    ab->qmi.target.board_id, ab->qmi.target.chip_id, board);
		return (lx_os_firmware(board, data, size));
	}
#endif
	return (ENOENT);
}

/* For stage 9 (lx_80211.c): which source the radio's address came from, in words; "" before it is known. */
const char *
lx_mac_source(void)
{
	return (mac_source_known ? lx_mac_source_name(mac_source) : "");
}

/* ---- what the firmware has said of the radio ------------------------------------------------------- */

/* After "unified ready": what wmi.c has kept of the firmware's two announcements. */
static void
say_radio(struct ath11k_base *ab)
{
	const struct ath11k_targ_cap *t = &ab->target_caps;

	printf("lx: THE FIRMWARE SAYS OF THE RADIO: %u radio(s); %u chains (mask %#x); up to %u channels in a scan, %u MACs; "
	    "transmit power %d..%d; phy %#x, HT %#x, VHT %#x (MCS %#x), system %#x\n", ab->num_radios, t->num_rf_chains,
	    t->txrx_chainmask, t->max_num_scan_channels, t->max_supported_macs, (int)t->hw_min_tx_power, (int)t->hw_max_tx_power,
	    t->phy_capability, t->ht_cap_info, t->vht_cap_info, t->vht_supp_mcs, t->sys_cap_info);
	/* (the second number is from the same structure of "ready" as the address: it shows that was read) */
	if (lx_os_radio_model() == 1) {
		/* (the PCIe path never says an address, not the firmware's either; stage 9 names where the radio's came from) */
		printf("lx: its check sum of the packet log's definitions: %#x\n", ab->pktlog_defs_checksum);
	} else {
		printf("lx: its address, by the firmware: %02x:%02x:%02x:%02x:%02x:%02x (and its check sum of the packet log's "
		    "definitions: %#x)\n", ab->mac_addr[0], ab->mac_addr[1], ab->mac_addr[2], ab->mac_addr[3], ab->mac_addr[4],
		    ab->mac_addr[5], ab->pktlog_defs_checksum);
	}
	for (unsigned i = 0; i < ab->num_radios && i < MAX_RADIOS; i++) {
		const struct ath11k_hal_reg_capabilities_ext *r = &ab->hal_reg_cap[i];
		const struct ath11k_pdev_cap *c = &ab->pdevs[i].cap;

		printf("lx: radio %u (pdev %u, phy %u): bands%s%s (%#x); 2.4 GHz %u..%u MHz, 5 GHz and up %u..%u MHz; regulatory "
		    "domain %#x, wireless modes %#x; chains tx %#x rx %#x; VHT %#x, A-MPDU density %u\n", i, ab->pdevs[i].pdev_id,
		    r->phy_id, c->supported_bands & WMI_HOST_WLAN_2G_CAP ? " 2.4 GHz" : "", c->supported_bands &
		    WMI_HOST_WLAN_5G_CAP ? " 5 GHz" : "", c->supported_bands, r->low_2ghz_chan, r->high_2ghz_chan, r->low_5ghz_chan,
		    r->high_5ghz_chan, r->eeprom_reg_domain, r->wireless_modes, c->tx_chain_mask, c->rx_chain_mask, c->vht_cap,
		    c->ampdu_density);
		for (unsigned band = 0; band < NUM_NL80211_BANDS; band++) {
			const struct ath11k_band_cap *b = &c->band[band];

			if (b->max_bw_supported == 0 && b->ht_cap_info == 0 && b->he_cap_info[0] == 0) {
				continue;
			}
			printf("lx: radio %u, band %u (0: 2.4 GHz, 1: 5 GHz, 3: 6 GHz): widest channel code %u, HT %#x, HE %#x %#x "
			    "(MCS %#x), HE at 6 GHz %#x\n", i, band, b->max_bw_supported, b->ht_cap_info, b->he_cap_info[0],
			    b->he_cap_info[1], b->he_mcs, b->he_6ghz_capa);
		}
	}
}

/* ---- the entries ---------------------------------------------------------------------------------- */

static struct ath11k_base *volatile driver;	/* the driver, while it is up: between lx_start() and lx_stop() */
static unsigned transmitters;

/* Close admission before freeing the radio; a queued Darwin transmit may be in it. */
static void driver_close(void)
{
	lx_os_lock();
	driver = NULL;
	while (transmitters) lx_os_sleep(&transmitters, 1000000ull);
	lx_os_unlock();
}

/* Full off/on for Settings' POWER request. Keep the static station boundary:
 * withdrawing it here would wait for this very callback to finish. State
 * readers are excluded by board/sta.c, and driver_close drains transmitters.
 * The boot chain retains its SMMU/pool ownership until normal teardown. */
int lx_pci_runtime_power(bool on)
{
#ifdef CONFIG_ATH11K_PCI
	struct ath11k_base *ab;
	unsigned long long began;
	int ret;
	if (!on) {
		lx_os_runtime_state(1, 0);
		if (driver) {
			driver_close();
			lx_station_leave();
			lx_pci_driver_exit();
		}
		ret = lx_os_pcie_down_status();
		lx_os_runtime_state(ret ? 5 : 0, ret);
		lx_tell("lx: runtime PCIe off: status %d\n", ret);
		return ret;
	}
	if (driver) return 0;
	lx_os_runtime_state(2, 0);
	if ((ret = lx_os_pcie_down_status()) != 0) return ret;
	if ((ret = lx_prepare()) != 0) return ret;
	ret = lx_pci_driver_init();
	if (ret) return -ret;
	lx_tell("lx: runtime PCIe probe returned; waiting for registered firmware\n");
	ab = lx_pci_base();
	if (!ab) { lx_pci_driver_exit(); return EIO; }
	began = lx_os_nanoseconds();
	unsigned diagnostic = 0;
	while (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) &&
	    !test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags) &&
	    lx_os_nanoseconds() - began < 60000000000ull) {
		if (lx_os_nanoseconds() - began > (diagnostic + 1ull) * 10000000000ull) {
			diagnostic++;
			lx_tell("lx: runtime firmware wait %u s flags %#lx\n", diagnostic * 10, ab->dev_flags);
			lx_os_qrtr_diagnostic(); lx_mhi_diagnostic();
		}
		lx_pause_ms(10);
	}
	if (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) ||
	    test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags)) {
		lx_pci_driver_exit(); return EIO;
	}
	lx_os_lock(); driver = ab; lx_os_unlock();
	(void)lx_os_pcie_pm_start();
	lx_os_runtime_state(3, 0);
	lx_tell("lx: runtime PCIe reprobe ready in %llu ms\n", (lx_os_nanoseconds() - began) / 1000000ull);
	return 0;
#else
	return EOPNOTSUPP;
#endif
}

/*
 * lx_start() runs on the steps' thread and lx_stop() on whoever ends the run (board/wifi.c), so a stop can come while a
 * start is on its way. Under lx_os_lock(): that a start is, and that a stop has come meanwhile. The start then goes no
 * further and takes down what it has brought up, and the stop waits for it to have done so. The frames lx_transmit()
 * has in the driver are counted, so that the driver is not taken down under one.
 */
static struct {
	bool starting;
	bool stop;
	volatile int sending;
} life;

static void driver_down(void);

/* The run ends, or a stop has come: the start goes no further. */
static bool
start_cut_short(void)
{
	bool stop;

	lx_os_lock();
	stop = life.stop;
	lx_os_unlock();
	return (stop || lx_os_option("wifi_ending"));
}

static int
bring_up(void)
{
	static struct device device;
	static struct device_node node;
	struct ath11k_base *ab;
	unsigned long long began;
	bool up;
	int ret;

	bus.closed = false;     /* (a start opens what an earlier stop -- of a driver that was never up, say -- closed) */
#ifdef CONFIG_ATH11K_PCI
	if (lx_os_radio_model() == 1) {
		if ((ret = lx_prepare()) != 0) return ret;
		/* as the AHB path below: what the driver says about QMI, its boot, WMI, regulatory and mac80211, unless wifi_quiet */
		ath11k_debug_mask = lx_os_option("wifi_quiet") ? 0 :
		    ATH11K_DBG_QMI | ATH11K_DBG_BOOT | ATH11K_DBG_PCI | ATH11K_DBG_WMI | ATH11K_DBG_REG | ATH11K_DBG_MAC;
		ath11k_frame_mode = ATH11K_HW_TXRX_ETHERNET;
		ret = lx_pci_driver_init();
		if (ret) return -ret;
		ab = lx_pci_base();
		if (!ab) { lx_pci_driver_exit(); return EIO; }
		began = lx_os_nanoseconds();
		while (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) &&
		    !test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags) &&
		    lx_os_nanoseconds() - began < 60000000000ull) lx_pause_ms(10);
		if (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) ||
		    test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags)) {
			lx_pci_driver_exit(); return EIO;
		}
		driver = ab;
		(void)lx_os_pcie_pm_start();
		say_radio(ab);
		if (lx_os_option("wifi_station")) ret = lx_station_offer();
		else if (lx_os_option("wifi_join")) ret = lx_station_join();
		else ret = lx_os_option("wifi_scan") ? lx_station_scan() : 0;
		if (ret || !lx_os_option("wifi_keep")) driver_down();      /* (inside the start: lx_stop() would wait for it) */
		return ret;
	}
#endif
	bus.how = lx_os_option("wifi_lx");
	if (lx_prepare() != 0) {
		lx_tell("lx: no thread, or no memory the radio can be given\n");
		return (ENOMEM);
	}
	/* what the driver has to say while it is being looked into; not in a session that just uses the radio */
	if (lx_os_option("wifi_quiet")) {
		ath11k_debug_mask = 0;
		bus.said = SAID_FIRST;
	} else {
		ath11k_debug_mask = ATH11K_DBG_QMI | ATH11K_DBG_BOOT | ATH11K_DBG_AHB | ATH11K_DBG_WMI | ATH11K_DBG_REG | ATH11K_DBG_MAC;
	}
	/*
	 * The driver's parameter "frame_mode": Ethernet. The firmware then puts frames into 802.11
	 * and takes them out of it, and ciphers them: what passes between the driver and the layer
	 * above is what goes over a cable.
	 */
	ath11k_frame_mode = ATH11K_HW_TXRX_ETHERNET;
	device.of_node = &node;
	device.kobj.name = "wifi";

	/* ahb.c: ath11k_ahb_probe(), for this chip */
	ab = ath11k_core_alloc(&device, 0, ATH11K_BUS_AHB);
	if (ab == NULL) {
		return (ENOMEM);
	}
	ab->hif.ops = &bus_ops;
	bus.irq = false;
	if (lx_os_option("wifi_irq")) {
		unsigned long long address;
		unsigned data;

		/* (pcic.c: ath11k_pcic_init_msi_config() and what ahb.c takes from the device tree for this chip) */
		if (lx_os_irq_msi(&address, &data)) {
			mutex_init(&bus.service);
			ab->pci.msi.config = &msi_config;
			ab->pci.msi.addr_lo = (u32)address;
			ab->pci.msi.addr_hi = (u32)(address >> 32);
			ab->pci.msi.ep_base_data = data;
			set_bit(ATH11K_FLAG_MULTI_MSI_VECTORS, &ab->dev_flags);
			ab->hif.ops = &bus_ops_irq;
			bus.irq = true;
		} else {
			lx_tell("lx: wifi_irq: the radio cannot be shown where to raise an interrupt: the rings are looked at, as before\n");
		}
	}
	ab->hw_rev = ATH11K_HW_WCN6750_HW10;
	ab->fw_mode = ATH11K_FIRMWARE_MODE_NORMAL;
	if ((ret = ath11k_core_pre_init(ab)) != 0 || (ret = ath11k_hal_srng_init(ab)) != 0 || (ret = ath11k_ce_alloc_pipes(ab)) != 0) {
		lx_tell("lx: the driver does not get as far as its exchange with the firmware: %d\n", ret);
		return (-ret);
	}
	ab->qmi.ce_cfg.tgt_ce_len = ab->hw_params.target_ce_count;
	ab->qmi.ce_cfg.tgt_ce = ab->hw_params.target_ce_config;
	ab->qmi.ce_cfg.svc_to_ce_map_len = ab->hw_params.svc_to_ce_map_len;
	ab->qmi.ce_cfg.svc_to_ce_map = ab->hw_params.svc_to_ce_map;
	ab->qmi.service_ins_id = ab->hw_params.qmi_service_ins_id;
	printf("lx: the driver (\"%s\") is set up; its exchange with the firmware begins (registers: %s)\n", ab->hw_params.name,
	    bus.how >= 2 ? "for real" : "NOT TOUCHED, wifi_lx=1");
	if ((ret = ath11k_core_init(ab)) != 0) {
		lx_tell("lx: the driver's start fails: %d\n", ret);
		return (-ret);
	}

	/*
	 * The rest happens on the driver's own threads, until core.c has been through its start (it
	 * says so with the flag below, whether that went well or not) or the exchange has failed.
	 * It went well if the bus is still up: core.c stops it where it gives up. A run that ends
	 * meanwhile ("wifi_ending", board/wifi.c), or an lx_stop() that comes, does not wait for it:
	 * what was started is wound down below, before the DSP under it is shut down.
	 */
	began = lx_os_nanoseconds();
	while (!test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) && !test_bit(ATH11K_FLAG_QMI_FAIL, &ab->dev_flags) &&
	    lx_os_nanoseconds() - began < 60000000000ull && !start_cut_short()) {
		lx_pause_ms(10);
	}
	up = test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) && bus.looks;
	lx_tell("lx: %llu ms after its start the driver %s; its flags: %#lx\n", (lx_os_nanoseconds() - began) / 1000000ull,
	    up ? "IS UP as far as it is built: the firmware has said \"unified ready\"" : test_bit(ATH11K_FLAG_REGISTERED,
	    &ab->dev_flags) ? "HAS GIVEN UP on the way and wound itself down" : "HAS NOT GOT THROUGH its exchange with the firmware",
	    ab->dev_flags);
	if (!up) {
		lx_pause_ms(600);		/* (for core.c to have wound down: the mode "off") */
		/*
		 * (core.c may have started the bus -- mission mode asked for -- and left the rings' thread at the copy engines: it
		 * is stopped, and the registers closed, before the DSP is shut down under them: a register of a radio without its
		 * DSP does not answer, and the phone resets without a word)
		 */
		bus_quiet();
		bus.closed = true;
		ath11k_qmi_deinit_service(ab);
		return (test_bit(ATH11K_FLAG_REGISTERED, &ab->dev_flags) ? ETIMEDOUT : EIO);
	}
	lx_pause_ms(1000);			/* (for what the firmware says unasked once it is ready) */
	say_radio(ab);

	if (start_cut_short()) {
		/* (the run has ended meanwhile: the driver is not taken up but goes down again at once, before its DSP does) */
		ath11k_core_deinit(ab);
		lx_tell("lx: the run ends: the driver is down again\n");
		return (EIO);
	}
	lx_os_lock();
	driver = ab;
	lx_os_unlock();
	if (lx_os_option("wifi_station")) {
		/* whoever wants a network asks the kernel, and the kernel asks the station: the driver stays up until lx_stop() */
		if ((ret = lx_station_offer()) == 0) {
			return (0);
		}
	} else if (lx_os_option("wifi_join")) {
		if ((ret = lx_station_join()) == 0) {
			if (lx_os_option("wifi_hold")) {	/* (seconds to stay before returning: to see frames pass) */
				lx_pause_ms(1000 * lx_os_option("wifi_hold"));
				lx_station_say();
			}
			return (0);			/* the network is joined, and the driver stays up until lx_stop() */
		}
	} else {
		ret = lx_os_option("wifi_scan") ? lx_station_scan() : 0;
	}
	driver_down();
	return (ret);
}

int
lx_start(void)
{
	bool stop;
	int ret;

	lx_os_lock();
	if (life.starting || driver != NULL) {
		lx_os_unlock();
		lx_tell("lx: the driver is up already, or on its way: not started again\n");
		return (EBUSY);
	}
	life.starting = true;
	life.stop = false;
	lx_os_unlock();
	ret = bring_up();
	/* (a stop that came while the start was on its way: what the start left up goes down before the stop is let go on) */
	lx_os_lock();
	stop = life.stop;
	lx_os_unlock();
	if (stop) {
		driver_down();
		ret = ret == 0 ? EIO : ret;
	}
	lx_os_lock();
	life.starting = false;
	lx_os_wakeup(&life);
	lx_os_unlock();
	return (ret);
}

/* The thread that counts the ticks, and the memory the radio is shown: once. 0, or an errno. */
int
lx_prepare(void)
{
	return (lx_base_init() != 0 || lx_pool_init() != 0 ? ENOMEM : 0);
}

/*
 * The network left, and the driver down as its own removal does it (the copy engines are let go of on the way: bus_quiet()).
 * The driver is taken from where the others find it under the lock: of two callers, one takes it down.
 */
static void
driver_down(void)
{
	struct ath11k_base *ab;
	lx_station_withdraw(); /* also withdraw the retained runtime-off boundary */

	lx_os_lock();
	ab = driver;
	driver = NULL;
	lx_os_unlock();
	if (ab == NULL) {
		/* (a start that gave up, or one cut short: whatever of the bus it started is stopped, and the registers closed,
		 * before the DSP is shut down) */
		bus_quiet();
		bus.closed = true;
		return;
	}
	driver_close();
	lx_station_leave();
	#ifdef CONFIG_ATH11K_PCI
 if (lx_os_radio_model() == 1) lx_pci_driver_exit();
 else
#endif
 ath11k_core_deinit(ab);
 lx_tell("lx: the driver is down again\n");
}

/*
 * A stop while a start is on its way has the start go no further, and waits for it to have taken down what it brought
 * up: the driver is then neither freed under a scan or a join of the start's nor left up on a radio whose DSP is about to
 * go. Ten seconds at the most: a start still on its way after that has the bus stopped and the registers closed under
 * it, as before, and takes the driver down itself when it gets back.
 */
void
lx_stop(void)
{
	bool starting;

	lx_os_lock();
	if (life.starting) {
		life.stop = true;
		for (unsigned slices = 0; life.starting && slices < 2000; slices++) {
			lx_os_sleep(&life, 5 * 1000000ull);
		}
	}
	starting = life.starting;
	lx_os_unlock();
	if (starting) {
		lx_tell("lx: the driver's start goes on past the wait for it: the bus is stopped under it\n");
		bus_quiet();
		bus.closed = true;
		return;
	}
	driver_down();
}

/* (counted while it is in the driver: driver_down() waits for that) */
int
lx_transmit(const unsigned char *frame, unsigned long length, unsigned flags)
{
	int ret;
	lx_os_lock();
	if (!driver) { lx_os_unlock(); return ENETDOWN; }
	transmitters++;
	lx_os_unlock();
	ret = lx_station_send(frame, length, flags);
	lx_os_lock();
	transmitters--;
	lx_os_wakeup(&transmitters);
	lx_os_unlock();
	return ret;
}
