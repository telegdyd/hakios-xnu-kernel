/*
 * en0: a network interface that is Ethernet as far as the rest of the kernel and Darwin can tell.
 *
 * Darwin expects its interfaces from IONetworkingFamily, which this kernel does not have; the BSD
 * half of the kernel does not need it. The interface is made with the kernel's own interface
 * KPI, the way bsd/net/if_fake.c makes its "feth": family Ethernet, the stock Ethernet framing
 * and demultiplexing, one function that is given a frame to send and one call to hand a frame
 * that has arrived to the stack. What Darwin's user space makes of an interface that has no
 * IOKit object behind it is another matter: see userland/src/net_test.m.
 *
 * What carries the frames is a link driver's business; sys/board/net.h is all that the two
 * know of each other. The first link is QEMU's virtio network device (board/virtio_net.c).
 * There can be two interfaces: en0 for the Wi-Fi (or virtio), en1 for the modem's packets
 * through the IPA (board/cell.c); each has its link, its state and its thread.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/kpi_interface.h>
#include <netinet/in.h>
#include <pexpert/pexpert.h>
#include <sys/kern_event.h>
#include <sys/kpi_mbuf.h>
#include <sys/proc.h>
#include <sys/board/net.h>
#include <sys/sockio.h>
#include <sys/systm.h>

extern void HakiosWLANStart(const char *interface, const uint8_t *address);  /* iokit/Kernel/BoardWLAN.cpp */

static LCK_GRP_DECLARE(net_lck_grp, "hakios-net");

/* One interface: enN on its link. */
struct net_if {
	unsigned        unit;
	const struct hakios_net_link *link;
	bool            up;             /* the link says frames can pass */
	ifnet_t         ifp;
	lck_mtx_t       tx_lock;        /* one frame at a time to the link */
	/* What has gone through: said once a minute while anything moves. */
	struct {
		uint64_t sent, received, sent_bytes, received_bytes, dropped_out, dropped_in;
		uint64_t checked, offloaded;    /* received with the link's checksum check; sent with checksums left to the link */
		uint64_t announced;             /* gratuitous ARPs after a link up (net_announce) */
	} count;
};
static struct net_if net_ifs[HAKIOS_NET_UNITS];

struct net_if;
static void net_announce_soon(struct net_if *n);	/* (gratuitous ARP shortly; below) */

static uint64_t
deadline_in_ms(unsigned ms)
{
	uint64_t deadline;

	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	return deadline;
}

static inline struct net_if *
net_if_of(ifnet_t ifp)
{
	return ifnet_softc(ifp);
}

/* The interface on unit `unit`, once attached; NULL before. */
static struct net_if *
net_if_unit(unsigned unit)
{
	return unit < HAKIOS_NET_UNITS && net_ifs[unit].ifp != NULL ? &net_ifs[unit] : NULL;
}

/*
 * Boot-arg net_trace=N: a line in the log for each of the first N frames, either way.
 * Boot-arg net_names=1: a line for every name asked of a DNS server and for every TCP connection
 * begun from here -- which is how to see what a system that has just been given a network does
 * with it (up to NAMES_MAX lines).
 */
#define NAMES_MAX       3000
static uint32_t net_trace, net_names;

static void
trace_frame(const char *way, const uint8_t *frame, size_t length, size_t whole)
{
	static uint32_t said, names;
	const uint8_t *end = frame + length, *ip = frame + ETHER_HDR_LEN, *l4;
	uint16_t type;
	bool ports;

	if (length < ETHER_HDR_LEN + 28 || (said >= net_trace && (!net_names || names >= NAMES_MAX))) {
		return;
	}
	type = (uint16_t)(frame[12] << 8 | frame[13]);
	l4 = ip + 4 * (ip[0] & 0xf);
	ports = type == ETHERTYPE_IP && (ip[9] == IPPROTO_TCP || ip[9] == IPPROTO_UDP) && l4 + 4 <= end;
	if (net_names && names < NAMES_MAX && ports && way[0] == 'o') {
		if (ip[9] == IPPROTO_UDP && (l4[2] << 8 | l4[3]) == 53 && l4 + 8 + 12 < end) {
			char name[128];
			const uint8_t *label = l4 + 8 + 12;
			size_t n = 0;

			/* the question: lengths and labels, up to a zero */
			while (label < end && *label != 0 && *label < 64 && label + 1 + *label < end && n + *label + 2 < sizeof(name)) {
				memcpy(name + n, label + 1, *label);
				n += *label;
				name[n++] = '.';
				label += 1 + *label;
			}
			name[n ? n - 1 : 0] = 0;
			names++;
			printf("net: asks for the name %s (type %u)\n", name, label + 2 < end ? label[1] << 8 | label[2] : 0);
		} else if (ip[9] == IPPROTO_TCP && l4 + 14 <= end && (l4[13] & 0x12) == 0x02) {
			names++;
			printf("net: connects to %u.%u.%u.%u port %u\n", ip[16], ip[17], ip[18], ip[19], l4[2] << 8 | l4[3]);
		}
	}
	if (said >= net_trace) {
		return;
	}
	said++;
	if (type == ETHERTYPE_IP) {
		printf("net: %s %zu bytes: IPv4 %s %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u%s\n", way, whole, ip[9] == IPPROTO_TCP ? "TCP" :
		    ip[9] == IPPROTO_UDP ? "UDP" : ip[9] == IPPROTO_ICMP ? "ICMP" : "protocol?", ip[12], ip[13], ip[14], ip[15],
		    ports ? l4[0] << 8 | l4[1] : 0, ip[16], ip[17], ip[18], ip[19], ports ? l4[2] << 8 | l4[3] : 0,
		    ip[9] == IPPROTO_TCP && ports && l4 + 14 <= end ? (l4[13] & 2 ? " SYN" : l4[13] & 1 ? " FIN" : l4[13] & 4 ? " RST" : "") : "");
	} else {
		printf("net: %s %zu bytes: %s (type %#x) to %02x:%02x:%02x:%02x:%02x:%02x\n", way, whole, type == ETHERTYPE_ARP ? "ARP" :
		    type == ETHERTYPE_IPV6 ? "IPv6" : "?", type, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
	}
}

/* ... of a frame on its way out: the start of it is enough */
static void
trace_packet(mbuf_t packet, size_t length)
{
	uint8_t start[192];
	size_t have = length < sizeof(start) ? length : sizeof(start);

	if ((net_trace || net_names) && mbuf_copydata(packet, 0, have, start) == 0) {
		trace_frame("out", start, have, length);
	}
}

/* ---- the interface -------------------------------------------------------------------------------- */

/* The stack has a frame to send (a chain of mbufs that starts with the Ethernet header). */
static errno_t
net_output(ifnet_t ifp, mbuf_t packet)
{
	struct net_if *n = net_if_of(ifp);
	size_t length = mbuf_pkthdr_len(packet);
	bool sent = false;

	if (length <= HAKIOS_NET_FRAME_MAX && n->up) {
		bpf_tap_out(ifp, DLT_EN10MB, packet, NULL, 0);
		trace_packet(packet, length);
		lck_mtx_lock(&n->tx_lock);
		sent = n->link->transmit(packet, length);
		lck_mtx_unlock(&n->tx_lock);
	}
	if (sent) {
		mbuf_csum_request_flags_t request = 0;
		uint32_t value;

		if (n->link->offload != 0 && mbuf_get_csum_requested(packet, &request, &value) == 0 && request != 0) {
			n->count.offloaded++;
		}
		n->count.sent++;
		n->count.sent_bytes += length;
		ifnet_stat_increment_out(ifp, 1, (uint32_t)length, 0);
	} else {
		n->count.dropped_out++;
		ifnet_stat_increment_out(ifp, 0, 0, 1);
	}
	mbuf_freem(packet);
	return 0;
}

/*
 * What a link's check of a received frame spares the stack: the IPv4 header's checksum, and the TCP or UDP one of a
 * whole datagram (IPv4 not a fragment; IPv6 with TCP or UDP right after the fixed header). A fragment's piece of the
 * datagram's checksum, and what follows IPv6 extension headers, are left for the stack to work out.
 */
static mbuf_csum_performed_flags_t
checked_flags(const uint8_t *frame, size_t length)
{
	const uint8_t *ip = frame + ETHER_HDR_LEN;
	uint16_t type = (uint16_t)(frame[12] << 8 | frame[13]);
	mbuf_csum_performed_flags_t flags = 0;

	if (type == ETHERTYPE_IP && length >= ETHER_HDR_LEN + 20 && (ip[0] >> 4) == 4) {
		flags = MBUF_CSUM_DID_IP | MBUF_CSUM_IP_GOOD;
		if ((ip[6] & 0x3f) == 0 && ip[7] == 0 && (ip[9] == IPPROTO_TCP || ip[9] == IPPROTO_UDP)) {
			flags |= MBUF_CSUM_DID_DATA | MBUF_CSUM_PSEUDO_HDR;
		}
	} else if (type == ETHERTYPE_IPV6 && length >= ETHER_HDR_LEN + 40 && (ip[0] >> 4) == 6 &&
	    (ip[6] == IPPROTO_TCP || ip[6] == IPPROTO_UDP)) {
		flags = MBUF_CSUM_DID_DATA | MBUF_CSUM_PSEUDO_HDR;
	}
	return flags;
}

/* A frame has arrived on unit `unit`. */
void
hakios_net_input_unit(unsigned unit, const uint8_t *frame, size_t length)
{
	hakios_net_input_checked(unit, frame, length, false);
}

void
hakios_net_input_checked(unsigned unit, const uint8_t *frame, size_t length, bool checked)
{
	struct net_if *n = net_if_unit(unit);
	struct ifnet_stat_increment_param stats = { .packets_in = 1, .bytes_in = (uint32_t)length };
	unsigned int chunks = 1;
	mbuf_csum_performed_flags_t csum;
	mbuf_t packet = NULL;
	char *header;

	if (n == NULL) {
		return;
	}
	trace_frame("in ", frame, length, length);
	if (length < ETHER_HDR_LEN || length > HAKIOS_NET_FRAME_MAX || mbuf_allocpacket(MBUF_DONTWAIT, length, &chunks, &packet) != 0) {
		n->count.dropped_in++;
		return;
	}
	if (mbuf_copyback(packet, 0, length, frame, MBUF_DONTWAIT) != 0) {
		mbuf_freem(packet);
		n->count.dropped_in++;
		return;
	}
	if (checked && (csum = checked_flags(frame, length)) != 0) {
		mbuf_set_csum_performed(packet, csum, 0xffff);         /* (with PSEUDO_HDR: the sum is final, all ones) */
		n->count.checked++;
	}
	/* as every Ethernet driver hands a frame up: the header noted and stepped over */
	header = mbuf_data(packet);
	mbuf_pkthdr_setrcvif(packet, n->ifp);
	mbuf_pkthdr_setheader(packet, header);
	mbuf_pkthdr_adjustlen(packet, -ETHER_HDR_LEN);
	mbuf_setdata(packet, header + ETHER_HDR_LEN, length - ETHER_HDR_LEN);
	bpf_tap_in(n->ifp, DLT_EN10MB, packet, header, ETHER_HDR_LEN);
	n->count.received++;
	n->count.received_bytes += length;
	ifnet_input(n->ifp, packet, &stats);
}

void
hakios_net_input(const uint8_t *frame, size_t length)
{
	hakios_net_input_unit(0, frame, length);
}

static unsigned net_ioctls_said;

static errno_t
net_ioctl(ifnet_t ifp, u_long command, void *data)
{
	struct net_if *n = net_if_of(ifp);
	/* (a radio says it is one here too: IO80211.framework lists the interfaces whose media is 802.11) */
	const int media[] = { (n->link->wireless ? IFM_IEEE80211 : IFM_ETHER) | IFM_AUTO };
	struct ifreq *request = data;
	struct ifmediareq *media_request = data;
	user_addr_t list;
	errno_t error = 0;

	if (n->link->wireless && net_ioctls_said < 80) {
		char who[32] = "?";

		/* (while Darwin's Wi-Fi side is being got to talk to this: who asks the interface what) */
		proc_selfname(who, sizeof(who));
		if (strcmp(who, "configd") != 0) {
			net_ioctls_said++;
			printf("net: en0 is asked %#lx by %s\n", command, who);
		}
	}
	switch (command) {
	case SIOCSIFADDR:
		ifnet_set_flags(ifp, IFF_UP, IFF_UP);
		if (n->link->wireless) {
			net_announce_soon(n);	/* (a new IPv4 address: the access point's proxy ARP must learn it) */
		}
		break;
	case SIOCSIFFLAGS:
		/* running whenever it is up: the link is there or it is not, nothing is switched on */
		ifnet_set_flags(ifp, (ifnet_flags(ifp) & IFF_UP) ? IFF_RUNNING : 0, IFF_RUNNING);
		break;
	case SIOCGIFMEDIA32:
	case SIOCGIFMEDIA64:
		/* what configd asks to learn whether the cable is in */
		list = command == SIOCGIFMEDIA64 ? ((struct ifmediareq64 *)media_request)->ifmu_ulist :
		    CAST_USER_ADDR_T(((struct ifmediareq32 *)media_request)->ifmu_ulist);
		media_request->ifm_active = media_request->ifm_current = media[0];
		media_request->ifm_mask = 0;
		media_request->ifm_status = IFM_AVALID | (n->up ? IFM_ACTIVE : 0);
		if (list != USER_ADDR_NULL && media_request->ifm_count > 0) {
			error = copyout(media, list, sizeof(media));
		}
		media_request->ifm_count = 1;
		break;
	case SIOCGIFDEVMTU:
		request->ifr_devmtu.ifdm_current = ifnet_mtu(ifp);
		request->ifr_devmtu.ifdm_max = ETHERMTU;
		request->ifr_devmtu.ifdm_min = IF_MINMTU;
		break;
	case SIOCSIFMTU:
		error = request->ifr_mtu > ETHERMTU || request->ifr_mtu < IF_MINMTU ? EINVAL : ifnet_set_mtu(ifp, request->ifr_mtu);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		break;                          /* the link hands us everything anyway */
	default:
		/* what Darwin's Wi-Fi side asks of a radio's interface: bsd/dev/board/apple80211.c */
		error = n->link->wireless ? hakios_apple80211_ioctl(command, data) : EOPNOTSUPP;
		break;
	}
	return error;
}

/* Tell whoever listens for kernel events (configd does) that the link is up, or down. */
static void
net_link_event(ifnet_t ifp, uint32_t code)
{
	_Alignas(struct kern_event_msg) char message[sizeof(struct kern_event_msg) + sizeof(struct net_event_data)] = { 0 };
	struct kern_event_msg *header = (struct kern_event_msg *)message;
	struct net_event_data *about = (struct net_event_data *)(header + 1);

	header->total_size = sizeof(message);
	header->vendor_code = KEV_VENDOR_APPLE;
	header->kev_class = KEV_NETWORK_CLASS;
	header->kev_subclass = KEV_DL_SUBCLASS;
	header->event_code = code;
	about->if_family = ifnet_family(ifp);
	about->if_unit = ifnet_unit(ifp);
	strlcpy(about->if_name, ifnet_name(ifp), IFNAMSIZ);
	ifnet_event(ifp, header);
}

/*
 * The interface's IPv4 addresses announced (gratuitous ARP: sender = target = the address, to everybody) after the link
 * comes up, as Darwin's IPConfiguration does on a link-up event: an access point with proxy ARP answers the network's
 * ARP for its clients from a table it learns from them, and never passes those requests on. run55: the PC could not
 * find the phone at its address until the phone happened to send (arp_test: the request never reached Darwin; the
 * answer came while Darwin sent none). xnu's own announcement (in_arp.c, when an address is set) is only made once,
 * when the address is set -- not after a rejoin that keeps it. Twice, 1 s and 3 s after the link is up.
 */
extern void inet_arp_init_ifaddr(ifnet_t interface, ifaddr_t ipaddr);
static thread_call_t net_announce_call[HAKIOS_NET_UNITS];
static unsigned net_announce_left[HAKIOS_NET_UNITS];

static void
net_announce(thread_call_param_t unit_param, __unused thread_call_param_t unused)
{
	unsigned unit = (unsigned)(uintptr_t)unit_param;
	struct net_if *n = net_if_unit(unit);
	ifaddr_t *addresses;
	unsigned said = 0;

	if (n == NULL || !n->up) {
		return;
	}
	if (ifnet_get_address_list_family(n->ifp, &addresses, AF_INET) == 0) {
		for (unsigned i = 0; addresses[i] != NULL; i++, said++) {
			inet_arp_init_ifaddr(n->ifp, addresses[i]);
		}
		ifnet_free_address_list(addresses);
	}
	n->count.announced += said;
	if (net_announce_left[unit] != 0 && --net_announce_left[unit] != 0) {
		thread_call_enter_delayed(net_announce_call[unit], deadline_in_ms(2000));
	}
}

/*
 * Announcements 1 s and 3 s from now: after a link up (an address kept across a rejoin), and after an address is set
 * (DHCP's comes some seconds after the link: run57, the link-up ones had no address to say, and xnu's own single one
 * as the address was set did not leave the AP knowing it).
 */
static void
net_announce_soon(struct net_if *n)
{
	unsigned unit = n->unit;

	if (net_announce_call[unit] == NULL) {
		net_announce_call[unit] = thread_call_allocate_with_options(net_announce, (thread_call_param_t)(uintptr_t)unit,
		    THREAD_CALL_PRIORITY_KERNEL, THREAD_CALL_OPTIONS_ONCE);
	}
	if (net_announce_call[unit] != NULL) {
		net_announce_left[unit] = 2;
		thread_call_enter_delayed(net_announce_call[unit], deadline_in_ms(1000));
	}
}

void
hakios_net_link_state_unit(unsigned unit, bool up)
{
	struct net_if *n = net_if_unit(unit);

	if (n == NULL || up == n->up) {
		return;
	}
	n->up = up;
	printf("net: en%u's link (%s) is %s\n", n->unit, n->link->name, up ? "up" : "down");
	net_link_event(n->ifp, up ? KEV_DL_LINK_ON : KEV_DL_LINK_OFF);
	if (up) {
		net_announce_soon(n);
	}
}

void
hakios_net_link_state(bool up)
{
	hakios_net_link_state_unit(0, up);
}

/* The minute's line in the log, if anything has moved since the last one. */
static void
net_say(struct net_if *n, uint64_t *said)
{
	uint64_t total = n->count.sent + n->count.received + n->count.dropped_out + n->count.dropped_in;

	if (total == *said) {
		return;
	}
	*said = total;
	printf("net: en%u so far %llu frames sent (%llu bytes), %llu received (%llu bytes); dropped: %llu out, %llu in; "
	    "checksums: %llu sent left to the link, %llu received checked by it\n", n->unit, n->count.sent,
	    n->count.sent_bytes, n->count.received, n->count.received_bytes, n->count.dropped_out, n->count.dropped_in,
	    n->count.offloaded, n->count.checked);
}

/*
 * The interface's own thread: for a link that wants to be asked, and for the minute's line in the log. A link with
 * nothing to ask (no poll: USB and Wi-Fi bring their frames themselves) wakes it once a minute only; every 10 ms it
 * kept the cluster from ever having llcc-off's 14 ms (POWER-COLLAPSE-PLAN.md step 4, llcc-diag).
 */
static void
net_thread(void *parameter, __unused wait_result_t result)
{
	struct net_if *n = parameter;
	uint64_t said = 0;
	unsigned quiet = 0;

	if (n->link->poll == NULL) {
		for (;;) {
			delay_for_interval(60, NSEC_PER_SEC);
			net_say(n, &said);
		}
	}
	for (unsigned tick = 1;; tick++) {
		quiet = n->link->poll() ? 0 : quiet + 1;
		if (tick % 8192 == 0) {         /* about once a minute */
			net_say(n, &said);
		}
		if (n->link->poll == NULL) {
			/* Frame delivery wakes directly on its own thread; avoid 100 Hz timer wakeups when idle. */
			delay_for_interval(60 * 1000, NSEC_PER_MSEC);
			continue;
		}
		/*
		 * While frames come, ask again soon: a TCP sender waits for the acknowledgements this brings in,
		 * and 2 ms between askings held a stream from the machine to a few MB a second. Quiet: every
		 * 2 ms, then (after 100 empty askings) every 10.
		 */
		delay_for_interval(quiet == 0 ? 200 : quiet < 100 ? 2000 : 10000, NSEC_PER_USEC);
	}
}

bool
hakios_net_attach(const struct hakios_net_link *link)
{
	unsigned unit = link->unit;
	struct net_if *n = unit < HAKIOS_NET_UNITS ? &net_ifs[unit] : NULL;
	char uniqueid[16];
	struct ifnet_init_eparams init = {
		.ver = IFNET_INIT_CURRENT_VERSION,
		.len = sizeof(init),
		/*
		 * The simple model: every frame straight to net_output(). And no Skywalk "nexus" made for
		 * the interface by itself: sockets are all that is asked of it so far.
		 */
		.flags = IFNET_INIT_LEGACY | IFNET_INIT_NX_NOAUTO,
		.uniqueid = uniqueid,
		.uniqueid_len = 11,
		.name = "en",
		.unit = unit,
		.softc = n,
		.family = IFNET_FAMILY_ETHERNET,
		.subfamily = link->wireless ? IFNET_SUBFAMILY_WIFI : IFNET_SUBFAMILY_ANY,
		.type = IFT_ETHER,
		.output = net_output,
		.demux = ether_demux,
		.add_proto = ether_add_proto,
		.del_proto = ether_del_proto,
		.check_multi = ether_check_multi,
		.framer_extended = ether_frameout_extended,
		.ioctl = net_ioctl,
		.broadcast_addr = etherbroadcastaddr,
		.broadcast_len = ETHER_ADDR_LEN,
	};
	const uint8_t *address = link->address;
	ifnet_t ifp = NULL;
	thread_t thread;
	errno_t error;

	if (n == NULL) {
		printf("net: there is no unit %u for %s\n", unit, link->name);
		return false;
	}
	if (n->link != NULL) {
		printf("net: en%u is on %s already; %s is not attached\n", unit, n->link->name, link->name);
		return false;
	}
	snprintf(uniqueid, sizeof(uniqueid), "hakios-en%u", unit);
	n->unit = unit;
	n->link = link;
	n->up = link->up;
	lck_mtx_init(&n->tx_lock, &net_lck_grp, LCK_ATTR_NULL);
	error = ifnet_allocate_extended(&init, &ifp);
	if (error != 0) {
		printf("net: the interface en%u cannot be made: error %d\n", unit, error);
		lck_mtx_destroy(&n->tx_lock, &net_lck_grp);     /* (made again by the next attach) */
		n->link = NULL;
		return false;
	}
	ifnet_set_capabilities_enabled(ifp, 0, -1);
	ifnet_set_addrlen(ifp, ETHER_ADDR_LEN);
	ifnet_set_baudrate(ifp, 100 * 1000 * 1000);
	ifnet_set_mtu(ifp, ETHERMTU);
	ifnet_set_flags(ifp, IFF_BROADCAST | IFF_MULTICAST | IFF_SIMPLEX, 0xffff);
	ifnet_set_hdrlen(ifp, ETHER_HDR_LEN);
	/* no TSO: ath11k (and the other links) send what they are given as it is; checksums if the link says it does them */
	ifnet_set_offload(ifp, link->offload & HAKIOS_NET_CSUM_TX);
	error = ifnet_attach(ifp, NULL);
	if (error != 0) {
		printf("net: the interface en%u cannot be attached: error %d\n", unit, error);
		ifnet_release(ifp);
		lck_mtx_destroy(&n->tx_lock, &net_lck_grp);
		n->link = NULL;
		return false;
	}
	ifnet_set_lladdr(ifp, address, ETHER_ADDR_LEN);
	bpfattach(ifp, DLT_EN10MB, ETHER_HDR_LEN);
	n->ifp = ifp;
	if (n->up) {
		net_link_event(ifp, KEV_DL_LINK_ON);
	}
	if (link->wireless) {
		/* (a radio's address is the phone's own: not for the log; and Darwin's Wi-Fi side finds the interface by a service) */
		printf("net: en%u attached on %s (a radio), link %s\n", unit, link->name, n->up ? "up" : "down");
		HakiosWLANStart(uniqueid + 7, address);
	} else {
		printf("net: en%u attached on %s, address %02x:%02x:%02x:%02x:%02x:%02x, link %s\n", unit, link->name, address[0],
		    address[1], address[2], address[3], address[4], address[5], n->up ? "up" : "down");
	}
	if (kernel_thread_start(net_thread, n, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		/* (the interface stays, as Darwin has it; but nothing will ever be received on it: said, and its link down) */
		printf("net: en%u's thread cannot be started: NOTHING WILL BE RECEIVED ON IT; its link is taken down\n", unit);
		hakios_net_link_state_unit(unit, false);
	}
	return true;
}

void
hakios_net_init(void)
{
	PE_parse_boot_argn("net_trace", &net_trace, sizeof(net_trace));
	PE_parse_boot_argn("net_names", &net_names, sizeof(net_names));
	hakios_virtio_net_init();
	hakios_usb_net_init();
}
