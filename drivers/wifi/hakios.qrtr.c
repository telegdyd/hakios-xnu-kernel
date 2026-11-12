/*
 * hakios.qrtr.c: Qualcomm's IPC router (QRTR), as the QMI helpers (drivers/modem/hakios.qmi.c) need it.
 *
 * What the channel carries (Linux: net/qrtr/af_qrtr.c, ns.c, smd.c): datagrams from a port of a
 * node to a port of a node, one in each link message, behind a header of eight words. A node's
 * services announce themselves to the other nodes through a port that all have, the control
 * port: when a link comes up each side says hello, and is told the other's services one by one
 * ("new server": a service, an instance, a node, a port), and later their going. We are node 1
 * (Linux's default, and what this phone's Android is); the DSP says who it is in what it sends.
 *
 * Below this file is one link (C6, sys/hakios.qrtr.h): GLINK's channel to the Wi-Fi DSP today
 * (hakios.wifi.c), MHI's IPCR channel of the PCIe chip later. Above it are the four calls of
 * struct hakios_qrtr_ops, which hakios.wifi.c hands to the QMI helpers with hakios_qmi_set_transport().
 *
 * The two locks keep the meaning they had in hakios.wifi.c: qrtr_lock keeps the tables (and the
 * link that is up), qrtr_calling is held by whoever makes callbacks (a receive, a poll, a link
 * going down) around them, so that they come from one thread at a time. Neither is held across
 * a call into the link (send), and receive is never entered from send.
 */
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <sys/errno.h>
#include <sys/hakios.qrtr.h>
#include <sys/systm.h>

/* A line of the Wi-Fi steps' log (hakios.wifi.c), which is where these lines went before the router moved here. */
extern int hakios_wifi_say(const char *format, ...);
#define printf hakios_wifi_say

#define HAKIOS_QRTR_MAX        8192            /* the most a datagram holds, either way */
#define QRTR_US                 1u
#define QRTR_CONTROL            0xfffffffeu     /* the control port */
#define QRTR_FIRST_PORT         0x4000u         /* where Linux begins numbering ports that ask for no number */
enum { QRTR_DATA = 1, QRTR_HELLO = 2, QRTR_BYE = 3, QRTR_NEW_SERVER = 4, QRTR_DEL_SERVER = 5, QRTR_DEL_CLIENT = 6, QRTR_RESUME_TX = 7 };

struct qrtr_header {                            /* version 1, which Linux sends; version 2 is read as well */
	uint32_t version, type, from_node, from_port, confirm, size, to_node, to_port;
};
struct qrtr_control {
	uint32_t command;
	uint32_t words[4];                      /* a server: service, instance, node, port; a client: node, port */
};

struct hakios_qrtr_port {
	uint32_t number;                        /* 0: free */
	hakios_qrtr_receive_t receive;
	hakios_qrtr_server_t server;
	void *context;
	struct {
		uint32_t service, instance;     /* the instance as on the wire: version | instance << 8; 0: any */
		bool asked;
		bool fresh;                     /* not yet told of the services that were there before it */
	} lookups[4];
};

static LCK_GRP_DECLARE(qrtr_lock_group, "hakios-qrtr");
static LCK_MTX_DECLARE(qrtr_lock, &qrtr_lock_group);    /* the tables below, and the link */
static LCK_MTX_DECLARE(qrtr_calling, &qrtr_lock_group); /* held by whoever makes callbacks, around them */
static struct hakios_qrtr_port qrtr_ports[16];
static struct {
	uint32_t service, instance, node, port;
	bool there;
} qrtr_servers[64];
static uint32_t qrtr_peer;                      /* the DSP's node; 0: it has not said */
static uint32_t qrtr_next_port = QRTR_FIRST_PORT, qrtr_in, qrtr_out, qrtr_hellos;

static const struct hakios_qrtr_link *qrtr_link;        /* the one that is up (qrtr_lock) */
static bool qrtr_closing;                       /* it is going down: no new traffic (qrtr_lock) */
static unsigned qrtr_sending;                   /* sends that are inside the link's send (qrtr_lock) */
static thread_t volatile qrtr_calling_thread;   /* who holds qrtr_calling, for a close() from a callback */

static void
qrtr_calling_enter(void)
{
	lck_mtx_lock(&qrtr_calling);
	qrtr_calling_thread = current_thread();
}

static void
qrtr_calling_leave(void)
{
	qrtr_calling_thread = NULL;
	lck_mtx_unlock(&qrtr_calling);
}

/* One datagram out, of any type. */
static int
qrtr_out_packet(uint32_t type, uint32_t from_port, uint32_t to_node, uint32_t to_port, const void *data, uint32_t length)
{
	uint32_t padded = (length + 3) & ~3u;
	const struct hakios_qrtr_link *link;
	struct qrtr_header *packet;
	int error;

	if (length == 0 || length > HAKIOS_QRTR_MAX) {
		return EMSGSIZE;
	}
	packet = kalloc_data(sizeof(*packet) + padded, Z_WAITOK | Z_ZERO);
	if (packet == NULL) {
		return ENOMEM;
	}
	packet->version = 1;
	packet->type = type;
	packet->from_node = QRTR_US;
	packet->from_port = from_port;
	packet->size = length;
	packet->to_node = to_node;
	packet->to_port = to_port;
	memcpy(packet + 1, data, length);
	lck_mtx_lock(&qrtr_lock);
	link = qrtr_closing ? NULL : qrtr_link;
	qrtr_sending += link != NULL;
	lck_mtx_unlock(&qrtr_lock);
	if (link == NULL) {
		error = ENOTCONN;
	} else {
		error = link->send(link->ctx, packet, (uint32_t)sizeof(*packet) + padded);      /* (no lock of ours is held) */
		lck_mtx_lock(&qrtr_lock);
		qrtr_sending--;
		qrtr_out += error == 0;
		thread_wakeup((event_t)&qrtr_sending);
		lck_mtx_unlock(&qrtr_lock);
	}
	kfree_data(packet, sizeof(*packet) + padded);
	return error;
}

static bool
qrtr_matches(uint32_t service, uint32_t instance, unsigned server)
{
	return qrtr_servers[server].there && qrtr_servers[server].service == service &&
	       (instance == 0 || qrtr_servers[server].instance == instance);
}

/* Everybody who looked a service up is told that it has come, or gone. With qrtr_calling held. */
static void
qrtr_tell(uint32_t service, uint32_t instance, uint32_t node, uint32_t port, bool arrived)
{
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		for (unsigned l = 0; l < 4; l++) {
			hakios_qrtr_server_t server = NULL;
			void *context;

			lck_mtx_lock(&qrtr_lock);
			if (qrtr_ports[p].number != 0 && qrtr_ports[p].lookups[l].asked && !qrtr_ports[p].lookups[l].fresh &&
			    qrtr_ports[p].lookups[l].service == service &&
			    (qrtr_ports[p].lookups[l].instance == 0 || qrtr_ports[p].lookups[l].instance == instance)) {
				server = qrtr_ports[p].server;
			}
			context = qrtr_ports[p].context;
			lck_mtx_unlock(&qrtr_lock);
			if (server != NULL) {
				server(context, arrived, service, instance & 0xff, instance >> 8, node, port);
			}
		}
	}
}

/* The control port's part: Linux's name service, without services of our own to tell of. */
static void
qrtr_control_message(const struct qrtr_header *header, const struct qrtr_control *control)
{
	struct qrtr_control hello = { .command = QRTR_HELLO };
	int found = -1, error;

	switch (control->command) {
	case QRTR_HELLO:
		if (qrtr_hellos++ != 0) {
			break;                                  /* (answered once: two that answer every hello never end) */
		}
		error = qrtr_out_packet(QRTR_HELLO, QRTR_CONTROL, header->from_node, QRTR_CONTROL, &hello, sizeof(hello));
		printf("wifi: qrtr: HELLO FROM NODE %u; ours to it: %s\n", header->from_node, error == 0 ? "SENT" : "NOT SENT");
		break;
	case QRTR_NEW_SERVER:
		lck_mtx_lock(&qrtr_lock);
		for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
			if (qrtr_servers[i].there && qrtr_servers[i].node == control->words[2] && qrtr_servers[i].port == control->words[3]) {
				found = (int)i;
				break;
			}
			if (!qrtr_servers[i].there && found < 0) {
				found = (int)i;
			}
		}
		if (found >= 0) {
			qrtr_servers[found].there = true;
			qrtr_servers[found].service = control->words[0];
			qrtr_servers[found].instance = control->words[1];
			qrtr_servers[found].node = control->words[2];
			qrtr_servers[found].port = control->words[3];
		}
		lck_mtx_unlock(&qrtr_lock);
		printf("wifi: qrtr: a service: %u (%#x), version %u, instance %u, at node %u port %u\n", control->words[0],
		    control->words[0], control->words[1] & 0xff, control->words[1] >> 8, control->words[2], control->words[3]);
		qrtr_tell(control->words[0], control->words[1], control->words[2], control->words[3], true);
		break;
	case QRTR_DEL_SERVER:
	case QRTR_BYE:
		for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
			bool gone;

			lck_mtx_lock(&qrtr_lock);
			gone = qrtr_servers[i].there && (control->command == QRTR_BYE ? qrtr_servers[i].node == header->from_node :
			    qrtr_servers[i].node == control->words[2] && qrtr_servers[i].port == control->words[3]);
			qrtr_servers[i].there = qrtr_servers[i].there && !gone;
			lck_mtx_unlock(&qrtr_lock);
			if (gone) {
				printf("wifi: qrtr: service %u at node %u port %u has gone\n", qrtr_servers[i].service, qrtr_servers[i].node,
				    qrtr_servers[i].port);
				qrtr_tell(qrtr_servers[i].service, qrtr_servers[i].instance, qrtr_servers[i].node, qrtr_servers[i].port, false);
			}
		}
		break;
	default:
		break;                                  /* (a client's going, and what else there is: nothing here wants it) */
	}
}

/* A message from the link: one datagram. With qrtr_calling held. */
static int
qrtr_input(const uint8_t *data, uint32_t length)
{
	struct qrtr_header header;
	hakios_qrtr_receive_t receive = NULL;
	void *context = NULL;
	uint32_t at;

	if (length >= sizeof(header) && data[0] == 1) {
		memcpy(&header, data, sizeof(header));
		at = sizeof(header);
	} else if (length >= 16 && data[0] == 3) {
		/* version 2: version, type, flags, a length of options; the size; then four numbers of 16 bits */
		bzero(&header, sizeof(header));
		header.type = data[1];
		header.confirm = data[2] & 1;
		memcpy(&header.size, data + 4, 4);
		header.from_node = (uint32_t)(data[8] | data[9] << 8);
		header.from_port = (uint32_t)(data[10] | data[11] << 8);
		header.to_node = (uint32_t)(data[12] | data[13] << 8);
		header.to_port = (uint32_t)(data[14] | data[15] << 8);
		header.from_port = header.from_port == 0xfffe ? QRTR_CONTROL : header.from_port;
		header.to_port = header.to_port == 0xfffe ? QRTR_CONTROL : header.to_port;
		at = 16 + data[3];
	} else {
		printf("wifi: qrtr: %u bytes that are no datagram (they begin %#x)\n", length, length ? data[0] : 0);
		return EINVAL;
	}
	/* (at + header.size would wrap for a huge size and pass: the check is a subtraction, after the size is known to fit) */
	if (header.size == 0 || header.size > HAKIOS_QRTR_MAX || header.size > length || at > length - header.size) {
		printf("wifi: qrtr: a datagram of %u bytes in a message of %u: dropped\n", header.size, length);
		return EINVAL;
	}
	lck_mtx_lock(&qrtr_lock);
	qrtr_in++;
	if (qrtr_peer == 0) {
		qrtr_peer = header.from_node;
	}
	lck_mtx_unlock(&qrtr_lock);
	if (header.type == QRTR_RESUME_TX) {
		return 0;                               /* (we never ask for one: the link's own flow control holds us back already) */
	}
	if (header.confirm) {
		/* the sender counts what it has sent to this port, and waits for this before it sends many more */
		struct qrtr_control resume = { .command = QRTR_RESUME_TX, .words = { header.to_node, header.to_port } };

		qrtr_out_packet(QRTR_RESUME_TX, header.to_port, header.from_node, header.from_port, &resume, sizeof(resume));
	}
	if (header.to_port == QRTR_CONTROL) {
		struct qrtr_control control = { 0 };

		memcpy(&control, data + at, header.size < sizeof(control) ? header.size : sizeof(control));
		qrtr_control_message(&header, &control);
		return 0;
	}
	lck_mtx_lock(&qrtr_lock);
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		if (qrtr_ports[p].number == header.to_port && header.type == QRTR_DATA) {
			receive = qrtr_ports[p].receive;
			context = qrtr_ports[p].context;
		}
	}
	lck_mtx_unlock(&qrtr_lock);
	if (receive != NULL) {
		receive(context, header.from_node, header.from_port, data + at, header.size);
	}
	return 0;
}

/* Lookups made since the last look are told of the services that were there before them. With qrtr_calling held. */
static void
qrtr_fresh(void)
{
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]); p++) {
		for (unsigned l = 0; l < 4; l++) {
			hakios_qrtr_server_t server;
			uint32_t service, instance;
			void *context;

			lck_mtx_lock(&qrtr_lock);
			if (qrtr_ports[p].number == 0 || !qrtr_ports[p].lookups[l].fresh) {
				lck_mtx_unlock(&qrtr_lock);
				continue;
			}
			qrtr_ports[p].lookups[l].fresh = false;
			server = qrtr_ports[p].server;
			service = qrtr_ports[p].lookups[l].service;
			instance = qrtr_ports[p].lookups[l].instance;
			context = qrtr_ports[p].context;
			lck_mtx_unlock(&qrtr_lock);
			for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]) && server != NULL; i++) {
				if (qrtr_matches(service, instance, i)) {
					server(context, true, qrtr_servers[i].service, qrtr_servers[i].instance & 0xff,
					    qrtr_servers[i].instance >> 8, qrtr_servers[i].node, qrtr_servers[i].port);
				}
			}
		}
	}
}

/* ---- the link (C6) ------------------------------------------------------------------------------------ */

int
hakios_qrtr_link_up(const struct hakios_qrtr_link *link)
{
	if (link == NULL || link->send == NULL) {
		return EINVAL;
	}
	lck_mtx_lock(&qrtr_lock);
	if (qrtr_link == link && !qrtr_closing) {
		lck_mtx_unlock(&qrtr_lock);
		return 0;                               /* (the same one again: nothing to do) */
	}
	if (qrtr_link != NULL) {
		lck_mtx_unlock(&qrtr_lock);
		return EBUSY;
	}
	/*
	 * Published here, before anything is sent over it. Nothing is said when a link comes up: the DSP
	 * begins, and its hello is answered with ours (qrtr_control_message()).
	 */
	qrtr_link = link;
	qrtr_closing = false;
	qrtr_hellos = qrtr_in = qrtr_out = 0;
	qrtr_peer = 0;
	lck_mtx_unlock(&qrtr_lock);
	return 0;
}

int
hakios_qrtr_receive(const struct hakios_qrtr_link *link, const void *packet, unsigned long len)
{
	int error;

	if (link == NULL) {
		return EINVAL;
	}
	qrtr_calling_enter();                           /* (a link going down is waited for here, and then this one is refused) */
	lck_mtx_lock(&qrtr_lock);
	error = qrtr_link == link && !qrtr_closing ? 0 : ENOTCONN;
	lck_mtx_unlock(&qrtr_lock);
	if (error == 0) {
		error = packet == NULL || len == 0 || len > sizeof(struct qrtr_header) + HAKIOS_QRTR_MAX ? EINVAL :
		    qrtr_input(packet, (uint32_t)len);
		if (error == 0) {
			qrtr_fresh();
		}
	}
	qrtr_calling_leave();
	return error;
}

void
hakios_qrtr_link_down(const struct hakios_qrtr_link *link)
{
	bool inside = false;

	lck_mtx_lock(&qrtr_lock);
	while (link != NULL && qrtr_link == link && qrtr_closing) {
		lck_mtx_sleep(&qrtr_lock, LCK_SLEEP_DEFAULT, (event_t)&qrtr_link, THREAD_UNINT);        /* (another is taking it down) */
	}
	if (link == NULL || qrtr_link != link) {
		lck_mtx_unlock(&qrtr_lock);
		return;
	}
	qrtr_closing = true;                            /* no new traffic, in or out */
	while (qrtr_sending != 0) {
		lck_mtx_sleep(&qrtr_lock, LCK_SLEEP_DEFAULT, (event_t)&qrtr_sending, THREAD_UNINT);
	}
	lck_mtx_unlock(&qrtr_lock);
	inside = qrtr_calling_thread == current_thread();       /* (down from a callback: that one cannot be waited for) */
	if (!inside) {
		qrtr_calling_enter();                   /* (a receive that is running ends first) */
	}
	/* The servers behind the link are gone, and the ports that looked them up are told (no line of the log: the link's owner has its own). */
	for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
		uint32_t service, instance, node, port;
		bool there;

		lck_mtx_lock(&qrtr_lock);
		there = qrtr_servers[i].there;
		service = qrtr_servers[i].service;
		instance = qrtr_servers[i].instance;
		node = qrtr_servers[i].node;
		port = qrtr_servers[i].port;
		qrtr_servers[i].there = false;
		lck_mtx_unlock(&qrtr_lock);
		if (there) {
			qrtr_tell(service, instance, node, port, false);
		}
	}
	lck_mtx_lock(&qrtr_lock);
	qrtr_peer = 0;
	qrtr_link = NULL;
	qrtr_closing = false;
	thread_wakeup((event_t)&qrtr_link);
	lck_mtx_unlock(&qrtr_lock);
	if (!inside) {
		qrtr_calling_leave();
	}
}

/*
 * For the link's thread to call now and then (hakios.wifi.c does, every millisecond): lookups
 * made since the last look are told of the services that are there already. A receive does the
 * same after each packet, so a link that has no thread of its own for this needs no more.
 */
void
hakios_qrtr_poll(void)
{
	qrtr_calling_enter();
	qrtr_fresh();
	qrtr_calling_leave();
}

/* What the steps of hakios.wifi.c write down about the exchange so far. */
void
hakios_qrtr_stats(uint32_t *peer, uint32_t *hellos, uint32_t *in, uint32_t *out, uint32_t *services)
{
	uint32_t there = 0;

	lck_mtx_lock(&qrtr_lock);
	for (unsigned i = 0; i < sizeof(qrtr_servers) / sizeof(qrtr_servers[0]); i++) {
		there += qrtr_servers[i].there;
	}
	*peer = qrtr_peer;
	*hellos = qrtr_hellos;
	*in = qrtr_in;
	*out = qrtr_out;
	*services = there;
	lck_mtx_unlock(&qrtr_lock);
}

/* ---- struct hakios_qrtr_ops --------------------------------------------------------------------------- */

static struct hakios_qrtr_port *
qrtr_open(hakios_qrtr_receive_t receive, hakios_qrtr_server_t server, void *context)
{
	struct hakios_qrtr_port *port = NULL;

	lck_mtx_lock(&qrtr_lock);
	for (unsigned p = 0; p < sizeof(qrtr_ports) / sizeof(qrtr_ports[0]) && port == NULL; p++) {
		if (qrtr_ports[p].number == 0) {
			port = &qrtr_ports[p];
			bzero(port, sizeof(*port));
			port->number = qrtr_next_port++;
			port->receive = receive;
			port->server = server;
			port->context = context;
		}
	}
	lck_mtx_unlock(&qrtr_lock);
	return port;
}

static int
qrtr_send(struct hakios_qrtr_port *port, uint32_t node, uint32_t to, const void *data, size_t length)
{
	if (port == NULL || port->number == 0 || length > HAKIOS_QRTR_MAX) {
		return EINVAL;
	}
	return qrtr_out_packet(QRTR_DATA, port->number, node, to, data, (uint32_t)length);
}

static int
qrtr_lookup(struct hakios_qrtr_port *port, uint32_t service, uint32_t version, uint32_t instance)
{
	int error = ENOSPC;

	lck_mtx_lock(&qrtr_lock);
	for (unsigned l = 0; l < 4 && error != 0 && port != NULL && port->number != 0; l++) {
		if (!port->lookups[l].asked) {
			port->lookups[l].service = service;
			port->lookups[l].instance = instance != 0 ? (version & 0xff) | instance << 8 : 0;
			port->lookups[l].asked = true;
			port->lookups[l].fresh = true;
			error = 0;
		}
	}
	lck_mtx_unlock(&qrtr_lock);
	return error;
}

static void
qrtr_close(struct hakios_qrtr_port *port)
{
	struct qrtr_control gone = { .command = QRTR_DEL_CLIENT, .words = { QRTR_US, port ? port->number : 0 } };
	bool outside = qrtr_calling_thread != current_thread();
	uint32_t peer;

	if (port == NULL || port->number == 0) {
		return;
	}
	if (outside) {
		lck_mtx_lock(&qrtr_calling);            /* (a callback of the port's that is running ends first) */
	}
	lck_mtx_lock(&qrtr_lock);
	port->number = 0;
	peer = qrtr_peer;
	lck_mtx_unlock(&qrtr_lock);
	if (outside) {
		lck_mtx_unlock(&qrtr_calling);
	}
	if (peer != 0) {
		qrtr_out_packet(QRTR_DEL_CLIENT, gone.words[1], peer, QRTR_CONTROL, &gone, sizeof(gone));
	}
}

const struct hakios_qrtr_ops hakios_qrtr_router_ops = {
	.open = qrtr_open, .send = qrtr_send, .lookup = qrtr_lookup, .close = qrtr_close
};
