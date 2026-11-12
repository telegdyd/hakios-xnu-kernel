/*
 * Host tests of the QRTR router (drivers/wifi/hakios.qrtr.c, compiled as it is) behind a fake link:
 * C6's link up/receive/down, the four calls of struct hakios_qrtr_ops above it, and what it writes
 * to the log. Run by run.sh under the address and undefined-behaviour sanitizers.
 */
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/hakios.qrtr.h>

/* What hakios.qrtr.c has besides C6 and the ops, and what it says its lines with (hakios.wifi.c's): here, into a buffer. */
extern const struct hakios_qrtr_ops hakios_qrtr_router_ops;
extern void hakios_qrtr_poll(void);
extern void hakios_qrtr_stats(uint32_t *peer, uint32_t *hellos, uint32_t *in, uint32_t *out, uint32_t *services);
int hakios_wifi_say(const char *format, ...);

#define OPS hakios_qrtr_router_ops

static char logbuf[16384];
static pthread_mutex_t logmutex = PTHREAD_MUTEX_INITIALIZER;

int
hakios_wifi_say(const char *format, ...)
{
	va_list list;
	size_t at;

	pthread_mutex_lock(&logmutex);
	at = strlen(logbuf);
	va_start(list, format);
	vsnprintf(logbuf + at, sizeof(logbuf) - at, format, list);
	va_end(list);
	pthread_mutex_unlock(&logmutex);
	return 0;
}

static int checks, failures;
#define CHECK(condition) do { \
		checks++; \
		if (!(condition)) { \
			failures++; \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		} \
	} while (0)

static bool
log_has(const char *text)
{
	return strstr(logbuf, text) != NULL;
}

static bool
log_empty(void)
{
	return logbuf[0] == 0;
}

static void
log_clear(void)
{
	pthread_mutex_lock(&logmutex);
	logbuf[0] = 0;
	pthread_mutex_unlock(&logmutex);
}

/* ---- the fake link ------------------------------------------------------------------------------------ */

#define MAXPACKETS 64
static struct {
	uint8_t bytes[32 + 8192 + 8];
	unsigned long length;
} sent[MAXPACKETS];
static unsigned sent_count;
static int send_error;
static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t send_cond = PTHREAD_COND_INITIALIZER;
static bool send_gate, send_inside;     /* a send that waits for the gate to open; and one that is in there */
static int link_context;

static int
fake_send(void *ctx, const void *packet, unsigned long length)
{
	pthread_mutex_lock(&send_mutex);
	if (ctx != &link_context) {
		failures++;
	}
	if (sent_count < MAXPACKETS && length <= sizeof(sent[0].bytes)) {
		memcpy(sent[sent_count].bytes, packet, length);
		sent[sent_count].length = length;
		sent_count++;
	}
	send_inside = true;
	pthread_cond_broadcast(&send_cond);
	while (send_gate) {
		pthread_cond_wait(&send_cond, &send_mutex);
	}
	send_inside = false;
	pthread_mutex_unlock(&send_mutex);
	return send_error;
}

static const struct hakios_qrtr_link link_a = { .send = fake_send, .ctx = &link_context, .name = "fake-a" };
static const struct hakios_qrtr_link link_b = { .send = fake_send, .ctx = &link_context, .name = "fake-b" };

static void
sent_clear(void)
{
	pthread_mutex_lock(&send_mutex);
	sent_count = 0;
	pthread_mutex_unlock(&send_mutex);
}

/* ---- packets ------------------------------------------------------------------------------------------ */

#define CONTROL 0xfffffffeu
enum { DATA = 1, HELLO = 2, BYE = 3, NEW_SERVER = 4, DEL_SERVER = 5, DEL_CLIENT = 6, RESUME_TX = 7 };
#define WLFW 0x45u

static unsigned long
v1(uint8_t *out, uint32_t type, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port, uint32_t confirm,
    const void *payload, uint32_t size)
{
	uint32_t header[8] = { 1, type, from_node, from_port, confirm, size, to_node, to_port };
	uint32_t padded = (size + 3) & ~3u;

	memset(out, 0, 32 + padded);
	memcpy(out, header, 32);
	memcpy(out + 32, payload, size);
	return 32 + padded;
}

/* A message for the control port: its command and four words. */
static unsigned long
v1_control(uint8_t *out, uint32_t type, uint32_t from_node, uint32_t command, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
	uint32_t body[5] = { command, a, b, c, d };

	return v1(out, type, from_node, CONTROL, 1, CONTROL, 0, body, sizeof(body));
}

/* Version 2: version 3, type, flags, options' length; size; four numbers of 16 bits; options; payload. */
static unsigned long
v2(uint8_t *out, uint32_t type, uint32_t flags, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port,
    unsigned options, const void *payload, uint32_t size)
{
	memset(out, 0, 16 + options + size);
	out[0] = 3;
	out[1] = (uint8_t)type;
	out[2] = (uint8_t)flags;
	out[3] = (uint8_t)options;
	memcpy(out + 4, &size, 4);
	out[8] = (uint8_t)from_node;
	out[9] = (uint8_t)(from_node >> 8);
	out[10] = (uint8_t)from_port;
	out[11] = (uint8_t)(from_port >> 8);
	out[12] = (uint8_t)to_node;
	out[13] = (uint8_t)(to_node >> 8);
	out[14] = (uint8_t)to_port;
	out[15] = (uint8_t)(to_port >> 8);
	memcpy(out + 16 + options, payload, size);
	return 16 + options + size;
}

static uint32_t
word(unsigned packet, unsigned index)
{
	uint32_t v;

	memcpy(&v, sent[packet].bytes + 4 * index, 4);
	return v;
}

static uint8_t buffer[32 + 8192 + 64];

static int
deliver(const struct hakios_qrtr_link *link, unsigned long length)
{
	return hakios_qrtr_receive(link, buffer, length);
}

/* ---- what the ports are told -------------------------------------------------------------------------- */

struct events {
	int servers, arrived, gone, datagrams;
	uint32_t service, version, instance, node, port;
	uint32_t from_node, from_port;
	size_t length;
	uint8_t data[64];
	struct hakios_qrtr_port *close_on_server;       /* closed from inside the callback, if set */
	volatile bool slow;                             /* a datagram waits here until it is let go */
	volatile bool slow_inside;
};

static void
on_server(void *context, bool arrived, uint32_t service, uint32_t version, uint32_t instance, uint32_t node, uint32_t port)
{
	struct events *e = context;

	e->servers++;
	e->arrived += arrived;
	e->gone += !arrived;
	e->service = service;
	e->version = version;
	e->instance = instance;
	e->node = node;
	e->port = port;
	if (e->close_on_server != NULL) {
		struct hakios_qrtr_port *p = e->close_on_server;

		e->close_on_server = NULL;
		OPS.close(p);
	}
}

static void
on_receive(void *context, uint32_t node, uint32_t port, const void *data, size_t length)
{
	struct events *e = context;

	if (e->slow) {
		e->slow_inside = true;
		while (e->slow) {
			usleep(1000);
		}
	}
	e->datagrams++;
	e->from_node = node;
	e->from_port = port;
	e->length = length;
	memcpy(e->data, data, length < sizeof(e->data) ? length : sizeof(e->data));
}

/* ---- the tests ---------------------------------------------------------------------------------------- */

static void
test_link_state(void)
{
	struct hakios_qrtr_link nosend = { .send = NULL, .ctx = NULL, .name = "no-send" };
	uint32_t body = 0;

	/* nothing is up: nothing is accepted, and nothing is sent */
	v1(buffer, DATA, 3, 1, 1, 0x4000, 0, &body, 4);
	CHECK(deliver(&link_a, 36) == ENOTCONN);
	CHECK(hakios_qrtr_receive(NULL, buffer, 36) == EINVAL);
	hakios_qrtr_link_down(&link_a);                 /* (not up: nothing) */
	hakios_qrtr_link_down(NULL);
	CHECK(OPS.send(NULL, 3, 7, "abcd", 4) == EINVAL);
	CHECK(hakios_qrtr_link_up(NULL) == EINVAL);
	CHECK(hakios_qrtr_link_up(&nosend) == EINVAL);

	/* up; again; and another */
	CHECK(hakios_qrtr_link_up(&link_a) == 0);
	CHECK(hakios_qrtr_link_up(&link_a) == 0);
	CHECK(hakios_qrtr_link_up(&link_b) == EBUSY);
	CHECK(deliver(&link_b, 36) == ENOTCONN);        /* (b is not the link that is up) */
	hakios_qrtr_link_down(&link_b);                 /* (not the one that is up: nothing) */
	CHECK(deliver(&link_a, 36) == 0);               /* (a is still up; to a port that is none) */
}

static void
test_hello(void)
{
	uint32_t peer, hellos, in, out, services;

	/* published before anything is sent: a hello is answered, over the link, at once */
	sent_clear();
	log_clear();
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, HELLO, 0, 0, 0, 0)) == 0);
	CHECK(sent_count == 1 && sent[0].length == 32 + 20);
	CHECK(word(0, 0) == 1);                         /* version */
	CHECK(word(0, 1) == HELLO);
	CHECK(word(0, 2) == 1);                         /* from node: ours */
	CHECK(word(0, 3) == CONTROL);
	CHECK(word(0, 4) == 0);                         /* confirm */
	CHECK(word(0, 5) == 20);                        /* size */
	CHECK(word(0, 6) == 3);                         /* to node: the sender */
	CHECK(word(0, 7) == CONTROL);
	CHECK(word(0, 8) == HELLO);                     /* the control message */
	CHECK(strcmp(logbuf, "wifi: qrtr: HELLO FROM NODE 3; ours to it: SENT\n") == 0);
	/* a second hello is not answered (two that answer every hello never end) */
	sent_clear();
	log_clear();
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, HELLO, 0, 0, 0, 0)) == 0);
	CHECK(sent_count == 0 && log_empty());
	hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
	CHECK(peer == 3 && hellos == 2 && out == 1 && services == 0 && in >= 3);
	/* a link that cannot send: the hello is not answered, and the line says so */
	{
		/* (only the first hello of a link is answered: a link of its own for this) */
		hakios_qrtr_link_down(&link_a);
		CHECK(hakios_qrtr_link_up(&link_a) == 0);
		hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
		CHECK(peer == 0 && hellos == 0 && in == 0 && out == 0);         /* (counted from the link's coming up) */
		send_error = EIO;
		log_clear();
		CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, HELLO, 0, 0, 0, 0)) == 0);
		CHECK(strcmp(logbuf, "wifi: qrtr: HELLO FROM NODE 3; ours to it: NOT SENT\n") == 0);
		send_error = 0;
		hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
		CHECK(out == 0 && hellos == 1);
	}
}

static struct events early, late;
static struct hakios_qrtr_port *early_port, *late_port;

static void
test_servers_and_lookup(void)
{
	struct hakios_qrtr_port *other_port;
	struct events other = { 0 }, third = { 0 };
	struct hakios_qrtr_port *third_port;

	/* a lookup made before the server appears */
	early_port = OPS.open(on_receive, on_server, &early);
	CHECK(early_port != NULL);
	CHECK(OPS.lookup(early_port, WLFW, 1, 0) == 0);
	hakios_qrtr_poll();
	CHECK(early.servers == 0);                      /* (nothing is there yet) */
	/* one for an instance that will not come */
	other_port = OPS.open(on_receive, on_server, &other);
	CHECK(OPS.lookup(other_port, WLFW, 1, 5) == 0);
	log_clear();
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, WLFW, 1, 3, 7)) == 0);
	CHECK(strcmp(logbuf, "wifi: qrtr: a service: 69 (0x45), version 1, instance 0, at node 3 port 7\n") == 0);
	CHECK(early.servers == 1 && early.arrived == 1);
	CHECK(early.service == WLFW && early.version == 1 && early.instance == 0 && early.node == 3 && early.port == 7);
	CHECK(other.servers == 0);
	/* a lookup made after it has: told by the next look (a receive does it, so does the link's thread's poll) */
	late_port = OPS.open(on_receive, on_server, &late);
	CHECK(OPS.lookup(late_port, WLFW, 1, 0) == 0);
	CHECK(late.servers == 0);                       /* (not from inside lookup: the QMI helper may hold locks there) */
	hakios_qrtr_poll();
	CHECK(late.servers == 1 && late.arrived == 1 && late.service == WLFW && late.node == 3 && late.port == 7);
	hakios_qrtr_poll();
	CHECK(late.servers == 1 && early.servers == 1); /* (once) */
	/* a service of another number is not what was asked for; version 2 instance 1 is told as such */
	third_port = OPS.open(on_receive, on_server, &third);
	CHECK(OPS.lookup(third_port, 0x47, 1, 0) == 0);
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, 0x46, 0x0102, 3, 9)) == 0);
	CHECK(third.servers == 0);
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, 0x47, 0x0102, 3, 10)) == 0);
	CHECK(third.servers == 1 && third.service == 0x47 && third.version == 2 && third.instance == 1 && third.port == 10);
	/* the instance of a lookup that names one */
	CHECK(OPS.lookup(other_port, 0x47, 2, 1) == 0);
	hakios_qrtr_poll();
	CHECK(other.servers == 1 && other.service == 0x47 && other.port == 10);
	/* their going, by port */
	log_clear();
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, DEL_SERVER, 0x47, 0x0102, 3, 10)) == 0);
	CHECK(third.servers == 2 && third.gone == 1 && other.gone == 1);
	CHECK(strcmp(logbuf, "wifi: qrtr: service 71 at node 3 port 10 has gone\n") == 0);
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, DEL_SERVER, 0x46, 0x0102, 3, 9)) == 0);   /* (one that was never here) */
	CHECK(third.servers == 2);
	/* the node's going takes its servers with it */
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, 0x47, 0x0102, 3, 12)) == 0);
	CHECK(third.servers == 3 && third.arrived == 2);
	log_clear();
	CHECK(deliver(&link_a, v1_control(buffer, BYE, 3, BYE, 0, 0, 0, 0)) == 0);
	CHECK(log_has("wifi: qrtr: service 69 at node 3 port 7 has gone\n") && log_has("service 71 at node 3 port 12 has gone\n"));
	CHECK(early.gone == 1 && late.gone == 1 && third.gone == 2);
	/* (and the Wi-Fi firmware's service is back, for what follows) */
	CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, WLFW, 1, 3, 7)) == 0);
	CHECK(early.arrived == 2 && late.arrived == 2);
	OPS.close(third_port);
	OPS.close(other_port);
	/* a port asks four times, no more */
	{
		struct hakios_qrtr_port *p = OPS.open(on_receive, on_server, &other);

		CHECK(OPS.lookup(NULL, WLFW, 1, 0) == ENOSPC);
		for (unsigned i = 0; i < 4; i++) {
			CHECK(OPS.lookup(p, 0x50 + i, 1, 0) == 0);
		}
		CHECK(OPS.lookup(p, 0x60, 1, 0) == ENOSPC);
		OPS.close(p);
	}
}

static void
test_datagrams(void)
{
	static const uint8_t question[7] = { 0, 1, 0, 0x24, 0, 0, 0 };
	static uint8_t big[8192];
	uint8_t answer[11] = { 2, 1, 0, 0x24, 0, 4, 0, 0, 0, 0, 0 };
	uint32_t from_port;

	/* out: the datagram, its header, its padding */
	sent_clear();
	CHECK(OPS.send(early_port, 3, 7, question, sizeof(question)) == 0);
	CHECK(sent_count == 1 && sent[0].length == 32 + 8);
	CHECK(word(0, 0) == 1 && word(0, 1) == DATA && word(0, 2) == 1);
	from_port = word(0, 3);
	CHECK(from_port >= 0x4000);
	CHECK(word(0, 4) == 0 && word(0, 5) == 7 && word(0, 6) == 3 && word(0, 7) == 7);
	CHECK(memcmp(sent[0].bytes + 32, question, 7) == 0 && sent[0].bytes[39] == 0);
	/* in: from the service's port to ours */
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, 7, 1, from_port, 0, answer, sizeof(answer))) == 0);
	CHECK(early.datagrams == 1 && early.from_node == 3 && early.from_port == 7 && early.length == sizeof(answer));
	CHECK(memcmp(early.data, answer, sizeof(answer)) == 0);
	CHECK(late.datagrams == 0);
	/* to a port that is none */
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, 7, 1, 0x3999, 0, answer, sizeof(answer))) == 0);
	CHECK(early.datagrams == 1);
	/* sizes and arguments */
	CHECK(OPS.send(early_port, 3, 7, question, 0) == EMSGSIZE);
	CHECK(OPS.send(early_port, 3, 7, question, 8193) == EINVAL);
	CHECK(OPS.send(NULL, 3, 7, question, 7) == EINVAL);
	sent_clear();
	CHECK(OPS.send(early_port, 3, 7, big, sizeof(big)) == 0);
	CHECK(sent_count == 1 && sent[0].length == 32 + 8192);
	memset(big, 0x5a, sizeof(big));
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, 7, 1, from_port, 0, big, sizeof(big))) == 0);
	CHECK(early.datagrams == 2 && early.length == 8192 + 0 && early.data[63] == 0x5a);
	/* a datagram that wants confirming is answered with the resume the sender waits for */
	sent_clear();
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, 7, 1, from_port, 1, answer, sizeof(answer))) == 0);
	CHECK(sent_count == 1 && word(0, 1) == RESUME_TX && word(0, 3) == from_port && word(0, 6) == 3 && word(0, 7) == 7);
	CHECK(word(0, 8) == RESUME_TX && word(0, 9) == 1 && word(0, 10) == from_port);
	CHECK(early.datagrams == 3);
	/* a resume of the sender's is not for anybody, and is not answered */
	sent_clear();
	CHECK(deliver(&link_a, v1(buffer, RESUME_TX, 3, 7, 1, from_port, 1, answer, sizeof(answer))) == 0);
	CHECK(early.datagrams == 3 && sent_count == 0);
	/* a failing link: the error comes back */
	send_error = EAGAIN;
	CHECK(OPS.send(early_port, 3, 7, question, sizeof(question)) == EAGAIN);
	send_error = 0;
	/* a port that closes tells the DSP it is gone, and is no port after */
	{
		struct events e = { 0 };
		struct hakios_qrtr_port *p = OPS.open(on_receive, on_server, &e);

		sent_clear();
		OPS.close(p);
		CHECK(sent_count == 1 && word(0, 1) == DEL_CLIENT && word(0, 6) == 3 && word(0, 7) == CONTROL);
		CHECK(word(0, 8) == DEL_CLIENT && word(0, 9) == 1 && word(0, 3) == word(0, 10));
		OPS.close(p);                           /* (twice: nothing) */
		OPS.close(NULL);
		CHECK(sent_count == 1);
	}
	/* a callback may close its own port, and the router is not stuck for it */
	{
		struct events e = { 0 };
		struct hakios_qrtr_port *p = OPS.open(on_receive, on_server, &e);

		e.close_on_server = p;
		CHECK(OPS.lookup(p, 0x61, 1, 0) == 0);
		CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, NEW_SERVER, 0x61, 1, 3, 20)) == 0);
		CHECK(e.servers == 1 && e.close_on_server == NULL);
		CHECK(OPS.send(p, 3, 7, question, 7) == EINVAL);        /* (it is closed) */
		CHECK(deliver(&link_a, v1_control(buffer, DATA, 3, DEL_SERVER, 0x61, 1, 3, 20)) == 0);
		CHECK(e.servers == 1);
	}
	/* sixteen ports, no more */
	{
		struct events e = { 0 };
		struct hakios_qrtr_port *ports[17];
		unsigned n = 0;

		while (n < 17 && (ports[n] = OPS.open(on_receive, on_server, &e)) != NULL) {
			n++;
		}
		CHECK(n == 14);                         /* (16, of which this file holds two: early and late) */
		for (unsigned i = 0; i < n; i++) {
			OPS.close(ports[i]);
		}
	}
}

static void
test_malformed(void)
{
	uint8_t data[8] = { 0 };
	unsigned long length;
	uint32_t peer, hellos, in_before, in, out, services;

	hakios_qrtr_stats(&peer, &hellos, &in_before, &out, &services);
	/* no datagram at all */
	log_clear();
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 1;
	CHECK(hakios_qrtr_receive(&link_a, buffer, 5) == EINVAL);                      /* (too short for either version) */
	CHECK(strcmp(logbuf, "wifi: qrtr: 5 bytes that are no datagram (they begin 0x1)\n") == 0);
	log_clear();
	buffer[0] = 9;
	CHECK(hakios_qrtr_receive(&link_a, buffer, 40) == EINVAL);                     /* (neither version) */
	CHECK(strcmp(logbuf, "wifi: qrtr: 40 bytes that are no datagram (they begin 0x9)\n") == 0);
	log_clear();
	CHECK(hakios_qrtr_receive(&link_a, NULL, 40) == EINVAL);
	CHECK(hakios_qrtr_receive(&link_a, buffer, 0) == EINVAL);
	CHECK(hakios_qrtr_receive(&link_a, buffer, 32 + 8192 + 1) == EINVAL);
	CHECK(log_empty());
	/* a size that does not fit what came */
	length = v1(buffer, DATA, 3, 7, 1, 0x4000, 0, data, 8);
	{
		uint32_t size = 9;

		memcpy(buffer + 20, &size, 4);
		log_clear();
		CHECK(hakios_qrtr_receive(&link_a, buffer, length) == EINVAL);
		CHECK(strcmp(logbuf, "wifi: qrtr: a datagram of 9 bytes in a message of 40: dropped\n") == 0);
		size = 0;
		memcpy(buffer + 20, &size, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, length) == EINVAL);
		size = 0xffffffffu;                     /* (an addition would wrap) */
		memcpy(buffer + 20, &size, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, length) == EINVAL);
		size = 0xfffffff9u;
		memcpy(buffer + 20, &size, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, length) == EINVAL);
		size = 8193;                            /* (more than a datagram holds, even where it fits) */
		memcpy(buffer + 20, &size, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, 32 + 8193 + 3) == EINVAL);
	}
	/* a packet with trailing bytes after its datagram is fine */
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, 1, 1, 0x3999, 0, data, 8) + 8) == 0);
	hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
	CHECK(in == in_before + 1);                     /* (none of the others was counted as having arrived) */
	/* version 2 with a size or options that do not fit */
	{
		uint32_t size = 100;
		uint8_t body[4] = { 0 };

		length = v2(buffer, DATA, 0, 3, 7, 1, 0x4000, 0, body, 4);
		memcpy(buffer + 4, &size, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, length) == EINVAL);
		length = v2(buffer, DATA, 0, 3, 7, 1, 0x4000, 200, body, 4);
		CHECK(hakios_qrtr_receive(&link_a, buffer, length - 150) == EINVAL);    /* (the options run past the end) */
		CHECK(hakios_qrtr_receive(&link_a, buffer, 15) == EINVAL);              /* (version 2's header is 16 bytes) */
	}
	/* a control message that is shorter than a control message is read as if it were zeros after: a hello, not a fault */
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, CONTROL, 1, CONTROL, 0, data, 1)) == 0);
	CHECK(deliver(&link_a, v1(buffer, DATA, 3, CONTROL, 1, CONTROL, 0, data, 5)) == 0);
}

static void
test_version2(void)
{
	uint32_t control[5] = { NEW_SERVER, 0x48, 0x0001, 3, 11 };
	uint8_t data[6] = { 1, 2, 3, 4, 5, 6 };
	struct events e = { 0 };
	struct hakios_qrtr_port *port = OPS.open(on_receive, on_server, &e);
	unsigned long length;
	uint32_t from_port;

	CHECK(OPS.lookup(port, 0x48, 1, 0) == 0);
	sent_clear();
	OPS.send(port, 3, 7, data, 6);
	from_port = word(0, 3);
	/* the control port is 0xfffe in 16 bits; the options (4 bytes) are stepped over */
	length = v2(buffer, DATA, 0, 3, 0xfffe, 1, 0xfffe, 4, control, sizeof(control));
	CHECK(deliver(&link_a, length) == 0);
	CHECK(e.servers == 1 && e.arrived == 1 && e.service == 0x48 && e.node == 3 && e.port == 11);
	/* data to the port: node and ports are 16 bits wide */
	length = v2(buffer, DATA, 0, 3, 0x1234, 1, from_port, 0, data, 6);
	CHECK(deliver(&link_a, length) == 0);
	CHECK(e.datagrams == 1 && e.from_node == 3 && e.from_port == 0x1234 && e.length == 6 && memcmp(e.data, data, 6) == 0);
	/* the confirm flag is bit 0 of the flags */
	sent_clear();
	length = v2(buffer, DATA, 1, 3, 0x1234, 1, from_port, 0, data, 6);
	CHECK(deliver(&link_a, length) == 0);
	CHECK(sent_count == 1 && word(0, 1) == RESUME_TX && word(0, 7) == 0x1234);
	/* the type is a byte of its own: a resume is nothing for the port */
	length = v2(buffer, RESUME_TX, 0, 3, 0x1234, 1, from_port, 0, data, 6);
	CHECK(deliver(&link_a, length) == 0);
	CHECK(e.datagrams == 2);
	OPS.close(port);
}

static void *
send_thread(void *argument)
{
	*(int *)argument = OPS.send(early_port, 3, 7, "abcd", 4);
	return NULL;
}

static volatile bool down_done;

static void *
down_thread(void *argument)
{
	hakios_qrtr_link_down(argument);
	down_done = true;
	return NULL;
}

/* a send that is inside the link when down is called: down waits for it, and it ends as a send */
static void
test_down_waits_for_a_send(void)
{
	pthread_t sender, downer;
	int result = -1;

	sent_clear();
	down_done = false;
	pthread_mutex_lock(&send_mutex);
	send_gate = true;
	send_inside = false;
	pthread_mutex_unlock(&send_mutex);
	pthread_create(&sender, NULL, send_thread, &result);
	pthread_mutex_lock(&send_mutex);
	while (!send_inside) {
		pthread_cond_wait(&send_cond, &send_mutex);
	}
	pthread_mutex_unlock(&send_mutex);
	pthread_create(&downer, NULL, down_thread, (void *)&link_a);
	usleep(100000);
	CHECK(!down_done);
	pthread_mutex_lock(&send_mutex);
	send_gate = false;
	pthread_cond_broadcast(&send_cond);
	pthread_mutex_unlock(&send_mutex);
	pthread_join(sender, NULL);
	pthread_join(downer, NULL);
	CHECK(down_done && result == 0);
}

static void
test_down(void)
{
	struct events watch = { 0 };
	struct hakios_qrtr_port *watcher = OPS.open(on_receive, on_server, &watch);
	uint32_t peer, hellos, in, out, services;
	int early_before = early.servers;

	CHECK(OPS.lookup(watcher, 0x48, 1, 0) == 0);
	hakios_qrtr_poll();
	CHECK(watch.arrived == 1);                      /* (0x48: from test_version2) */
	hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
	CHECK(services >= 2 && peer == 3);
	log_clear();
	test_down_waits_for_a_send();
	/* the servers are gone, the ports that looked them up are told, and the router has forgotten the node */
	CHECK(watch.gone == 1 && watch.servers == 2);
	CHECK(early.servers == early_before + 1 && early.gone == 2 && late.gone == 2);
	hakios_qrtr_stats(&peer, &hellos, &in, &out, &services);
	CHECK(services == 0 && peer == 0);
	CHECK(log_empty());                             /* (down says nothing of its own) */
	/* nothing comes in, nothing goes out */
	CHECK(deliver(&link_a, 36) == ENOTCONN);
	sent_clear();
	CHECK(OPS.send(early_port, 3, 7, "abcd", 4) == ENOTCONN);
	CHECK(sent_count == 0);
	hakios_qrtr_link_down(&link_a);                 /* (again: nothing) */
	hakios_qrtr_poll();
	CHECK(watch.servers == 2 && early.servers == early_before + 1);
	OPS.close(watcher);
	OPS.close(early_port);
	OPS.close(late_port);
	CHECK(sent_count == 0);                         /* (no peer: no one is told) */
}

/* a data callback is running when down is called: down waits for it, and none comes after it */
static volatile bool stop_receiving;

static void *
receiver_thread(void *argument)
{
	uint8_t packet[64];
	uint32_t body = 5, port = *(uint32_t *)argument;

	while (!stop_receiving) {
		v1(packet, DATA, 3, 7, 1, port, 0, &body, 4);
		(void)hakios_qrtr_receive(&link_b, packet, 36);
	}
	return NULL;
}

static void
test_down_waits_for_a_callback(void)
{
	struct events e = { 0 };
	struct hakios_qrtr_port *port;
	uint32_t number;
	pthread_t receiver, downer;
	int after;

	CHECK(hakios_qrtr_link_up(&link_a) == 0);       /* (a came down; this one begins afresh) */
	CHECK(hakios_qrtr_link_up(&link_b) == EBUSY);
	hakios_qrtr_link_down(&link_a);
	CHECK(hakios_qrtr_link_up(&link_b) == 0);       /* (and now b may) */
	port = OPS.open(on_receive, on_server, &e);
	sent_clear();
	CHECK(OPS.send(port, 3, 7, "abcd", 4) == 0);
	number = word(0, 3);
	e.slow = true;
	stop_receiving = false;
	down_done = false;
	pthread_create(&receiver, NULL, receiver_thread, &number);
	while (!e.slow_inside) {
		usleep(1000);
	}
	pthread_create(&downer, NULL, down_thread, (void *)&link_b);
	usleep(100000);
	CHECK(!down_done);                              /* (a callback of the link's is running) */
	e.slow = false;
	pthread_join(downer, NULL);
	CHECK(down_done);
	after = e.datagrams;
	usleep(50000);                                  /* (the other thread keeps trying) */
	CHECK(e.datagrams == after && after >= 1);
	stop_receiving = true;
	pthread_join(receiver, NULL);
	CHECK(e.datagrams == after);
	CHECK(deliver(&link_b, 36) == ENOTCONN);
	OPS.close(port);
}

int
main(void)
{
	test_link_state();
	test_hello();
	test_servers_and_lookup();
	test_datagrams();
	test_malformed();
	test_version2();
	test_down();
	test_down_waits_for_a_callback();
	printf("qrtr-test: %d checks, %d passed, %d failed\n", checks, checks - failures, failures);
	return failures != 0;
}
