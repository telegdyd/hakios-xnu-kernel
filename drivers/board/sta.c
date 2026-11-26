/*
 * HakiOS: what stands between the two sides of the station boundary
 * (sys/board/sta.h).
 *
 * Whoever wants a network asks from wherever it happens to be: an ioctl of wifid's, the thread
 * that brings the radio up. The station side wants to be called by one thread, one call at a
 * time, and a scan or a join takes seconds. So what is asked for is noted here and done by a
 * thread of this file's; the asker returns at once and hears the outcome as an event. One
 * request of each kind can wait: a second scan asked for while the first has not begun
 * replaces it, as does a second join.
 *
 * What the last scan heard is kept here, so that it can be read without entering the station
 * side, and so is the list of channels.
 *
 * The network the kernel was given in a file is a standing request: joined at the start, and
 * again whenever the radio is turned back on (Settings' switch), until somebody asks for
 * another network -- as Darwin itself rejoins the networks it knows. If that other network cannot
 * be joined (a wrong passphrase), the standing one is joined again. And when the access point sends
 * the station away (a router restarting, or dropping a station it thinks idle: deauthentication),
 * the standing network is joined again after a moment, then less and less often while that fails:
 * wifid does not, since it never joined that network itself.
 *
 * While a network is joined and nothing is asked for, the thread measures its signal once a
 * second (the station side's signal(), else its last beacon) and says so (HAKIOS_STA_SIGNAL):
 * Darwin hears how the link goes without asking (WIFI-PLAN.md P3b). A country request that
 * leaves another country in force is said too (HAKIOS_STA_COUNTRY).
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/board/input.h>
#include <sys/board/sta.h>
#include <sys/systm.h>

#include "kernel.h"

#define HEARD_MOST      128
#define CHANNELS_MOST   96
#define SIGNAL_EVERY_MS 1000            /* how often the signal of the network joined is measured */

static LCK_GRP_DECLARE(sta_lock_group, "hakios-sta");
static LCK_MTX_DECLARE(sta_lock, &sta_lock_group);

static const struct hakios_sta_ops *sta_ops;   /* the station side, while there is one */
static bool sta_thread_made;
static bool sta_calls;                          /* the thread is in a call of the station side's */
static unsigned sta_reads;                      /* ... and so many others are in its state() */
static bool sta_power_change;                  /* power may free/recreate the driver's radio */
static bool sta_on;                             /* the radio's power, as last asked for */
static bool sta_joined;                         /* a network is joined (as the events have said) */

/* what has been asked for and not begun */
static bool asked_power, asked_power_on, asked_scan, asked_join, asked_leave, asked_country;
static char asked_country_what[2];
static char country_now[2];                     /* of the domain in force, as the station side last said (zero: none) */
static unsigned country_done;                   /* country requests carried out (whoever asked waits for it to move) */
static int country_error;                       /* ... and how the last one went */
static struct hakios_sta_scan asked_scan_what;
static struct hakios_sta_join asked_join_what;
static bool asked_join_by_darwin;               /* ... asked for with hakios_sta_ask_join(), not by this file itself */
static struct hakios_sta_join standing;        /* the network joined whenever the radio comes on (ssid_length 0: none) */
static struct hakios_sta_join fallback;        /* ... as it was before a join that has not come through yet */
static uint64_t rejoin_at;                      /* when the standing network is joined again after it was lost (0: not) */
static unsigned rejoin_tries;                   /* ... and how many times that has been tried since it was last joined */
static bool rejoining;                          /* the join under way is such a one */
static bool join_by_darwin;                     /* the join under way, or the last, was asked_join_by_darwin (the thread's) */

static struct hakios_sta_bss *heard;           /* by the last scan that came to an end */
static unsigned heard_count;
static uint64_t scan_generation;
static struct hakios_sta_channel channels[CHANNELS_MOST];
static unsigned channel_count;

static void
heard_one(const struct hakios_sta_bss *bss, __unused void *context)
{
	/* (only the thread writes the list, and it reads the count without the lock) */
	if (heard_count < HEARD_MOST) {
		lck_mtx_lock(&sta_lock);
		heard[heard_count++] = *bss;
		lck_mtx_unlock(&sta_lock);
	}
}

/*
 * The standing network to be joined again (sta_lock held): after 2 s, three times, then after 30 s, as long as it
 * is the standing one and nobody asks for anything else.
 */
static void
rejoin_later(void)
{
	uint64_t deadline;

	if (standing.ssid_length == 0 || !sta_on || asked_join || asked_leave) {
		return;
	}
	clock_interval_to_deadline(rejoin_tries < 3 ? 2000 : 30000, NSEC_PER_MSEC, &deadline);
	rejoin_tries++;
	rejoin_at = deadline;
}

static void
sta_runs(__unused void *parameter, __unused wait_result_t result)
{
	static struct hakios_sta_scan scan;
	static struct hakios_sta_join join;

	for (;;) {
		const struct hakios_sta_ops *ops;
		bool power = false, power_on = false, leave = false, do_join = false, do_rejoin = false, do_scan = false;
		bool country = false, sample = false;
		char code[2] = { 0 };
		int error = 0;

		lck_mtx_lock(&sta_lock);
		while (sta_ops == NULL || !(asked_power || asked_leave || asked_join || asked_scan || asked_country)) {
			uint64_t deadline = 0, signal_at = 0;

			if (rejoin_at != 0 && sta_ops != NULL && mach_absolute_time() >= rejoin_at) {
				rejoin_at = 0;
				if (standing.ssid_length != 0 && sta_on) {
					asked_join_what = standing;
					asked_join = rejoining = true;
					asked_join_by_darwin = false;
				}
				continue;
			}
			/* (with no station, a rejoin's time come and gone would wake this at once, every time: not slept to) */
			if (sta_ops != NULL && rejoin_at != 0) {
				deadline = rejoin_at;
			}
			/* (joined: a second with nothing asked for is the time to measure the signal) */
			if (sta_ops != NULL && sta_joined) {
				clock_interval_to_deadline(SIGNAL_EVERY_MS, NSEC_PER_MSEC, &signal_at);
				if (deadline == 0 || signal_at < deadline) {
					deadline = signal_at;
				} else {
					signal_at = 0;
				}
			}
			if (deadline == 0) {
				lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_ops, THREAD_UNINT);
				continue;
			}
			if (lck_mtx_sleep_deadline(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_ops, THREAD_UNINT, deadline) ==
			    THREAD_TIMED_OUT && signal_at != 0 && sta_ops != NULL && sta_joined && !(asked_power || asked_leave ||
			    asked_join || asked_scan || asked_country)) {
				sample = true;
				break;
			}
		}
		ops = sta_ops;
		/* one thing at a time, in the order that makes sense when several wait */
		if (sample) {
			/* (below) */
		} else if (asked_power) {
			power = true;
			power_on = asked_power_on;
			asked_power = false;
		} else if (asked_country) {     /* (before a scan or a join: they are to use the country's channels) */
			country = true;
			memcpy(code, asked_country_what, sizeof(code));
			asked_country = false;
		} else if (asked_leave) {
			leave = true;
			asked_leave = false;
		} else if (asked_join) {
			do_join = true;
			do_rejoin = rejoining;
			rejoining = false;
			join_by_darwin = asked_join_by_darwin;
			join = asked_join_what;
			bzero(&asked_join_what, sizeof(asked_join_what));
			asked_join = false;
		} else {
			do_scan = true;
			scan = asked_scan_what;
			asked_scan = false;
		}
		sta_calls = true;
		if (power) {
			sta_power_change = true;
			while (sta_reads != 0)
				lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_calls, THREAD_UNINT);
		}
		lck_mtx_unlock(&sta_lock);

		if (power) {
			struct hakios_sta_state *state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO);
			bool was_joined = false;

			if (state != NULL) {
				ops->state(state);
				was_joined = state->phase == HAKIOS_STA_JOINED;
				kfree_data(state, sizeof(*state));
			}
			if (!power_on) {
				lck_mtx_lock(&sta_lock);
				sta_joined = false;
				lck_mtx_unlock(&sta_lock);
			}
			error = ops->power(power_on);
			if (!power_on && was_joined) {
				hakios_apple80211_event(HAKIOS_STA_LINK, 0);
			}
			hakios_apple80211_event(HAKIOS_STA_POWER, error == 0 ? power_on : !power_on);
			lck_mtx_lock(&sta_lock);
			if (error != 0 && !asked_power) {
				/*
				 * (sta_on was set when the power was asked for: the change did not happen, and what is said of the power --
				 * hakios_sta_powered(), the get-power request -- is to be what the radio is, as the event says it)
				 */
				sta_on = !power_on;
			}
			if (error == 0 && power_on && standing.ssid_length != 0 && !asked_join) {
				asked_join_what = standing;
				asked_join = true;
				asked_join_by_darwin = false;
			}
			lck_mtx_unlock(&sta_lock);
		} else if (country) {
			struct hakios_sta_channel *now = kalloc_data(CHANNELS_MOST * sizeof(*now), Z_WAITOK);
			char in_force[2] = { 0 };
			unsigned count = 0;
			bool changed;

			error = ops->country != NULL ? ops->country(code, in_force) : ENOTSUP;   /* (a station side may have no country of its own) */
			if (now != NULL) {
				count = ops->channels(now, CHANNELS_MOST);
				if (count > CHANNELS_MOST) {
					count = CHANNELS_MOST;
				}
			}
			lck_mtx_lock(&sta_lock);
			changed = in_force[0] != 0 && memcmp(country_now, in_force, sizeof(country_now)) != 0;
			memcpy(country_now, in_force, sizeof(country_now));
			/* the new country's channels may be where the standing network is (13 at 2.4 GHz in Europe): joined again now */
			if (changed && error == 0 && standing.ssid_length != 0 && sta_on && !sta_joined && !asked_join && !asked_leave) {
				asked_join_what = standing;
				asked_join = true;
				asked_join_by_darwin = false;
			}
			if (now != NULL) {
				memcpy(channels, now, count * sizeof(*now));
				channel_count = count;
			}
			country_error = error;
			country_done++;
			lck_mtx_unlock(&sta_lock);
			thread_wakeup((event_t)&country_done);
			if (now != NULL) {
				kfree_data(now, CHANNELS_MOST * sizeof(*now));
			}
			hakios_wifi_tell("wifi: the country %c%c: %s (in force: %c%c, %u channels)\n", code[0], code[1], error == 0 ? "done" :
			    "NOT DONE", in_force[0] ? in_force[0] : '-', in_force[1] ? in_force[1] : '-', count);
			if (changed) {
				hakios_apple80211_event(HAKIOS_STA_COUNTRY, 0);
			}
		} else if (leave) {
			lck_mtx_lock(&sta_lock);
			sta_joined = false;
			lck_mtx_unlock(&sta_lock);
			ops->leave();
			hakios_apple80211_event(HAKIOS_STA_LINK, 0);
		} else if (do_join) {
			lck_mtx_lock(&sta_lock);
			sta_joined = false;
			lck_mtx_unlock(&sta_lock);
			error = ops->join(&join);
			bzero(&join, sizeof(join));
			hakios_apple80211_event(HAKIOS_STA_JOIN_DONE, error);
			if (error == 0) {
				lck_mtx_lock(&sta_lock);
				sta_joined = true;
				lck_mtx_unlock(&sta_lock);
				hakios_apple80211_event(HAKIOS_STA_LINK, 1);
			}
			/* a join that did not come through (a wrong passphrase) leaves things as they were: the standing network again */
			lck_mtx_lock(&sta_lock);
			if (error != 0 && fallback.ssid_length != 0 && !asked_join && !asked_leave && !asked_power) {
				standing = asked_join_what = fallback;
				asked_join = true;
				asked_join_by_darwin = false;
			}
			bzero(&fallback, sizeof(fallback));
			if (error == 0) {
				rejoin_tries = 0;
			} else if (do_rejoin) {
				rejoin_later();
			}
			lck_mtx_unlock(&sta_lock);
			if (do_rejoin && error == 0) {
				hakios_wifi_tell("wifi: THE NETWORK OF THE FILE IS JOINED AGAIN after it was lost\n");
			} else if (do_rejoin) {
				hakios_wifi_tell("wifi: the network of the file was lost and is not joined again yet (%d): it is tried "
				    "again later\n", error);
			}
		} else if (do_scan) {
			error = ops->scan(&scan);
			lck_mtx_lock(&sta_lock);
			heard_count = 0;
			lck_mtx_unlock(&sta_lock);
			ops->heard(heard_one, NULL);
			lck_mtx_lock(&sta_lock);
			scan_generation++;
			lck_mtx_unlock(&sta_lock);
			hakios_apple80211_event(HAKIOS_STA_SCAN_DONE, error);
		} else if (sample) {
			struct hakios_sta_state *state;
			bool still, known;
			int dbm = 0;

			/* the radio's own measurement, else the last beacon's (a network lost meanwhile is not measured) */
			known = ops->signal != NULL && ops->signal(&dbm) == 0;
			if (!known && (state = kalloc_data(sizeof(*state), Z_WAITOK | Z_ZERO)) != NULL) {
				ops->state(state);
				known = state->phase == HAKIOS_STA_JOINED;
				dbm = state->bss.rssi;
				kfree_data(state, sizeof(*state));
			}
			lck_mtx_lock(&sta_lock);
			still = sta_joined;
			lck_mtx_unlock(&sta_lock);
			if (still && known) {
				hakios_apple80211_event(HAKIOS_STA_SIGNAL, dbm);
			}
		}

		lck_mtx_lock(&sta_lock);
		sta_calls = false;
		sta_power_change = false;
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&sta_calls);
	}
}

/* ---- the station side ---------------------------------------------------------------------------------- */

void
hakios_sta_attach(const struct hakios_sta_ops *ops)
{
	struct hakios_sta_channel given[CHANNELS_MOST];
	unsigned count;
	char in_force[2] = { 0 };
	thread_t thread;

	if (ops == NULL) {
		lck_mtx_lock(&sta_lock);
		sta_ops = NULL;
		while (sta_calls || sta_reads != 0) {
			lck_mtx_sleep(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&sta_calls, THREAD_UNINT);
		}
		/*
		 * What was asked for and not begun is forgotten only now that no call is under way: such a call, as it ends,
		 * asks for what comes after it (the standing network after a join that failed, its rejoin later, its join
		 * once the radio is on), and the next station is not to act on any of that. A rejoin of the station gone is
		 * not tried by the next either.
		 */
		asked_power = asked_scan = asked_join = asked_leave = asked_country = false;
		bzero(&asked_join_what, sizeof(asked_join_what));
		rejoin_at = 0;
		rejoining = false;
		/* (a join that was asked for and never begun did not come through: the standing network is as it was before it) */
		if (standing.ssid_length == 0) {
			standing = fallback;
		}
		bzero(&fallback, sizeof(fallback));
		heard_count = channel_count = 0;
		sta_joined = false;
		bzero(country_now, sizeof(country_now));
		country_error = ENXIO;
		country_done++;                 /* (whoever waits for a country hears that there is no radio) */
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&country_done);
		return;
	}
	if (heard == NULL) {
		heard = kalloc_data(HEARD_MOST * sizeof(*heard), Z_WAITOK | Z_ZERO);
	}
	if (heard == NULL) {
		hakios_wifi_tell("wifi: no memory for what a scan hears: nobody can ask the radio for a network\n");
		return;
	}
	if (!sta_thread_made) {
		if (kernel_thread_start(sta_runs, NULL, &thread) != KERN_SUCCESS) {
			hakios_wifi_tell("wifi: no thread for the station: nobody can ask the radio for a network\n");
			return;
		}
		thread_set_thread_name(thread, "wifi-station");
		thread_deallocate(thread);
		sta_thread_made = true;
	}
	/* (filled aside, then put in place under the lock: an ioctl may be reading the list at any time) */
	count = ops->channels(given, CHANNELS_MOST);
	if (count > CHANNELS_MOST) {
		count = CHANNELS_MOST;          /* (the radio's driver is built elsewhere: what it says is held to the list's room) */
	}
	if (ops->country != NULL) {
		ops->country(NULL, in_force);
	}
	lck_mtx_lock(&sta_lock);
	memcpy(channels, given, count * sizeof(*channels));
	memcpy(country_now, in_force, sizeof(country_now));
	channel_count = count;
	sta_ops = ops;
	sta_on = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	hakios_apple80211_event(HAKIOS_STA_POWER, 1);
}

void
hakios_sta_lost(int error)
{
	lck_mtx_lock(&sta_lock);
	sta_joined = false;
	lck_mtx_unlock(&sta_lock);
	hakios_apple80211_event(HAKIOS_STA_LINK, error != 0 ? error : ECONNRESET);
	lck_mtx_lock(&sta_lock);
	if (sta_ops != NULL) {
		rejoin_later();
	}
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
}

/* ---- who wants a network ------------------------------------------------------------------------------- */

bool
hakios_sta_present(void)
{
	return sta_ops != NULL;
}

bool
hakios_sta_powered(void)
{
	return sta_ops != NULL && sta_on;
}

bool
hakios_sta_busy(void)
{
	bool busy;

	lck_mtx_lock(&sta_lock);
	busy = sta_ops != NULL && (sta_calls || asked_power || asked_leave || asked_join || asked_scan);
	lck_mtx_unlock(&sta_lock);
	return busy;
}

int
hakios_sta_ask_power(bool on)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	sta_on = on;
	asked_power = true;
	asked_power_on = on;
	if (!on) {
		asked_scan = asked_join = asked_leave = false;
		rejoin_at = 0;
	}
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

int
hakios_sta_ask_scan(const struct hakios_sta_scan *what)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL || !sta_on) {
		lck_mtx_unlock(&sta_lock);
		return sta_ops == NULL ? ENXIO : ENETDOWN;
	}
	asked_scan_what = *what;
	asked_scan = true;
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

/* A join asked for (sta_lock held). */
static int
ask_join(const struct hakios_sta_join *what, bool by_darwin)
{
	if (sta_ops == NULL || !sta_on) {
		return sta_ops == NULL ? ENXIO : ENETDOWN;
	}
	if (standing.ssid_length != 0) {
		fallback = standing;
	}
	bzero(&standing, sizeof(standing));     /* (whoever asks for a network now has the say) */
	rejoin_at = 0;
	asked_join_what = *what;
	asked_join = true;
	asked_join_by_darwin = by_darwin;
	asked_leave = false;
	return 0;
}

int
hakios_sta_ask_join_standing(const struct hakios_sta_join *what)
{
	int error;

	/*
	 * (made the standing network in the same hold of the lock as its join is asked for: in between, the thread could
	 * take the join, fail it and put back the standing network it replaced -- and this would then make the network
	 * that failed the standing one)
	 */
	lck_mtx_lock(&sta_lock);
	if ((error = ask_join(what, false)) == 0) {
		standing = *what;
	}
	lck_mtx_unlock(&sta_lock);
	if (error == 0) {
		thread_wakeup((event_t)&sta_ops);
	}
	return error;
}

int
hakios_sta_ask_join(const struct hakios_sta_join *what)
{
	int error;

	lck_mtx_lock(&sta_lock);
	error = ask_join(what, true);
	lck_mtx_unlock(&sta_lock);
	if (error == 0) {
		thread_wakeup((event_t)&sta_ops);
	}
	return error;
}

bool
hakios_sta_join_by_darwin(void)
{
	return join_by_darwin;
}

int
hakios_sta_ask_leave(void)
{
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	asked_leave = true;
	asked_join = false;
	rejoin_at = 0;
	bzero(&asked_join_what, sizeof(asked_join_what));
	bzero(&fallback, sizeof(fallback));
	lck_mtx_unlock(&sta_lock);
	thread_wakeup((event_t)&sta_ops);
	return 0;
}

/*
 * Unlike the others this waits for the outcome (at most 15 s: a scan under way comes first), because
 * whoever sets a country goes by the answer: 0 once the radio is in it, or why not.
 */
int
hakios_sta_ask_country(const char *code)
{
	uint64_t deadline;
	unsigned ticket;
	int error;

	clock_interval_to_deadline(15, NSEC_PER_SEC, &deadline);
	lck_mtx_lock(&sta_lock);
	if (sta_ops == NULL) {
		lck_mtx_unlock(&sta_lock);
		return ENXIO;
	}
	memcpy(asked_country_what, code, sizeof(asked_country_what));
	asked_country = true;
	ticket = country_done;
	thread_wakeup((event_t)&sta_ops);
	while (country_done == ticket) {
		if (lck_mtx_sleep_deadline(&sta_lock, LCK_SLEEP_DEFAULT, (event_t)&country_done, THREAD_UNINT, deadline) ==
		    THREAD_TIMED_OUT) {
			break;
		}
	}
	error = country_done == ticket ? ETIMEDOUT : country_error;
	lck_mtx_unlock(&sta_lock);
	return error;
}

bool
hakios_sta_country(char *code)
{
	bool known;

	lck_mtx_lock(&sta_lock);
	known = sta_ops != NULL && country_now[0] != 0;
	if (known) {
		memcpy(code, country_now, sizeof(country_now));
	}
	lck_mtx_unlock(&sta_lock);
	return known;
}

bool
hakios_sta_heard(unsigned index, struct hakios_sta_bss *bss)
{
	bool there;

	lck_mtx_lock(&sta_lock);
	if ((there = index < heard_count)) {
		*bss = heard[index];
	}
	lck_mtx_unlock(&sta_lock);
	return there;
}

uint64_t
hakios_sta_scan_generation(void)
{
	uint64_t generation;
	lck_mtx_lock(&sta_lock);
	generation = scan_generation;
	lck_mtx_unlock(&sta_lock);
	return generation;
}

void
hakios_sta_state(struct hakios_sta_state *state)
{
	const struct hakios_sta_ops *ops;

	bzero(state, sizeof(*state));
	lck_mtx_lock(&sta_lock);
	ops = sta_power_change ? NULL : sta_ops;
	if (ops != NULL) {
		sta_reads++;
	}
	lck_mtx_unlock(&sta_lock);
	if (ops != NULL) {
		ops->state(state);
		lck_mtx_lock(&sta_lock);
		sta_reads--;
		lck_mtx_unlock(&sta_lock);
		thread_wakeup((event_t)&sta_calls);
	}
}

unsigned
hakios_sta_channels(struct hakios_sta_channel *out, unsigned most)
{
	unsigned count;

	lck_mtx_lock(&sta_lock);
	count = channel_count < most ? channel_count : most;
	memcpy(out, channels, count * sizeof(*out));
	lck_mtx_unlock(&sta_lock);
	return count;
}

bool
hakios_sta_address(uint8_t *address)
{
	const struct hakios_sta_ops *ops = sta_ops;

	if (ops != NULL) {
		memcpy(address, ops->address, 6);
	}
	return ops != NULL;
}
