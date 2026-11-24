/* SM8450 shared WLAN/BT power contract. Implementation belongs to BT WP1.
 * No hardware access or boot-time enable is introduced by this header.
 */
#ifndef _SYS_HAKIOS_RADIO_H_
#define _SYS_HAKIOS_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define HAKIOS_RADIO_WLAN  (1u << 0)
#define HAKIOS_RADIO_BT    (1u << 1)
#define HAKIOS_RADIO_ALL   (HAKIOS_RADIO_WLAN | HAKIOS_RADIO_BT)

/* Thread context, may sleep; 0 or positive errno. Invalid/empty masks return
 * EINVAL without writes. Transitions, including XO pulses, are serialized;
 * repeated requests are idempotent. Combined up runs WLAN then BT. A failed
 * up leaves that half disabled and preserves pre-existing owners; an earlier
 * successful half of a combined request stays up. The caller unwinds only
 * halves it acquired. No rail is ever voted off, including after failure.
 * Keep the bounded shared RSC transaction lock, quarantine uncertain votes,
 * and release that lock before sequence delays. A failure must not be
 * reported as an acquired owner or allow a conflicting retry.
 *
 * WLAN: preserve owned BT_EN, retain cold WLAN-only write order/delays; when
 * BT is low wait a fresh 100 ms before XO/WLAN pulse. BT: BT_EN low 50 ms,
 * then extra 100 ms if WLAN low (500 ms below 5 C or temperature unknown),
 * XO high 200 us, BT_EN high, XO low after 6 ms, settle 50 ms, check SW_CTRL.
 * Normal down changes only requested enable(s); BT down waits 100 ms.
 * No implicit reference counting: each half has one logical owner.
 */
int hakios_radio_up(unsigned mask);
int hakios_radio_down(unsigned mask);

/* Terminal panic handoff, all halves: cached mappings only, enable-low only.
 * No locks, waits, allocation, XO writes or rail votes. Prevent every in-flight
 * normal transition from reasserting enables after this handoff. Normal up
 * thereafter fails; this is not a recoverable controller restart operation.
 * PCIe still owns PERST/CLKREQ and its clocks/PHY undo.
 */
void hakios_radio_panic_quiesce(void);

#ifdef __cplusplus
}
#endif
#endif /* _SYS_HAKIOS_RADIO_H_ */
