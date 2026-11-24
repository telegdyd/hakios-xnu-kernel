/*
 * SM8450 QCA6490 Bluetooth: Wave-0 interfaces, implementations in BT WP2-5.
 * Based on iOSPlus10 negroni_bt.h at patches 0330-0338 (MIT).
 * Port changes: hakios names, public sys/ placement, C++ linkage, shared radio
 * ownership, fixed full HCI run (no negronibt level selector), stage log ABI.
 * Protocol signatures retain 0337 writable TX and 0338 from_rx handling.
 * Boot options and ownership: docs/bluetooth-contract.md.
 *
 * Copyright (c) 2026 iOSPlus10 contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef _SYS_HAKIOS_BT_H_
#define _SYS_HAKIOS_BT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stage numbers/names are a log ABI shared with WP7; no numbers for substeps.
 * Stages 12-15 require owner/host evidence; earlier driver success cannot
 * complete them. Stage 10 also requires a host-observed 60 s without restart.
 */
#define HAKIOS_BT_STAGES(X) \
	X(1, power) \
	X(2, uart) \
	X(3, rom) \
	X(4, baud) \
	X(5, patch) \
	X(6, nvm) \
	X(7, fw) \
	X(8, addr) \
	X(9, h4) \
	X(10, init) \
	X(11, lext) \
	X(12, settings) \
	X(13, switch) \
	X(14, pairing) \
	X(15, coexistence)

/* WP2: printf AND loader_log_line, irrespective of quiet packet logging.
 * bt: STAGE <n> <name>: ok <detail> OR ... FAILED <reason>
 * Thread context; table name only, detail <=255 bytes, CR/LF become spaces.
 * No addresses, identity seeds, link keys or pairing secrets in any log.
 */
void hakios_bt_stage(unsigned n, const char *name, int ok, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

/* WP2 UART. Caller serializes all access: bring-up/HCI first, then H4.
 * write waits until sent, 0 or positive errno; read returns immediately with
 * 0..max bytes, draining hardware into the ring even during a blocked write.
 * The H4 RX thread must not read while chip_on/off or HCI owns the UART.
 */
int hakios_bt_uart_write(const uint8_t *buf, uint32_t len);
uint32_t hakios_bt_uart_read(uint8_t *buf, uint32_t max);
int hakios_bt_uart_set_baud(uint32_t baud); /* 0 or errno; reject unsupported rates */
void hakios_bt_uart_flow(bool on); /* false: RFR not ready + CTS ignored */
uint32_t hakios_bt_uart_baud(void);
uint64_t hakios_bt_us(void); /* monotonic microseconds */
void hakios_bt_sleep_ms(uint32_t ms);
uint32_t hakios_bt_arg(const char *name, uint32_t def); /* suffix -> bt_<name> */

/* WP3: full stages 3-8, 0 or errno; image-backed firmware, selected persistent
 * identity and HCI readback. No raw address-bearing command/NVM dump.
 * WP2 calls with UART ownership after power/115200 setup; on failure WP2
 * lowers only BT_EN via radio_down(BT) and restores its saved GCC vote word.
 */
int hakios_bt_hci_run(void);

/* WP2 power worker entry points, called by WP4 with exclusive UART ownership.
 * off: radio_down(BT), rails/clocks retained. on: 115200 + radio_up(BT) +
 * full HCI run with the SAME identity; 0 or errno, BT_EN low on failure.
 * Failure cleanup also restores any changed GCC vote word.
 */
void hakios_bt_chip_off(void);
int hakios_bt_chip_on(void);

/* WP4: attach control + device nodes before bring-up; sends wait for ready.
 * start hands UART to RX thread; fail unblocks waiters and applies bt_dead.
 * inject copies one complete H4 packet (including type) before returning.
 * send_cmd serializes a complete shim-generated H4 command to the chip:
 * from_rx=true uses RX-safe delivery, never waits for that same RX worker.
 * Do not call the TX shim recursively. Returns 0 or positive errno.
 */
void hakios_bt_h4_attach(void);
void hakios_bt_h4_start(void);
void hakios_bt_h4_fail(const char *why);
void hakios_bt_h4_inject(const uint8_t *pkt, uint32_t len);
int hakios_bt_h4_send_cmd(const uint8_t *pkt, uint32_t len, bool from_rx);

/* WP5: complete, writable H4 packets; may rewrite within the supplied len.
 * PASS sends/delivers the original length, DONE consumes TX, DROP consumes RX.
 * Replacement packets use inject/send_cmd, which copy before returning.
 * init once at attach; session resets per-client state before a new client.
 * WP4 serializes shim TX/RX/session state as required by the port.
 */
#define HAKIOS_BT_SHIM_PASS  0
#define HAKIOS_BT_SHIM_DONE  1
#define HAKIOS_BT_SHIM_DROP  2
void hakios_bt_shim_init(void);
void hakios_bt_shim_session(void);
int hakios_bt_shim_tx(uint8_t *pkt, uint32_t len);
int hakios_bt_shim_rx(uint8_t *pkt, uint32_t len);

#ifdef __cplusplus
}
#endif
#endif /* _SYS_HAKIOS_BT_H_ */
