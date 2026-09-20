/*
 * The RF link to the Steam Controller.
 *
 * This is the puck's reason to exist: the controller talks to the puck over a
 * bare nRF radio ESB link, and the puck presents it to the Switch 2 (over
 * BLE/USB) or to Steam (over the USB dongle interfaces). Ported from the
 * Arduino build. See the porting notes in rf_link.c.
 *
 * Pairing happens entirely on the USB side: Steam writes a 24-byte bond record
 * to a slot (see puck_hid.c) and THAT record is what this layer beacons for. A
 * controller only answers a puck that names the uuids it was bonded with.
 */

#ifndef RF_LINK_H
#define RF_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Derive each bonded slot's unique session address and prepare the radio.
 * Call after the bonds have been restored from NVS.
 */
void rf_link_init(void);

/*
 * A slot's bond changed. Re-derive its session address.
 *
 * MUST be called whenever Steam writes or clears a bond, not just at startup.
 * rf_link_init() runs once when the radio is handed to RF, so a bond created
 * LATER (Steam re-pairing at runtime) never got an address derived: measured
 * 2026-08-17, a re-pair into slot 1 left it advertising base 00000000 prefix 00.
 * The controller adopts whatever the beacon names, so the link came up LIVE on
 * an all-zero address, exactly the degenerate value the derivation scrubs
 * because 0x00/0xFF correlate badly against the preamble. The symptom was
 * stuttering input, not an obvious failure.
 */
void rf_link_bond_changed(int slot);

/* Send one E1 beacon for `slot`.
 *
 * `discovery` picks the address it goes out on: the shared "ibex" rendezvous
 * (which is where a controller that has just powered on is listening), or this
 * slot's own session address once the controller has adopted it. The advertised
 * session parameters are identical either way.
 *
 * Returns true if a reply was received during the discovery listen window.
 */
bool rf_link_beacon(int slot, bool discovery);

/* Run the link for one iteration: beacon the bonded slots and report what
 * comes back. Safe to call from the main loop; does nothing unless the radio
 * has been handed to RF mode.
 */
void rf_link_task(void);

/* Whether a controller is answering polls on this slot, which is what the dongle
 * reports to Steam as connection state (command 0xB4).
 */
bool rf_link_slot_live(int slot);

/* Tighter (300 ms) liveness, used for the connection state reported to Steam. */
bool rf_link_slot_conn(int slot);

/* Whether the RF layer currently owns the radio. BLE and RF cannot share it,
 * so main.c hands ownership back and forth. See enter_rf_mode().
 */
void rf_link_set_active(bool active);

/* Dump every bond slot with its derived session address. Bound to console key 's'.
 * Two slots showing the SAME address means the same controller was bonded
 * twice, which makes both slots poll it and both forward its input.
 */
void rf_link_dump(void);

#endif /* RF_LINK_H */

/*
 * Relay a rumble level to the controller (Steam output report 0x80).
 *
 * `low` and `high` are the console's two motors as 16-bit levels. Zero for both
 * is a stop, which is relayed as a burst. The relay is NO-ACK and a lost stop
 * leaves the controller latched buzzing.
 */
void rf_link_rumble(int slot, uint16_t low, uint16_t high);

/*
 * Queue a raw command for the controller, carried inside the next poll.
 *
 * This is how Steam's own haptics reach the controller: Steam speaks the
 * controller's language directly, so the puck forwards its actuator commands
 * verbatim rather than translating them. `shots` repeats the command on
 * successive polls, because the relay is NO-ACK.
 */
void rf_link_relay(int slot, uint8_t rid, const uint8_t *data, uint8_t len,
		   uint8_t shots);
