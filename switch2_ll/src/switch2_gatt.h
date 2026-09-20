/*
 * Switch 2 attribute table, ported from BleSwitch2Poc.ino's setupGatt().
 */

#ifndef SWITCH2_GATT_H
#define SWITCH2_GATT_H

#include <stdint.h>

/*
 * The puck's public address, wire order (least significant octet first).
 * Joy-Con 2 OUI, with the tail derived per device by puck_addr().
 *
 * Defined ONCE and used both to set the controller address and to build the
 * PAIR/SET_MAC announcement, because those two must be physically incapable of
 * drifting apart, on a real device the SET_MAC payload's tail IS the
 * controller's BT address, and keeping two copies in step by hand across two
 * files is what this avoids.
 *
 * 38:C6:CE is the real Joy-Con 2 OUI, and the tail is kept clear of any real
 * unit's so the puck cannot collide with a registered device.
 *
 * *** BUMP THE TAIL AFTER ANY CHANGE TO THE ATTRIBUTE TABLE'S SHAPE, OR WHEN
 * THE CONSOLE HOLDS A REGISTRATION THE FIRMWARE CAN NO LONGER HONOUR. *** A
 * bonded console caches the table and the link key with no way to discover
 * either has moved. A new address makes it treat the puck as a new device and
 * pair from scratch. The case that calls for it: a console holding a
 * registration whose derived key did not survive the disconnect, which it
 * answers by connecting and then refusing to speak at all.
 *
 * Do NOT bump this per boot. Every value mints a phantom controller in the
 * console's finite registry, which is what makes pairing backoff arrive after
 * 1-3 attempts instead of ~12.
 */
#define PUCK_OUI_BYTES 0xCE, 0xC6, 0x38

/*
 * The address, wire order, six bytes. The top three are the Joy-Con 2 OUI; the
 * bottom three are derived from this chip's FICR DEVICEID, so every unit gets
 * its own address the way real hardware does.
 *
 * Derived rather than hardcoded because a fixed address means every puck built
 * from this firmware presents the same identity, and a console stores a
 * registered controller BY address. Two of them near one console is the same
 * identity collision that bd283 causes, and it fails the same way: pairs once,
 * never reconnects.
 *
 * DEVICEID is fixed in factory ROM, so this satisfies the stability
 * requirement above. It does not change across boots or reflashes.
 */
const uint8_t *puck_addr(void);

/*
 * A stored attribute value. One pair of read/write handlers serves the whole
 * table by hanging one of these off each attribute's user_data.
 *
 * `len` is the CURRENT length and `cap` the buffer size. They are separate on
 * purpose: the extra 679d/b746 descriptors must be VARIABLE length, because a
 * fixed-length attribute bounces the console's 2-byte 0x0085 stream-trigger
 * write with ATT 0x0D and nothing observable happens.
 */
struct switch2_val {
	uint8_t *data;
	uint16_t len;
	uint16_t cap;
};

/* Register both services in call order rather than the linker's order.
 * Must be called after bt_enable(). Returns 0 on success.
 */
int switch2_gatt_init(void);

/* Print every attribute the stack assigned, with its handle. This is the check
 * that the layout landed. Zephyr numbers its own attributes differently from
 * the SoftDevice, so the alignment must be measured, never assumed.
 */
void switch2_gatt_dump(void);

/* Report whether the report channel landed on the real device's handles
 * (0x000E/0x000F/0x0010), and publish this table's bd282 handles into bd281.
 */
void switch2_gatt_check(void);

/* d5a9's 679d descriptor, the handle the console writes 0x0085 to in order to
 * start the report stream.
 */
uint16_t switch2_gatt_trigger_handle(void);

/* d5a9's value handle, where input reports are notified from. */
uint16_t switch2_gatt_report_handle(void);

/* Tear down per-link state: stops the report stream and drops the derived link
 * key. Must be called from the disconnected callback, or the stream thread will
 * keep notifying into a dead connection.
 */
void switch2_gatt_disconnected(void);

/* Clear all per-link state before handing the radio to RF. bt_disable() leaves
 * the dynamic services and their CCC configs registered, so stale subscription
 * state would otherwise survive into the next BLE session.
 */
void switch2_gatt_radio_release(void);

/* Re-install the derived link key for a new connection, so a bonded reconnect
 * can encrypt. Call from the connected callback.
 */
struct bt_conn;
void switch2_gatt_connected(struct bt_conn *conn);

/*
 * Registration is complete, switch the advertisement to WAKE mode.
 *
 * Implemented in main.c, which owns the advertising payload. Called after the
 * console's LAST phase-6 command (0x0C/0x04), not merely on encryption: doing
 * it earlier hid the puck from the Change Grip/Order screen mid-registration.
 *
 * Until this happens the puck presents as a brand-new controller forever, so
 * the console re-pairs it every time that screen is opened and an autonomous
 * wake stays out of reach: a sleeping Switch scans for ITS OWN address in the
 * reconnect field, and a pairing-mode advertiser is invisible to it by
 * design.
 */
void switch2_registration_complete(void);

/*
 * Arm a HOME press. The WAKE action.
 *
 * A real Joy-Con wakes a sleeping Switch by sending HOME in its input report
 * stream. This is what the puck will fire when the Steam Controller macro asks
 * it to wake the console, before it drops back to RF.
 *
 * Note the console also needs a few face-button presses afterwards or the lock
 * screen puts it back to sleep; the continuous SL+SR auto-press covers that.
 */
void switch2_gatt_arm_wake(void);

/* Run the post-wake unlock: press a button three times to clear the lock
 * screen, then stop. Without it the console goes straight back to sleep.
 */
void switch2_gatt_begin_unlock(void);

/* Cycle which single button the unlock sequence presses. Only BTN_B's bit is
 * documented, so the rest are candidates to sweep rather than known values.
 */
void switch2_gatt_cycle_unlock_button(void);

/* Continuous SL+SR (needed for the Change Grip/Order slot confirmation) on or
 * off. A real Joy-Con presses NOTHING on a normal reconnect.
 */
void switch2_gatt_set_registering(bool on);

/* Mark that advertising was started AS a wake action, so the unlock sequence
 * fires once the console opens the report stream.
 */
void switch2_gatt_set_wake_pending(bool on);

/*
 * The wake is finished, stop advertising, drop the link, go quiet.
 *
 * Implemented in main.c. This exists because a puck that holds a connection
 * forever PREVENTS THE CONSOLE FROM EVER SLEEPING AGAIN. In the product this is
 * the moment BLE hands back to RF.
 */
void switch2_wake_complete(void);

/*
 * Ask for a sleeping console to be woken. Held QAM triggers it.
 *
 * Implemented in main.c, called from the RF thread. Safe to call at any time:
 * it validates that a console is stored and that a wake is not already in
 * flight, and defers the radio handover to the work queue rather than tearing
 * down RF from inside the RF thread.
 *
 * `slot` is the controller that asked, and hears the chime (wake.sound) that
 * plays before the handover.
 */
void switch2_request_wake(int slot);

/*
 * Borrow the radio to register with a console, then give it back. Works in any
 * mode, so registration can happen at the console rather than at a PC.
 */
void switch2_request_register(void);

/* True once a console has registered with this puck. */
bool switch2_have_host(void);

#endif /* SWITCH2_GATT_H */
