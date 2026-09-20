/*
 * The Valve dongle HID interfaces. One per bond slot, and the path Steam
 * pairs a Steam Controller through.
 */

#ifndef PUCK_HID_H
#define PUCK_HID_H

/* Register the report descriptor and callbacks on every HID interface. MUST run
 * before usbd_init(): until this is called the HID class contributes no
 * interface descriptor and device assembly fails with -EINVAL.
 */
int puck_hid_register_all(void);

/*
 * Reload the Steam bonds from NVS. Needed in BOTH USB modes: the RF layer
 * derives each slot session address from its bond, so without this the puck
 * never beacons and no controller input arrives, in Pro Controller mode that
 * looks like a pad that enumerates perfectly with every button dead.
 */
void puck_hid_restore_bonds(void);

/*
 * This slot's Steam bond record, or NULL if the slot is empty.
 *
 * [0..4] proteus uuid, [4..8] ibex uuid, [8..24] the controller's 16-byte
 * serial. The RF layer needs the two uuids: they go in the E1 beacon so the
 * controller can recognise a puck it is bonded to, and the ibex uuid seeds this
 * slot's unique session address.
 */
const uint8_t *puck_hid_bond(int slot);

/*
 * Push one input report to Steam on this slot's interface.
 *
 * Steam decides a controller is CONNECTED by receiving these, not by the 0xB4
 * connection-state answer alone, so nothing appears online until the RF layer
 * starts forwarding the controller's 0x45/0x42 reports through here.
 */
int puck_hid_send_input(int slot, uint8_t rid, const uint8_t *data,
			uint16_t len);

/*
 * Push each slot's connection state to Steam (report 0x79). Call periodically.
 *
 * Steam does not notice a controller going away by itself. Without this it
 * keeps listing one that has been switched off, and a later re-pair then shows
 * up as a SECOND controller alongside the stale entry.
 */
void puck_hid_conn_task(void);

/*
 * Per-slot state of the outgoing report path, whether the host is draining
 * this interface, and how many reports were dropped because it was not.
 */
void puck_hid_dump_tx(void);

/* A power-off was relayed to this slot (the Steam+Y chord), so force a clean
 * disconnect and hold it through the controller's post-off report tail.
 */
void puck_hid_note_power_off(int slot);

#endif /* PUCK_HID_H */
