/*
 * The Valve dongle's HID interfaces, one per bond slot.
 *
 * This is the pairing path. A Steam Controller cannot be paired over the air:
 * Steam drives a control channel over HID feature reports on these interfaces,
 * and the bond record it produces, [proteus_uuid 4][ibex_uuid 4][serial 16], is
 * what the RF layer later beacons for. Without these interfaces nothing can be
 * bonded at all.
 *
 * Registering the report descriptor here is also what makes the USB device
 * assemble: Zephyr's HID class contributes no interface descriptor until
 * hid_device_register() has been called, and usbd_init() answers -EINVAL,
 * "this class added no interface", for one merely registered as a class.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/usb_buf.h>
#include <string.h>
#include <hal/nrf_ficr.h>

#include "puck_hid.h"
#include "puck_usb.h"
#include "switch2_store.h"
#include "rf_link.h"

/*
 * PUCK_HID_DESC, byte-for-byte from the Arduino firmware.
 *
 * Report IDs, for orientation:
 *   0x40 mouse, 0x41 keyboard   the dongle's lizard-mode HID
 *   0x42/0x43/0x44/0x45/0x79/0x7B  input reports (controller state)
 *   0x80..0x89                 output reports
 *   0x01/0x02                  FEATURE reports, 63 bytes: Steam's control
 *                              channel, and where pairing happens
 *
 * Steam's controller and haptics paths expect this shape exactly, so it is
 * copied rather than regenerated.
 */
static const uint8_t PUCK_HID_DESC[] =
{
	0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x40, 0x09, 0x01, 0xA1, 0x00,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x02, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01, 0x05, 0x01,
	0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x02,
	0x81, 0x06, 0x95, 0x01, 0x09, 0x38, 0x81, 0x06, 0x05, 0x0C, 0x0A, 0x38,
	0x02, 0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0, 0x05, 0x01, 0x09, 0x06, 0xA1,
	0x01, 0x85, 0x41, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25,
	0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x81, 0x01, 0x19, 0x00, 0x29,
	0x65, 0x15, 0x00, 0x25, 0x65, 0x75, 0x08, 0x95, 0x06, 0x81, 0x00, 0xC0,
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x42, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x35, 0x09, 0x42, 0x81, 0x02, 0x85, 0x44,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x05, 0x09, 0x44, 0x81,
	0x02, 0x85, 0x79, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x01,
	0x09, 0x79, 0x81, 0x02, 0x85, 0x43, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x0E, 0x09, 0x43, 0x81, 0x02, 0x85, 0x7B, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x0C, 0x09, 0x7B, 0x81, 0x02, 0x85, 0x45,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x2D, 0x09, 0x45, 0x81,
	0x02, 0x85, 0x80, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09,
	0x09, 0x80, 0x91, 0x02, 0x85, 0x81, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x07, 0x09, 0x81, 0x91, 0x02, 0x85, 0x82, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x82, 0x91, 0x02, 0x85, 0x83,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x09, 0x09, 0x83, 0x91,
	0x02, 0x85, 0x84, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x08,
	0x09, 0x84, 0x91, 0x02, 0x85, 0x85, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x03, 0x09, 0x85, 0x91, 0x02, 0x85, 0x86, 0x15, 0x00, 0x26,
	0xFF, 0x00, 0x75, 0x08, 0x95, 0x03, 0x09, 0x86, 0x91, 0x02, 0x85, 0x87,
	0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F, 0x09, 0x87, 0x91,
	0x02, 0x85, 0x89, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x3F,
	0x09, 0x89, 0x91, 0x02, 0x85, 0x88, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75,
	0x08, 0x95, 0x3F, 0x09, 0x88, 0x91, 0x02, 0x85, 0x01, 0x95, 0x3F, 0x09,
	0x01, 0xB1, 0x02, 0x85, 0x02, 0x95, 0x3F, 0x09, 0x01, 0xB1, 0x02, 0xC0
};


/* ---- Steam's control channel -----------------------------------------------
 *
 * Feature reports 0x01 and 0x02 carry a command channel, framed
 * [cmd][len][payload...] in a 63-byte report. Report id 1 is nominally the
 * bonded CONTROLLER, id 2 the puck/dongle itself, an important distinction
 * for identity commands, because Steam matches a controller to its bond by
 * comparing what id 1 reports against the bond record.
 *
 * Observed live on enumeration: Steam probes all four slots with 0x87, then
 * reads 0xA3 on slot 0.
 */
#define CMD_ATTRIBUTES 0x83	/* device attributes */
#define CMD_SETTINGS   0x87	/* SET_SETTINGS_VALUES, N x [id][val16] */
#define CMD_GET_SETTING 0x89	/* GET_SETTINGS_VALUES */
#define CMD_BOND_WRITE 0xA2	/* write/clear THIS slot's bond record */
#define CMD_BOND_READ  0xA3	/* read THIS slot's bond record */
#define CMD_PAIRING    0xAD	/* pairing mode on/off */
#define CMD_STRINGS    0xAE	/* string attributes (serials) */
#define CMD_CONNECTED  0xB4	/* is a controller live on this slot? */

/*
 * Device attributes, product 0x1304 (the Proteus puck). Copied verbatim.
 *
 * On report id 1 the low byte of the product is flipped 0x04 -> 0x02, so the
 * CONTROLLER reports 0x1302 rather than the dongle's 0x1304. Steam checks this:
 * seeing "a controller with a dongle's id" drops it to the legacy 0x81
 * CLEAR_DIGITAL_MAPPINGS init path, whose rapid re-arm storm is audible as the
 * connect buzz. With the right id it uses the modern quiet 0x87 path.
 */
static const uint8_t ATTR83[25] = {
	0x01, 0x04, 0x13, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x0A, 0xF2, 0xF9, 0xD2, 0x68, 0x04, 0x53, 0xD0,
	0x18, 0x6A, 0x09, 0x47, 0x00, 0x00, 0x00
};

/* The bond record Steam writes: [proteus_uuid 4][ibex_uuid 4][serial 16]. */
#define BOND_LEN 24
static uint8_t g_bond[PUCK_HID_SLOTS][BOND_LEN];
static bool g_bond_used[PUCK_HID_SLOTS];

/* Last time Steam wrote anything, proof it is alive and reacting. */
static uint32_t g_steam_alive_ms;

/*
 * HAPTICS SNIFFER: every haptic-shaped write from Steam, printed whole.
 *
 * For finding out whether the controller takes a command that sets pitch as
 * well as strength. One line per write:
 *
 *   hap <ms> s<slot> <path> id=0xNN n=<len> <fate>: <every byte>
 *
 * <path> is "out" for an output report, "cmd" for a frame on the feature
 * channel. <fate> is what the puck did with it. The RF relay carries at most
 * 16 bytes, so "relay-trunc16" means the controller got less than Steam sent.
 *
 * Built into one buffer and printed once: printk is synchronous here (see
 * CONFIG_LOG_PRINTK in prj.conf), and one write per byte would stall the USB
 * callback and interleave with the RF thread's output.
 */
static void haptic_dump(int slot, const char *path, uint8_t id,
			const uint8_t *d, uint16_t n, const char *fate)
{
	char line[256];
	int o;

	o = snprintk(line, sizeof(line), "hap %u s%d %s id=0x%02X n=%u %s:",
		     k_uptime_get_32(), slot, path, id, n, fate);
	for (uint16_t i = 0; i < n && o < (int)sizeof(line) - 4; i++) {
		o += snprintk(line + o, sizeof(line) - o, " %02X", d[i]);
	}
	printk("%s\n", line);
}

static const char *relay_fate(uint16_t n)
{
	return n > 16 ? "relay-trunc16" : "relay";
}

/*
 * Steam is dropping this bond (Forget), or a duplicate is being cleared: tell
 * Steam the controller is gone while the slot is STILL bonded, so the send
 * passes the g_bond_used gate in puck_hid_send_input(), and reset the
 * connection bookkeeping so a later re-pair starts from a clean edge.
 * Defined with the rest of the 0x79 state at the bottom of the file.
 */
static void slot_disconnect_and_reset(int slot);

/* Staged reply per slot, returned by the next GET on that interface. */
static uint8_t g_resp[PUCK_HID_SLOTS][63];

/* Settings shadow. Steam writes settings with 0x87 and reads them back to
 * verify they "took"; answering from a shadow is what stops its endless
 * config-verify retry (the 0x81/0x87 storm).
 */
static uint16_t g_setting[PUCK_HID_SLOTS][0x53];

/* Per-puck serials, derived from the chip so no two collide. */
static char g_unit[16];
static char g_board[16];

static void gen_serials(void)
{
	uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];

	snprintk(g_unit, sizeof(g_unit), "FXB99602%05X", (unsigned)(id & 0xFFFFF));
	snprintk(g_board, sizeof(g_board), "MXB99602%05X", (unsigned)(id & 0xFFFFF));
}

/*
 * Build the reply for one command. `rid` is the feature report id: 1 means the
 * question is about the bonded controller, 2 about the puck.
 */
static void handle_cmd(int slot, uint8_t rid, const uint8_t *buf, uint16_t len)
{
	uint8_t *resp = g_resp[slot];
	uint8_t cmd = buf[0];
	uint8_t pln = len > 1 ? buf[1] : 0;
	const uint8_t *pl = buf + 2;

	/*
	 * BOUND THE CLAIMED LENGTH BY WHAT ACTUALLY ARRIVED.
	 *
	 * pln comes from the report itself, so trusting it reads past the end of
	 * the transfer and forwards whatever follows to the controller as if it
	 * were part of the command.
	 */
	if (len < 2) {
		pln = 0;
	} else if (pln > len - 2) {
		pln = (uint8_t)(len - 2);
	}



	memset(resp, 0, 63);

	switch (cmd) {
	case CMD_ATTRIBUTES:
		resp[0] = CMD_ATTRIBUTES;
		resp[1] = sizeof(ATTR83);
		memcpy(resp + 2, ATTR83, sizeof(ATTR83));
		/* Report id 1 = the CONTROLLER, so flip the product id's low
		 * byte 0x04 -> 0x02. See the note above ATTR83.
		 */
		if (rid == 1) {
			resp[2 + 1] = 0x02;
		}
		break;

	case CMD_STRINGS: {
		uint8_t idx = pln > 0 ? pl[0] : 1;

		resp[0] = CMD_STRINGS;
		resp[1] = 0x14;
		resp[2] = idx;
		/*
		 * Steam matches a connected controller to its bond by serial:
		 * it reads the controller's serial here on rid 1 and compares
		 * it with the 16-byte serial inside the bond record returned by
		 * 0xA3. Answer with the puck's serial and nothing ever matches,
		 * the "paired to" list stays empty, and Steam re-runs its
		 * config storm on every connect.
		 *
		 * idx 0/1/4 = board/unit/alt serial; a real controller returns
		 * the SAME serial for 0 and 4.
		 */
		if (rid == 1 && g_bond_used[slot] &&
		    (idx == 0 || idx == 1 || idx == 4)) {
			memcpy(resp + 3, g_bond[slot] + 8, 16);
		} else {
			const char *s = (rid == 1)              ? "NA" :
					(idx == 0 || idx == 4)  ? g_board :
					(idx == 1)              ? g_unit : "NA";
			memcpy(resp + 3, s, strlen(s));
		}
		break;
	}

	case CMD_CONNECTED:
		/*
		 * 0x02 = a controller is live on this slot, 0x01 = not. This
		 * follows the RF link and not the bond: a bonded controller
		 * that is switched off is offline.
		 */
		resp[0] = CMD_CONNECTED;
		resp[1] = 0x01;
		resp[2] = rf_link_slot_live(slot) ? 0x02 : 0x01;
		break;

	case CMD_BOND_WRITE:
		/* THIS is how a controller gets paired: Steam does the pairing
		 * itself and writes the resulting 24-byte record here. An empty
		 * record clears the slot.
		 */
		if (pln >= BOND_LEN) {
			bool empty = true;

			for (int i = 0; i < BOND_LEN; i++) {
				if (pl[i]) {
					empty = false;
					break;
				}
			}

			if (empty) {
				slot_disconnect_and_reset(slot);
				g_bond_used[slot] = false;
				memset(g_bond[slot], 0, BOND_LEN);
				switch2_store_clear_bond(slot);
				printk("hid%d: slot CLEARED by Steam\n", slot);
				rf_link_bond_changed(slot);
			} else {
				memcpy(g_bond[slot], pl, BOND_LEN);
				g_bond_used[slot] = true;
				printk("hid%d: BONDED by Steam: uuid ", slot);
				for (int i = 0; i < 8; i++) {
					printk("%02x", g_bond[slot][i]);
				}
				printk(" serial %.16s\n", (char *)g_bond[slot] + 8);

				/*
				 * ONE CONTROLLER, ONE SLOT. Steam does not
				 * always re-pair into the slot it just cleared
				 * (observed moving 0 -> 1 -> 0 across
				 * re-pairs) and a controller left bonded in
				 * two slots would have BOTH polled (they derive
				 * the same session address from the same uuid)
				 * and BOTH forwarding its input on different
				 * interfaces.
				 */
				for (int o = 0; o < PUCK_HID_SLOTS; o++) {
					if (o == slot || !g_bond_used[o]) {
						continue;
					}
					if (!memcmp(g_bond[o] + 8,
						    g_bond[slot] + 8, 16)) {
						printk("hid%d: same controller was already bonded here: clearing\n",
						       o);
						slot_disconnect_and_reset(o);
						g_bond_used[o] = false;
						memset(g_bond[o], 0, BOND_LEN);
						switch2_store_clear_bond(o);
						rf_link_bond_changed(o);
					}
				}

				if (switch2_store_save_bond(slot, g_bond[slot])) {
					printk("hid%d: bond NOT persisted\n", slot);
				}

				/* Give the new bond its session address NOW.
				 * The RF layer derives addresses when it starts,
				 * so a bond Steam creates later would otherwise
				 * be beaconed on an all-zero address.
				 */
				rf_link_bond_changed(slot);
			}
		}
		resp[0] = CMD_BOND_WRITE;
		resp[1] = 0;
		break;

	case CMD_BOND_READ:
		resp[0] = CMD_BOND_READ;
		resp[1] = BOND_LEN;
		if (g_bond_used[slot]) {
			memcpy(resp + 2, g_bond[slot], BOND_LEN);
		}
		break;

	case CMD_PAIRING:
		/*
		 * Recorded but never acted on: the puck does no over-the-air
		 * discovery. Steam performs the pairing itself, over the
		 * CONTROLLER's own USB, and delivers the result via 0xA2.
		 */
		printk("hid%d: pairing mode %s\n", slot,
		       (pln > 0 && pl[0]) ? "ON" : "off");
		resp[0] = CMD_PAIRING;
		resp[1] = 0;
		break;

	case CMD_SETTINGS:
		/*
		 * SET_SETTINGS_VALUES: payload is N x [id][val16 LE]. Shadow
		 * each one so the 0x89 read-back matches what Steam wrote.
		 * That is what stops its endless config-verify retry. ACK with
		 * a clean [0x87][0] like the real dongle, not a payload echo.
		 */
		for (uint8_t i = 0; i + 2 < pln; i += 3) {
			uint8_t id = pl[i];

			if (id < 0x53) {
				g_setting[slot][id] = (uint16_t)pl[i + 1] |
						      ((uint16_t)pl[i + 2] << 8);
			}
		}
		resp[0] = CMD_SETTINGS;
		resp[1] = 0;
		break;

	case CMD_GET_SETTING: {
		/* Answer from the shadow, in the same [id][val16 LE] form. */
		uint8_t n = 0;

		resp[0] = CMD_GET_SETTING;
		for (uint8_t i = 0; i < pln && n + 3 < 60; i++) {
			uint8_t id = pl[i];
			uint16_t v = (id < 0x53) ? g_setting[slot][id] : 0;

			resp[2 + n++] = id;
			resp[2 + n++] = (uint8_t)v;
			resp[2 + n++] = (uint8_t)(v >> 8);
		}
		resp[1] = n;
		break;
	}

	default:
		/*
		 * STEAM'S OWN HAPTICS, FORWARDED VERBATIM.
		 *
		 * Steam speaks the controller's language directly. It does
		 * not ask the puck to rumble, it sends the controller's own
		 * commands and expects the dongle to pass them on.
		 *
		 * 0x05 is the rumble command on this controller's firmware,
		 * found by watching Steam's vibration test: ten 12-byte 0x05
		 * writes, one per press, and nothing else that correlated. It
		 * sits below the 0x80-0x88 actuator range, so a relay that
		 * starts at 0x80 drops the one command that matters.
		 *
		 * Deliberately not relayed: everything the puck answers itself,
		 * meaning identity, bonds and settings reads. Their replies
		 * cannot come back over a no-ack link, and 0x83 executes on the
		 * controller, which comes out as a click every time Steam
		 * re-polls identity.
		 */
		if (cmd >= 0x80 && cmd <= 0x88) {
			haptic_dump(slot, "cmd", cmd, pl, pln, relay_fate(pln));
			rf_link_relay(slot, cmd, pl, pln, 1);
		} else {
			/* Bytes too: the original Steam Controller's haptic
			 * pulse was 0x8F, outside the relayed range.
			 */
			haptic_dump(slot, "cmd", cmd, pl, pln, "unhandled");
		}
		resp[1] = 0;
		break;
	}
}

/* Which bond slot this interface is. Interface N owns slot N. */
static const struct device *hid_dev[PUCK_HID_SLOTS];

static int slot_of(const struct device *dev)
{
	for (int i = 0; i < PUCK_HID_SLOTS; i++) {
		if (hid_dev[i] == dev) {
			return i;
		}
	}
	return -1;
}

/*
 * Steam reading a reply.
 */
static int puck_get_report(const struct device *dev, const uint8_t type,
			   const uint8_t id, const uint16_t len, uint8_t *const buf)
{
	int slot = slot_of(dev);
	uint16_t n;

	ARG_UNUSED(type);

	if (slot < 0 || len == 0) {
		return -ENODEV;
	}

	/*
	 * Hand back whatever the matching SET staged: Steam writes a command,
	 * then reads the answer from the same interface.
	 *
	 * Two things Zephyr does differently from TinyUSB, both silently.
	 *
	 * 1. It does not prepend the report id. TinyUSB force-writes the id as
	 *    wire byte 0 and hands the app the buffer past it. Here byte 0 is
	 *    this code's, so the wire format [id][cmd][len][payload] has to be
	 *    assembled in full.
	 *
	 * 2. The return value is the reply length and it must be positive.
	 *    puck_get_report() only calls net_buf_add() when ret > 0; anything
	 *    else sets errno and stalls the control transfer. Returning 0, the
	 *    obvious "success", answers every one of Steam's reads with a
	 *    stall, and that is what "Pairing Failed (16)" was.
	 */
	buf[0] = id;
	n = MIN(len - 1, sizeof(g_resp[slot]));
	memcpy(buf + 1, g_resp[slot], n);

	return n + 1;
}

/*
 * Steam writing a command. This is where pairing commands arrive.
 *
 * Decoded against the Arduino firmware in handle_cmd(). 0xA2 is the one that
 * matters: Steam does the pairing itself and writes the bond record here.
 */
static int puck_set_report(const struct device *dev, const uint8_t type,
			   const uint8_t id, const uint16_t len,
			   const uint8_t *const buf)
{
	int slot = slot_of(dev);

	if (slot < 0 || len < 2) {
		return -ENODEV;
	}

	/* Any write from Steam counts as it being alive and reacting, which
	 * is what stops the connected-state resend below.
	 */
	g_steam_alive_ms = k_uptime_get_32();

	/*
	 * STEAM'S HAPTICS ARE OUTPUT REPORTS 0x80-0x86, AND THEY ARE NOT
	 * COMMAND FRAMES.
	 *
	 * The report ID is the command and the bytes after it are its payload,
	 * so they go to the controller verbatim. Steam's vibration test rides
	 * 0x85, ping and grip use 0x85 and 0x86, ordinary rumble uses 0x82.
	 *
	 * Running these through handle_cmd(), which reads [cmd][len][payload],
	 * reads a 4-byte write to report 0x85 as "command 0x05, one byte" and
	 * hands the controller that instead of a haptic.
	 *
	 * 0x81 is dropped deliberately. It is Steam's haptic reset, it re-arms
	 * the controller's amplifier every time, and it is where the click on
	 * connect comes from.
	 */
	if (type == HID_REPORT_TYPE_OUTPUT && id >= 0x80 && id <= 0x86) {
		if (id != 0x81) {
			haptic_dump(slot, "out", id, buf + 1, len - 1,
				    relay_fate(len - 1));
			rf_link_relay(slot, id, buf + 1, (uint8_t)(len - 1), 1);
		} else {
			haptic_dump(slot, "out", id, buf + 1, len - 1, "dropped");
		}
		return 0;
	}

	/* The 63-byte output reports 0x87-0x89 are parsed as command frames
	 * below. Print them raw first, so a long haptic command riding one of
	 * them is seen as Steam sent it rather than as the parser read it.
	 */
	if (type == HID_REPORT_TYPE_OUTPUT) {
		haptic_dump(slot, "out", id, buf + 1, len - 1, "as-cmd");
	}

	/* Everything else is a command frame: buf[0] echoes the report id, then
	 * [cmd][len][payload].
	 */
	handle_cmd(slot, id, buf + 1, len - 1);
	return 0;
}

static void puck_set_protocol(const struct device *dev, const uint8_t proto)
{
	printk("hid%d protocol=%u\n", slot_of(dev), proto);
}

/*
 * OUTGOING REPORTS. Queued, never submitted straight from the caller.
 *
 * hid_device_submit_report() is synchronous unless the ops table below provides
 * input_report_done(). Without that callback it ends in
 *
 *     k_sem_take(&ddata->in_sem, K_FOREVER);     (subsys/usb/.../usbd_hid.c)
 *
 * so one report on an interface the host is not draining blocks the calling
 * thread forever. Blocking the RF thread there stops the puck answering Steam's
 * 0xA3 slot polls, and Steam responds by deleting the registration rows one at
 * a time.
 *
 * Registering input_report_done makes submit asynchronous, but the class wraps
 * the caller's buffer by reference (net_buf_alloc_with_data) rather than
 * copying it, so a report has to stay put until the callback says it went out.
 * The stack buffer the blocking version could safely use becomes a
 * use-after-free the moment submit stops blocking. Hence the per-slot queue
 * below, in UDC-aligned storage.
 *
 * The depth is needed. The endpoint stays busy about a millisecond after each
 * send while the controller produces 300+ reports/s, so dropping whenever it is
 * busy loses roughly a third of them. Queue instead, and when the queue is full
 * too, drop the oldest rather than the newest.
 */
#define PUCK_HID_REPORT_MAX 64
#define TXQ_DEPTH	    4
#define TX_ENTRY_SZ	    UDC_ROUND_UP(PUCK_HID_REPORT_MAX)

/*
 * TXQ_DEPTH pending entries per slot, plus one holding the report currently
 * handed to the stack. That one has to outlive the submit call.
 */
UDC_STATIC_BUF_DEFINE(g_txbuf, PUCK_HID_SLOTS *(TXQ_DEPTH + 1) * TX_ENTRY_SZ);

#define TX_ENT(s, k)   (&g_txbuf[((s) * (TXQ_DEPTH + 1) + (k)) * TX_ENTRY_SZ])
#define TX_INFLIGHT(s) TX_ENT(s, TXQ_DEPTH)

static uint16_t g_txq_len[PUCK_HID_SLOTS][TXQ_DEPTH];
static uint16_t g_tx_inflight_len[PUCK_HID_SLOTS];
static uint8_t g_txq_head[PUCK_HID_SLOTS];
static uint8_t g_txq_tail[PUCK_HID_SLOTS];
static uint8_t g_txq_cnt[PUCK_HID_SLOTS];
static bool g_tx_busy[PUCK_HID_SLOTS];
static uint32_t g_tx_dropped[PUCK_HID_SLOTS];
static struct k_spinlock g_tx_lock;

/*
 * Hand the stack the in-flight buffer. NEVER call this holding g_tx_lock: it
 * walks into the USB driver, and input_report_done() can land before it
 * returns.
 */
static void tx_submit(int slot)
{
	k_spinlock_key_t key;
	int ret;

	ret = hid_device_submit_report(hid_dev[slot], g_tx_inflight_len[slot],
				       TX_INFLIGHT(slot));
	if (ret == 0) {
		return;
	}

	/*
	 * -EACCES just means the host has not enabled this interface yet, which
	 * is ordinary during enumeration, and is the case the old code threw
	 * away by ignoring this return value entirely. Whatever the reason, the
	 * slot must be released or it stays busy forever and the queue never
	 * drains again.
	 */
	key = k_spin_lock(&g_tx_lock);
	g_tx_busy[slot] = false;
	k_spin_unlock(&g_tx_lock, key);
}

/*
 * The stack has finished with the in-flight buffer: promote the next queued
 * report into it and keep the pipe moving.
 */
static void puck_input_done(const struct device *dev,
			    const uint8_t *const report)
{
	k_spinlock_key_t key;
	int slot = slot_of(dev);
	bool more = false;

	ARG_UNUSED(report);

	if (slot < 0) {
		return;
	}

	key = k_spin_lock(&g_tx_lock);
	if (g_txq_cnt[slot] > 0) {
		uint8_t t = g_txq_tail[slot];

		memcpy(TX_INFLIGHT(slot), TX_ENT(slot, t), g_txq_len[slot][t]);
		g_tx_inflight_len[slot] = g_txq_len[slot][t];
		g_txq_tail[slot] = (t + 1) % TXQ_DEPTH;
		g_txq_cnt[slot]--;
		more = true;
	} else {
		g_tx_busy[slot] = false;
	}
	k_spin_unlock(&g_tx_lock, key);

	if (more) {
		tx_submit(slot);
	}
}

static const struct hid_device_ops puck_hid_ops = {
	.get_report = puck_get_report,
	.set_report = puck_set_report,
	.set_protocol = puck_set_protocol,
	.input_report_done = puck_input_done,
};

/*
 * Reload the bonds Steam wrote on a previous boot.
 *
 * Separate from registering the dongle's HID interfaces, because both USB
 * modes need it. The RF layer derives each slot's session address from its
 * bond, so a puck with no bonds loaded never beacons and never picks up a
 * controller: in Pro Controller mode, a pad that enumerates perfectly and then
 * sits there with every button dead.
 *
 * Steam pairs once, over the controller's own USB, and never repeats it. A
 * record lost to a power cut is a controller that silently stops working with
 * no way to tell why.
 */
void puck_hid_restore_bonds(void)
{
	gen_serials();

	for (int i = 0; i < PUCK_HID_SLOTS; i++) {
		if (switch2_store_load_bond(i, g_bond[i])) {
			g_bond_used[i] = true;
			printk("hid%d: bond restored: serial %.16s\n", i,
			       (char *)g_bond[i] + 8);
		}
	}
}

int puck_hid_register_all(void)
{
	/* Bonds must be in place before the first 0xA3 poll can answer "empty"
	 * and let Steam hand the slot to someone else.
	 */
	puck_hid_restore_bonds();

	hid_dev[0] = DEVICE_DT_GET(DT_NODELABEL(hid_slot_0));
	hid_dev[1] = DEVICE_DT_GET(DT_NODELABEL(hid_slot_1));
	hid_dev[2] = DEVICE_DT_GET(DT_NODELABEL(hid_slot_2));
	hid_dev[3] = DEVICE_DT_GET(DT_NODELABEL(hid_slot_3));

	for (int i = 0; i < PUCK_HID_SLOTS; i++) {
		if (!device_is_ready(hid_dev[i])) {
			printk("hid%d: device not ready\n", i);
			continue;
		}
		/*
		 * MUST run before usbd_init(). Zephyr's HID class contributes
		 * no interface descriptor until the report descriptor is
		 * registered, and usbd_init() then rejects the WHOLE device
		 * with -EINVAL, which takes the CDC console down with it and
		 * leaves the board enumerating as "Unknown USB Device (Port
		 * Reset Failed)", recoverable only by a physical double-tap.
		 */
		hid_device_register(hid_dev[i], PUCK_HID_DESC,
				    sizeof(PUCK_HID_DESC), &puck_hid_ops);
	}

	printk("hid: %d interfaces registered (%u-byte report descriptor)\n",
	       PUCK_HID_SLOTS, (unsigned)sizeof(PUCK_HID_DESC));
	return 0;
}

const uint8_t *puck_hid_bond(int slot)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS || !g_bond_used[slot]) {
		return NULL;
	}
	return g_bond[slot];
}

int puck_hid_send_input(int slot, uint8_t rid, const uint8_t *data, uint16_t len)
{
	k_spinlock_key_t key;
	bool kick = false;
	uint8_t *dst;

	if (slot < 0 || slot >= PUCK_HID_SLOTS || !g_bond_used[slot]) {
		return -EINVAL;
	}

	/*
	 * NO INTERFACE IN PRO CONTROLLER MODE. hid_dev[] is filled in by
	 * puck_hid_register_all(), which only runs when the puck comes up as
	 * the Valve dongle, while bonds are restored in both modes. So
	 * g_bond_used[] is set and the gate above passes happily, and
	 * submitting on a NULL device takes the firmware down on the first RF
	 * poll: a silent board, a stale enumeration, and then "Unknown USB
	 * Device (Device Descriptor Request Failed)".
	 *
	 * It appears only in Pro Controller mode and only once RF starts, which
	 * looks exactly like USB/RF contention and is not.
	 */
	if (!hid_dev[slot]) {
		return -ENODEV;
	}
	if (len + 1 > PUCK_HID_REPORT_MAX) {
		len = PUCK_HID_REPORT_MAX - 1;
	}

	key = k_spin_lock(&g_tx_lock);

	if (!g_tx_busy[slot]) {
		dst = TX_INFLIGHT(slot);
		g_tx_inflight_len[slot] = len + 1;
		g_tx_busy[slot] = true;
		kick = true;
	} else {
		if (g_txq_cnt[slot] == TXQ_DEPTH) {
			/* Queue full: drop the OLDEST pending report rather
			 * than this one, so the newest sample survives. See
			 * the note above.
			 */
			g_txq_tail[slot] = (g_txq_tail[slot] + 1) % TXQ_DEPTH;
			g_txq_cnt[slot]--;
			g_tx_dropped[slot]++;
		}
		dst = TX_ENT(slot, g_txq_head[slot]);
		g_txq_len[slot][g_txq_head[slot]] = len + 1;
		g_txq_head[slot] = (g_txq_head[slot] + 1) % TXQ_DEPTH;
		g_txq_cnt[slot]++;
	}

	/*
	 * The report id leads, then the controller's body verbatim. Do not pad
	 * to the descriptor's declared length: that breaks battery reporting in
	 * both Steam and lizard mode. A real puck forwards exactly what the
	 * controller sent.
	 *
	 * THE COPY STAYS INSIDE THE LOCK. Moving it out looks safe, since the
	 * destination is already reserved, and it is not. puck_input_done()
	 * runs from USB completion, takes this same lock, and copies the oldest
	 * queue entry straight into the in-flight buffer. Fill an entry outside
	 * the lock and a completion landing in between ships whatever is in it,
	 * so a report goes out half old and half new: rare, load dependent, and
	 * near impossible to trace back to here.
	 */
	dst[0] = rid;
	memcpy(dst + 1, data, len);

	k_spin_unlock(&g_tx_lock, key);

	if (kick) {
		tx_submit(slot);
	}

	return 0;
}

/*
 * CONNECTION STATE PUSHED TO STEAM. Report 0x79, one byte: 0x02 connected,
 * 0x01 disconnected.
 *
 * Steam does not work out that a controller went away on its own. Without this
 * push it keeps showing one that has been switched off, and a forget-and-
 * re-pair then produces two entries: the stale one Steam still believes in
 * plus the new one.
 *
 * Edge-triggered, then resent, because neither alone works:
 *
 *   - On the edge only, a single dropped report strands Steam in the wrong
 *     state indefinitely, listing a controller as connected with no input
 *     flowing.
 *   - Repeating forever re-triggers Steam's connect handling, which re-runs
 *     its config and replays the connect chime every interval.
 *
 * So connected is resent until Steam reacts, any write from it counting as an
 * ack, and disconnected only for a bounded window.
 */
#define CONN_WINDOW_MS   300u	/* replies newer than this = connected */
#define CONN_RESEND_MS   750u
#define DISC_RESEND_MS  6000u	/* bounded, so disc never spams forever */
#define POWEROFF_HOLD_MS 2000u	/* covers the controller's post-off F1 tail */

static uint32_t g_poweroff_ms[PUCK_HID_SLOTS];
static bool g_usb_conn[PUCK_HID_SLOTS];	/* what Steam currently believes */
static uint32_t g_last79[PUCK_HID_SLOTS];
static uint32_t g_conn_edge[PUCK_HID_SLOTS];
static uint32_t g_disc_edge[PUCK_HID_SLOTS];
static uint32_t g_po_handled[PUCK_HID_SLOTS];

void puck_hid_note_power_off(int slot)
{
	if (slot >= 0 && slot < PUCK_HID_SLOTS) {
		g_poweroff_ms[slot] = k_uptime_get_32();
	}
}

static bool slot_powering_off(int slot)
{
	return g_poweroff_ms[slot] &&
	       (k_uptime_get_32() - g_poweroff_ms[slot]) < POWEROFF_HOLD_MS;
}

static void send_conn_state(int slot, bool conn)
{
	uint8_t st = conn ? 0x02 : 0x01;

	g_last79[slot] = k_uptime_get_32();
	puck_hid_send_input(slot, 0x79, &st, 1);
}

/*
 * Steam is dropping this bond, so it has to be told the controller went away.
 * It does not work that out for itself, and the stale entry it keeps is what
 * makes a later re-pair show up as a second controller.
 *
 * This runs while the slot is still bonded, because puck_hid_send_input()
 * refuses to send on an unbonded one. Do not lift that gate to make the send
 * work: an unbonded slot can have no interface behind it, and submitting there
 * blocks the caller forever. See the tx queue note above.
 *
 * Sent once rather than resent: the caller clears the bond immediately after,
 * and puck_hid_conn_task() then skips this slot, so there is no resend window
 * to anchor. The queue is what makes the single send reliable.
 */
static void slot_disconnect_and_reset(int slot)
{
	if (g_usb_conn[slot]) {
		printk("hid%d: bond cleared: telling Steam disconnected\n",
		       slot);
		send_conn_state(slot, false);
	}

	/*
	 * Reset the edge bookkeeping too, or a re-pair into this slot inherits
	 * "Steam already thinks it is connected" and the connected edge never
	 * fires again: a controller that works but never appears.
	 */
	g_usb_conn[slot] = false;
	g_conn_edge[slot] = 0;
	g_disc_edge[slot] = 0;
	g_last79[slot] = 0;
	g_poweroff_ms[slot] = 0;
	g_po_handled[slot] = 0;
}

void puck_hid_conn_task(void)
{
	uint32_t now = k_uptime_get_32();

	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		bool conn, steam_acked, disc_resend;

		if (!g_bond_used[s]) {
			continue;
		}

		/*
		 * A relayed power-off (the Steam+Y chord) forces ONE clean
		 * disconnect regardless of the prior edge. The controller keeps
		 * streaming input for about a second after being told to shut
		 * down, which desyncs the tracked state from Steam's so the
		 * ordinary edge never fires: the "controller never gets
		 * removed" case.
		 */
		if (g_poweroff_ms[s] && g_poweroff_ms[s] != g_po_handled[s]) {
			g_po_handled[s] = g_poweroff_ms[s];
			printk("hid%d: power-off: telling Steam disconnected\n", s);
			send_conn_state(s, false);
			g_usb_conn[s] = false;
			g_disc_edge[s] = now;
			continue;
		}

		/* Held disconnected through the post-power-off tail, or a stray
		 * dying reply bounces it back to connected and Steam reads that
		 * as a reconnect.
		 */
		conn = !slot_powering_off(s) && rf_link_slot_conn(s);

		steam_acked = g_steam_alive_ms &&
			      (int32_t)(g_steam_alive_ms - g_conn_edge[s]) >= 0;
		disc_resend = !conn && (now - g_disc_edge[s]) < DISC_RESEND_MS;

		if (conn != g_usb_conn[s] ||
		    (conn && !steam_acked && (now - g_last79[s]) >= CONN_RESEND_MS) ||
		    (disc_resend && (now - g_last79[s]) >= CONN_RESEND_MS)) {
			if (conn && !g_usb_conn[s]) {
				g_conn_edge[s] = now;
				printk("hid%d: controller CONNECTED -> Steam\n", s);
			}
			if (!conn && g_usb_conn[s]) {
				g_disc_edge[s] = now;
				printk("hid%d: controller DISCONNECTED -> Steam\n", s);
			}
			g_usb_conn[s] = conn;
			send_conn_state(s, conn);
		}
	}
}

/*
 * Is the host actually draining the IN endpoints?
 *
 * On a console key of its own: a slot stuck "busy" with a rising queue is the
 * signature of an interface nobody is reading.
 */
void puck_hid_dump_tx(void)
{
	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		printk("hid%d: tx %-4s queued=%u dropped=%u%s\n", s,
		       g_tx_busy[s] ? "busy" : "idle", g_txq_cnt[s],
		       g_tx_dropped[s], g_bond_used[s] ? "" : "  (no bond)");
	}
}
