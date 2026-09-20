/*
 * Pair-Raught: the RF link to the Steam Controller.
 *
 * Ported from the Arduino build, kept in reference/. The radio primitives come
 * from rf_radio.c, timing from rf_micros(), and the bond records from the USB
 * dongle layer instead of LittleFS.
 *
 * THE FRAME. Decoded from a real puck and CRC-validated. The radio is set up
 * for ESB with dynamic payload length, so the buffer is
 *
 *     [LENGTH][S1 = PID<<1][payload ...]
 *
 * and the hardware appends CRC16 (0x11021) itself. The E1 beacon's payload is
 * 18 bytes, which the controller checks by comparing buf[0] against 0x12:
 *
 *     [0]      0xE1                  marker
 *     [1..5]   proteus uuid (LE)     from the bond record
 *     [5..9]   ibex uuid (LE)        from the bond record
 *     [9]      session channel       the controller moves here
 *     [10..13] zero
 *     [13..17] session base address  this slot's unique address
 *     [17]     session prefix
 *
 * WHY THE BEACON IS NEEDED AT ALL. A real puck sends no E1 on its session
 * channel, and a bonded controller already knows the per-bond address and just
 * resumes. This one still sends it, because the beacon is how the controller
 * learns THIS puck's session base, prefix and channel. Discovery on channel 2
 * is separate and always runs.
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <hal/nrf_ficr.h>

#include "rf_link.h"
#include "rf_radio.h"
#include "puck_usb.h"
#include "puck_hid.h"
#include "puck_input.h"
#include "switch2_gatt.h"
#include "switch_pro_usb.h"
#include "puck_trace.h"
#include "switch2_store.h"
#include "puck_settings.h"
#include "puck_melody.h"
#include <zephyr/sys/reboot.h>
#include "puck_usb.h"

/* The shared discovery rendezvous: base "ibex", prefix 0x10, channel 2. A
 * controller that has just powered on listens here, so it is the only address
 * on which a puck it has never heard from can reach it.
 */
static const uint8_t RF_PAIR_BASE[4] = { 0x69, 0x62, 0x65, 0x78 };

/* Session channel. The beacon advertises it and the controller adopts it. */
#define RF_SESS_CH 18

/* Poll reply window, microseconds. Fixed, not tunable. See rf_poll_slot(). */
#define RF_RX_WIN_US 1200u

/* How often to probe a slot that is not answering. See rf_link_task(). */
#define RF_DARK_PROBE_MS 100u

static uint8_t g_sess_base[PUCK_HID_SLOTS][4];
static uint8_t g_sess_prefix[PUCK_HID_SLOTS];
static uint8_t g_pid;
static bool g_active;

static uint8_t rftx[RF_BUF_LEN];
static uint8_t rfrx[RF_BUF_LEN];

static uint32_t g_beacons;
static uint32_t g_crc_bad;
static uint32_t g_polls;
static uint32_t g_f1[PUCK_HID_SLOTS];
static uint32_t g_link_ms[PUCK_HID_SLOTS];
static uint8_t g_poll_pid[PUCK_HID_SLOTS];
static uint8_t g_last_seq[PUCK_HID_SLOTS];
static uint32_t g_fresh[PUCK_HID_SLOTS];
static uint32_t g_probe_ms[PUCK_HID_SLOTS];
static uint32_t g_beacon_ms[PUCK_HID_SLOTS];
static uint32_t g_chord_ms[PUCK_HID_SLOTS];
static bool g_chord_fired[PUCK_HID_SLOTS];
static uint32_t g_wake_chord_ms[PUCK_HID_SLOTS];
static bool g_wake_chord_fired[PUCK_HID_SLOTS];
static uint32_t g_mode_chord_ms[PUCK_HID_SLOTS];
static uint32_t g_reg_chord_ms[PUCK_HID_SLOTS];
static bool g_mode_chord_fired[PUCK_HID_SLOTS];
static bool g_reg_chord_fired[PUCK_HID_SLOTS];
static uint32_t g_replies;

/*
 * Derive this slot's unique session address from the bond's ibex uuid.
 *
 * Every bonded controller needs its OWN address. That is what keeps four
 * controllers from hearing each other's traffic, since the channel is shared.
 * Mixing in the puck's own FICR device id keeps two pucks holding the same
 * cloned bond from landing on one address.
 */
static void derive_session_addr(int slot, const uint8_t *uuid)
{
	uint32_t h, h2;
	bool live = false;

	for (int i = 0; i < 8; i++) {
		if (uuid[i]) {
			live = true;
			break;
		}
	}

	if (live) {
		h = (uint32_t)uuid[0] * 0x9E3779B1u ^
		    (uint32_t)uuid[2] * 0x85EBCA6Bu ^
		    (uint32_t)uuid[4] * 0xC2B2AE35u ^ (uint32_t)uuid[6] ^
		    nrf_ficr_deviceid_get(NRF_FICR, 0);
		h2 = (uint32_t)uuid[1] * 0x27D4EB2Fu ^
		     (uint32_t)uuid[3] * 0x165667B1u ^ (uint32_t)uuid[5] ^
		     (uint32_t)uuid[7] * 0x9E3779B1u ^
		     nrf_ficr_deviceid_get(NRF_FICR, 1);
	} else {
		h = nrf_ficr_deviceid_get(NRF_FICR, 0) * 0x9E3779B1u ^
		    nrf_ficr_deviceid_get(NRF_FICR, 1);
		h2 = nrf_ficr_deviceid_get(NRF_FICR, 1) * 0x85EBCA6Bu ^
		     nrf_ficr_deviceid_get(NRF_FICR, 0);
	}

	/* Scrub 0x00/0xFF out of the address: they correlate badly against the
	 * preamble, which degrades sync reliability.
	 */
	for (int i = 0; i < 4; i++) {
		uint8_t b = (uint8_t)(h >> (i * 8));

		if (b == 0x00 || b == 0xFF) {
			b ^= 0x5A;
		}
		g_sess_base[slot][i] = b;
	}

	g_sess_prefix[slot] = (uint8_t)(h2 >> 16);
	if (g_sess_prefix[slot] == 0x00 || g_sess_prefix[slot] == 0xFF) {
		g_sess_prefix[slot] = 0x5C;
	}

	/* Never land on the shared discovery base. That would put this
	 * session back on the rendezvous every other controller is scanning,
	 * and the per-slot isolation would be gone.
	 */
	if (!memcmp(g_sess_base[slot], RF_PAIR_BASE, 4)) {
		g_sess_base[slot][0] ^= 0x80;
	}
}

void rf_link_init(void)
{
	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		const uint8_t *rec = puck_hid_bond(s);

		if (!rec) {
			continue;
		}

		derive_session_addr(s, rec + 4);
		printk("rf: slot %d session addr %02x%02x%02x%02x/%02x ch%u\n", s,
		       g_sess_base[s][0], g_sess_base[s][1], g_sess_base[s][2],
		       g_sess_base[s][3], g_sess_prefix[s], RF_SESS_CH);
	}
}

bool rf_link_beacon(int slot, bool discovery)
{
	const uint8_t *rec = puck_hid_bond(slot);
	const uint8_t *tx_base;
	uint8_t tx_prefix;
	uint32_t t0;
	bool got = false;
	uint8_t rcopy[24];
	uint8_t rlen = 0, rn = 0;
	bool rcrc = false;

	if (!rec) {
		return false;
	}
	/*
	 * Never transmit on an underived address. A bond appearing while RF is
	 * already running can reach this point before its session address is
	 * built, and base 00000000 prefix 00 goes out; the controller adopts
	 * that quite happily and then stutters. Derive late rather than beacon
	 * garbage.
	 */
	if (!g_sess_base[slot][0] && !g_sess_base[slot][1] &&
	    !g_sess_base[slot][2] && !g_sess_base[slot][3]) {
		rf_link_bond_changed(slot);
	}


	memset(rftx, 0, sizeof(rftx));
	rftx[0] = 0x12;					/* LENGTH = 18 */
	rftx[1] = (uint8_t)((g_pid++ & 3) << 1);	/* S1 = PID<<1, noack */
	rftx[2] = 0xE1;					/* payload[0] marker */
	memcpy(rftx + 3, rec + 0, 4);			/* proteus uuid */
	memcpy(rftx + 7, rec + 4, 4);			/* ibex uuid */
	rftx[11] = RF_SESS_CH;				/* payload[9] */
	memcpy(rftx + 15, g_sess_base[slot], 4);	/* payload[13..17] */
	rftx[19] = g_sess_prefix[slot];			/* payload[17] */

	tx_base = discovery ? RF_PAIR_BASE : g_sess_base[slot];
	tx_prefix = discovery ? RF_PAIR_PREFIX : g_sess_prefix[slot];

	rf_config(discovery ? RF_PAIR_CH : RF_SESS_CH);
	rf_set_addr(tx_base, tx_prefix);

	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	rf_wait_disabled();
	NRF_RADIO->EVENTS_DISABLED = 0;
	g_beacons++;

	/* A session keepalive expects nothing back (the controller
	 * answers E3 polls, not beacons) and the radio is already disabled by the
	 * END_DISABLE short, so there is no window to pay for.
	 */
	if (!discovery) {
		return false;
	}

	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;

	t0 = rf_micros();
	while (!NRF_RADIO->EVENTS_END && (rf_micros() - t0) < 800u) {
	}

	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;
		g_replies++;
		got = true;

		/* Capture only. The dump runs after the radio is down; see the
		 * logging note at the bottom of rf_radio.c.
		 */
		rlen = rfrx[0];
		rcrc = NRF_RADIO->CRCSTATUS & 1;

		/* rlen is the LENGTH byte off the air, captured before the CRC
		 * is checked, so it is noise until proven otherwise. Clamp
		 * against the destination rather than against 24: they are the
		 * same number today and would not stay that way.
		 */
		rn = (uint8_t)MIN((unsigned)rlen + 2u, sizeof(rcopy));
		memcpy(rcopy, rfrx, rn);
	}

	NRF_RADIO->TASKS_DISABLE = 1;
	rf_wait_disabled();
	NRF_RADIO->EVENTS_DISABLED = 0;

	if (got) {
		printk("rf: REPLY#%u slot%d crc%d len%u:", (unsigned)g_replies,
		       slot, rcrc, rlen);
		for (uint8_t i = 0; i < rn; i++) {
			printk(" %02x", rcopy[i]);
		}
		printk("\n");
	}
	return got;
}




/*
 * One pending relay per slot, sent inside the next poll.
 *
 * The relay is NO-ACK on the wire, so a command is sent as a short burst: a
 * single lost frame must not silently fail to power the controller off.
 */
#define RELAY_SHOTS 3

static uint8_t g_relay_rid[PUCK_HID_SLOTS];
static uint8_t g_relay_data[PUCK_HID_SLOTS][16];	/* 9 for rumble, 4 for off */
static uint8_t g_relay_len[PUCK_HID_SLOTS];
static uint8_t g_relay_shots[PUCK_HID_SLOTS];
static struct k_spinlock g_relay_lock;

/*
 * Queue a command for the controller, sent inside the next poll(s).
 *
 * The relay is NO-ACK on the wire, so `shots` sends the same command on that
 * many successive polls. One is right for anything self-correcting (a rumble
 * level, followed immediately by more rumble); a burst is right for anything
 * whose loss is permanent, like a stop or a power-off.
 */
void rf_link_relay(int slot, uint8_t rid, const uint8_t *data, uint8_t len,
		   uint8_t shots)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS) {
		return;
	}
	if (len > sizeof(g_relay_data[0])) {
		len = sizeof(g_relay_data[0]);
	}

	/*
	 * Locked against build_poll(). Relays arrive from the USB stack, the
	 * console, and the work queue that plays the wake chime, which outranks
	 * this thread and can preempt it mid-copy. A poll built from half an
	 * update would send one command's id with another's bytes.
	 */
	k_spinlock_key_t key = k_spin_lock(&g_relay_lock);

	g_relay_rid[slot] = rid;
	memcpy(g_relay_data[slot], data, len);
	g_relay_len[slot] = len;
	g_relay_shots[slot] = shots;

	k_spin_unlock(&g_relay_lock, key);
}

static void rf_queue_power_off(int slot)
{
	static const uint8_t OFF[4] = { 0x6f, 0x66, 0x66, 0x21 };  /* "off!" */

	rf_link_relay(slot, 0x9F, OFF, sizeof(OFF), RELAY_SHOTS);
}

/*
 * Relay a rumble level to the controller.
 *
 * The Steam Controller's output report 0x80 is
 *   [type][intensity16][left_speed16][gain][right_speed16][gain]
 * so the console's two motors map onto left/right speeds, with the larger as
 * overall intensity. Type 0x04 is rumble; 0 is the off report.
 *
 * A STOP is relayed as a BURST, an ON as a single frame. The relay is NO-ACK:
 * a dropped ON corrects itself microseconds later because the console streams
 * rumble continuously during play, but a dropped final STOP leaves the
 * controller latched buzzing with nothing following to correct it. The
 * Arduino build saw exactly that, "constant rumble that didn't stop", clearable
 * only by a replug.
 */
void rf_link_rumble(int slot, uint16_t low, uint16_t high)
{
	uint8_t p[9];
	bool on;

	if (slot < 0 || slot >= PUCK_HID_SLOTS) {
		return;
	}

	/*
	 * Rumble disabled: force silence rather than returning, so a disable
	 * that lands mid-buzz still sends the STOP. Skipping the frame outright
	 * would leave the controller latched buzzing with nothing to clear it.
	 */
	if (!g_cfg.rumble_enabled) {
		low = 0;
		high = 0;
	}

	/*
	 * Scaled by the configured percentage; the Arduino build shipped 200,
	 * i.e. the decoded amplitude doubled. Clamped to 16 bits.
	 */
	low = (uint16_t)MIN((uint32_t)low * g_cfg.rumble_scale_pct / 100u, 0xFFFFu);
	high = (uint16_t)MIN((uint32_t)high * g_cfg.rumble_scale_pct / 100u, 0xFFFFu);
	on = low || high;

	p[0] = on ? 0x04 : 0x00;
	p[1] = (uint8_t)((low > high ? low : high) & 0xFF);
	p[2] = (uint8_t)((low > high ? low : high) >> 8);
	p[3] = (uint8_t)(low & 0xFF);
	p[4] = (uint8_t)(low >> 8);
	p[5] = 0;
	p[6] = (uint8_t)(high & 0xFF);
	p[7] = (uint8_t)(high >> 8);
	p[8] = 0;

	rf_link_relay(slot, 0x80, p, sizeof(p), on ? 1 : RELAY_SHOTS);
}

/*
 * Build the poll payload, carrying a pending relay if there is one.
 *
 * TWO FRAMINGS, AND CHOOSING THE WRONG ONE IS SILENT. Confirmed from real
 * puck to controller sniffs:
 *
 *   type-01   E3 [2+len][01][rid][len][data...]   the command lands (executes)
 *   legacy    E3 [1+len][05][rid][data...]        the ordinary relay form
 *
 * The legacy form makes the controller discard any command 0x87 or above, so
 * power-off (0x9F) has to use type-01. But type-01 is a whitelist rather than a
 * default: only 0x9F and a few specific 0x87 registers go out that way, and
 * everything else, all haptics included (0x82 rumble, 0x85/0x86 ping and grip),
 * goes out legacy.
 *
 * Sending haptics as type-01 is silent. The controller does not act on them and
 * nothing reports an error, which is what a rumble that never worked looks like
 * from this side.
 *
 * With nothing pending the poll is a bare E3, one opcode byte, which is what a
 * real puck sends 93% of the time.
 *
 * Returns the payload length.
 */
static uint8_t build_poll(int slot, uint8_t *p)
{
	uint8_t len;
	uint8_t rid;
	uint8_t data[sizeof(g_relay_data[0])];
	bool land01;
	k_spinlock_key_t key;

	p[0] = 0xE3;

	/* Take the pending relay whole, under the lock rf_link_relay() writes
	 * it under, and frame from the copy.
	 */
	key = k_spin_lock(&g_relay_lock);
	if (!g_relay_shots[slot]) {
		k_spin_unlock(&g_relay_lock, key);
		return 1;
	}
	len = g_relay_len[slot];
	rid = g_relay_rid[slot];
	memcpy(data, g_relay_data[slot], len);
	g_relay_shots[slot]--;
	k_spin_unlock(&g_relay_lock, key);

	/*
	 * Land only what must execute. 0x9F is power-off. The 0x87 registers
	 * the Arduino lands (LED brightness 0x2D, the id9 lizard keepalive, and
	 * the amplifier block 0x18/0x2E/0x34/0x35) are not sent by this port
	 * yet; when they are, they belong here too, and 0x30 must NOT land,
	 * because landing it freezes the gyro.
	 */
	land01 = (rid == 0x9F);

	if (land01) {
		p[1] = (uint8_t)(2 + len);
		p[2] = 0x01;
		p[3] = rid;
		p[4] = len;
		memcpy(p + 5, data, len);
		return (uint8_t)(5 + len);
	}

	p[1] = (uint8_t)(1 + len);
	p[2] = 0x05;
	p[3] = rid;
	memcpy(p + 4, data, len);
	return (uint8_t)(4 + len);
}

/*
 * The puck's own chords: it watches the controller's input stream and acts on
 * combinations the controller itself knows nothing about.
 *
 *   Steam + Y            power the controller off (relayed as a 0x9F "off!")
 *   QAM held             wake a sleeping console (borrow the radio for BLE)
 *   back-4 + L3 + R3     register with a console
 *   back-4 + L1 + R1     swap the USB identity and reboot
 *
 * Read off the raw Steam Controller button word before anything masks it, and
 * timed rather than counted, since the poll rate varies. Each fires once per
 * hold and re-arms only on release.
 *
 * The presses are not masked here. Dongle mode forwards the raw 0x45 to Steam,
 * which owns the Steam button. Pro Controller mode masks each chord's buttons
 * in switch_pro_buttons(), which lists these same chords, so a new chord needs
 * an entry in both places.
 *
 * TB_* come from puck_input.h, so the bit positions have one home.
 */
#define CHORD_HOLD_MS 2000u

/* The wake is a single dedicated button, so it needs less of a hold than a
 * chord, but not none: waking takes the radio off RF for several seconds.
 */
#define WAKE_HOLD_MS  1000u

/* Buttons live at report bytes [2..6], little endian. */
static uint32_t buttons_of(const uint8_t *rep)
{
	return (uint32_t)rep[2] | ((uint32_t)rep[3] << 8) |
	       ((uint32_t)rep[4] << 16) | ((uint32_t)rep[5] << 24);
}

/*
 * One chord's hold timer, with the hold time given. Returns true on the single
 * poll where the hold first completes, so the caller acts exactly once.
 */
static bool chord_held_for(uint32_t b, uint32_t mask, uint32_t hold_ms,
			   uint32_t *since, bool *fired)
{
	uint32_t now = k_uptime_get_32();

	if ((b & mask) != mask) {
		*since = 0;
		*fired = false;
		return false;
	}

	if (*since == 0) {
		*since = now;
	} else if (!*fired && (now - *since) >= hold_ms) {
		*fired = true;
		return true;
	}
	return false;
}

/* The two-second chords: deliberate, awkward-on-purpose combinations. */
static bool chord_held(uint32_t b, uint32_t mask, uint32_t *since, bool *fired)
{
	return chord_held_for(b, mask, g_cfg.chord_hold_ms, since, fired);
}

/*
 * Reboot into the USB identity the mode chord has just saved. Called from the
 * chord itself when there is no chime, or by the mode chime from the work
 * queue once its last stop has left.
 */
static void mode_reboot(void)
{
	printk("rf: rebooting to re-enumerate\n");
	k_sleep(K_MSEC(150));	/* let the console line flush */
	sys_reboot(SYS_REBOOT_COLD);
}

static void chord_check(int slot, const uint8_t *rep)
{
	uint32_t b = buttons_of(rep);

	if (chord_held(b, TB_STEAM | TB_Y, &g_chord_ms[slot],
		       &g_chord_fired[slot])) {
		printk("rf: slot%d Steam+Y held: powering the controller off\n",
		       slot);
		rf_queue_power_off(slot);
		/* Tell Steam now: the controller keeps streaming for about a
		 * second after being told to shut down, and without an explicit
		 * disconnect Steam keeps listing it as connected.
		 */
		puck_hid_note_power_off(slot);
	}

	/*
	 * The wake macro, on the QAM button, the three dots between the
	 * trackpads. This is what BLE exists for: the puck hands the radio
	 * over, calls the sleeping console, clears its lock screen, and takes
	 * the radio straight back.
	 *
	 * A dedicated button rather than a chord. QAM is unclaimed here: it
	 * carries a per-user remap code in the Arduino build, which this port
	 * leaves to the config screen.
	 *
	 * Held, not tapped. Waking takes the radio away from RF for several
	 * seconds, so a brush during play would drop the controller.
	 *
	 * Everything after this point happens on the main work queue: this is
	 * the RF thread, and it is about to be told to stop.
	 */
	if (chord_held_for(b, TB_QAM, g_cfg.wake_hold_ms, &g_wake_chord_ms[slot],
			   &g_wake_chord_fired[slot])) {
		printk("rf: slot%d QAM held: asking for a console wake\n",
		       slot);
		switch2_request_wake(slot);
	}

	/*
	 * All four back paddles + both stick clicks: register with a console.
	 *
	 * No face button, for the reason spelled out on the mode chord below:
	 * Steam owns the controller in dongle mode and binds every face button
	 * to something, so holding one as part of a chord makes Steam act on it
	 * too. Stick clicks are a deliberate grip and Steam does little with
	 * them.
	 *
	 * This is here so registration can happen at the console. Every other
	 * way in needs a PC, because the config page and the `p` key both work
	 * by rebooting into pairing mode, and Pro Controller mode never gets
	 * there anyway.
	 */
	if (chord_held(b, CHORD_BACK4 | TB_L3 | TB_R3, &g_reg_chord_ms[slot],
		       &g_reg_chord_fired[slot])) {
		printk("rf: slot%d back-4 + L3 + R3: registering with a console\n",
		       slot);
		switch2_request_register();
	}

	/*
	 * All four back paddles + both bumpers: swap the USB identity.
	 *
	 * NO FACE BUTTON. Steam owns the controller in dongle mode and binds
	 * all four: A and B are click, Y is middle click, X opens the on-screen
	 * keyboard. The puck reads the raw report regardless, so the chord
	 * fires either way, but Steam acts on the same press. Bumpers scroll at
	 * worst. (Steam calls them L1 and R1. The masks here keep the LB/RB
	 * names ported from the Arduino build.)
	 *
	 * NOT Steam either. That button is the controller's own power switch,
	 * and holding it through a chord and a reboot turns the controller off.
	 *
	 * The reboot is the point. USB identity is declared before any host
	 * software exists, so becoming something else means going away and
	 * coming back, and the puck is usually in a dock where nobody can
	 * replug it.
	 */
	if (chord_held(b, CHORD_BACK4 | TB_RB | TB_LB, &g_mode_chord_ms[slot],
		       &g_mode_chord_fired[slot])) {
		/* Ask what this boot IS, not what is stored. A dongle request
		 * is consumed at boot, so the stored value already reads "Pro"
		 * while this boot is running as the dongle.
		 */
		bool to_dongle = puck_usb_is_pro();

		printk("rf: slot%d back-4 + L1 + R1: USB identity -> %s\n", slot,
		       to_dongle ? "VALVE DONGLE (Steam, until next power-up)"
				 : "PRO CONTROLLER");

		if (switch2_store_save_usb_mode(to_dongle
							? PUCK_USB_MODE_DONGLE
							: PUCK_USB_MODE_PRO)) {
			printk("rf: mode save FAILED: staying as we are\n");
		} else if (g_cfg.mode_sound &&
			   puck_melody_play_chime(to_dongle ? PUCK_CHIME_DONGLE
							    : PUCK_CHIME_PRO,
						  slot, mode_reboot) == 0) {
			/*
			 * The chime reaches the controller through this thread's
			 * polls, so the reboot cannot happen here: the chime calls
			 * mode_reboot() once its last stop has left.
			 */
			printk("rf: mode chime, then rebooting\n");
		} else {
			mode_reboot();
		}
	}
}

/*
 * Decode an F1 input reply and forward every input report it carries to Steam.
 *
 * The reply is a list of TLVs starting at offset 3: [len][type][payload...].
 * Type 6 is a HID input report, and its first payload byte is the report id,
 * 0x45 on legacy firmware, 0x42 since the mid-2026 controller update. Both
 * decode identically: the 0x42 body [0..45] is byte-for-byte the same layout,
 * it just adds trailing bytes and two always-on status bits nothing reads.
 *
 * WALK ALL THE TLVs, not just the first: taking only [0] halves the report rate.
 * `idx` is int rather than uint8_t on purpose. A length byte of 0xFE would
 * wrap `idx += tlen + 2` mod 256 and spin forever, hanging USB.
 */
static void rf_decode_f1(int slot, uint8_t rxlen)
{
	int idx = 3;
	int end = rxlen + 2;

	while (idx + 1 < end) {
		uint8_t tlen = rfrx[idx];
		uint8_t ttype = rfrx[idx + 1];

		if (tlen == 0) {
			break;
		}

		/* Only a report that fits ENTIRELY inside rfrx: a short or
		 * garbled TLV must never let the decode read past the buffer.
		 */
		if (ttype == 6 && tlen >= 28 &&
		    (size_t)(idx + 2) + tlen <= sizeof(rfrx) &&
		    (rfrx[idx + 2] == 0x45 || rfrx[idx + 2] == 0x42)) {
			const uint8_t *rep = &rfrx[idx + 2];

			/* rep[1] is the sequence counter; a repeat means the
			 * poll caught the same report twice.
			 */
			if (rep[1] != g_last_seq[slot]) {
				g_last_seq[slot] = rep[1];
				g_fresh[slot]++;
			}

			/* Chords are read from every report, fresh or not: a
			 * held combo repeats the same sequence number.
			 */
			chord_check(slot, rep);

			/* Decode into the shared input state every output mode
			 * reads. That is what makes the puck a CONTROLLER
			 * rather than a relay.
			 */
			puck_input_update(slot, rep);

			puck_hid_send_input(slot, rep[0], rep + 1,
					    (uint16_t)(tlen - 1));
		}

		idx += tlen + 2;
	}
}
/*
 * Poll one slot and collect the controller's reply.
 *
 * The poll is a BARE 0xE3, one opcode byte, nothing else. That is what a real
 * puck sends: of 2003 polls in the reference air capture, 1857 were bare E3,
 * and the very first frame of the session was answered by an F1 input report
 * immediately. An earlier reverse-engineering "recipe" claimed 0xE7 awake-
 * announce plus a GET-report-0x45 sub-TLV were required; the capture says
 * otherwise, so the bare form is what gets ported.
 *
 * S1 carries the PID cycled per slot, with the low bit set (g_e3mode 1 in the
 * Arduino build): cycling the ESB PID is what drains the controller's report
 * queue at full rate, roughly 400 reports/s against ~60 with a fixed PID.
 *
 * Returns the reply length, or 0 on a silent poll.
 */
static uint8_t rf_poll_slot(int slot)
{
	uint8_t s1 = (uint8_t)(((g_poll_pid[slot]++ & 3) << 1) | 1);
	uint32_t t0;
	uint8_t plen;
	uint8_t rxlen = 0;

	g_polls++;

	memset(rftx, 0, sizeof(rftx));
	plen = build_poll(slot, rftx + 2);
	rftx[0] = plen;
	rftx[1] = s1;

	rf_config(RF_SESS_CH);
	rf_set_addr(g_sess_base[slot], g_sess_prefix[slot]);

	NRF_RADIO->PACKETPTR = (uint32_t)rftx;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->TASKS_TXEN = 1;
	rf_wait_disabled();
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
	rfrx[0] = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk |
			    RADIO_SHORTS_ADDRESS_RSSISTART_Msk |
			    RADIO_SHORTS_DISABLED_RSSISTOP_Msk;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->TASKS_RXEN = 1;

	/* The reply returns EARLY on EVENTS_END, so this window is only paid in
	 * full on a genuinely silent poll. 1200 us is the proven value. It was
	 * once raised to 2000 and that alone dropped the report rate, because a
	 * no-reply poll then no longer fits inside the 4 ms cycle.
	 */
	t0 = rf_micros();
	while (!NRF_RADIO->EVENTS_END && (rf_micros() - t0) < RF_RX_WIN_US) {
	}

	if (NRF_RADIO->EVENTS_END) {
		bool crcok = NRF_RADIO->CRCSTATUS & 1;

		NRF_RADIO->EVENTS_END = 0;
		rxlen = rfrx[0];

		if (!crcok) {
			g_crc_bad++;
			rxlen = 0;
		} else if (rxlen && rxlen <= 96) {
			/*
			 * Only F-type replies (0xF1 input, 0xF2 disconnect,
			 * 0xF3 status) count as a bonded controller. Every puck
			 * shares the "ibex" address and CRC config, so a second
			 * puck's E-type beacon lands here too, and without this
			 * gate its traffic would mark the link alive.
			 */
			uint8_t rtype = rfrx[2];

			if (rtype >= 0xF0) {
				puck_trace_stage(TR_RF_LIVE);
				g_link_ms[slot] = k_uptime_get_32();
				g_replies++;
				if (rtype == 0xF1) {
					g_f1[slot]++;
					rf_decode_f1(slot, rxlen);
				}
			} else {
				rxlen = 0;
			}
		} else {
			rxlen = 0;
		}
	}

	NRF_RADIO->TASKS_DISABLE = 1;
	rf_wait_disabled();
	NRF_RADIO->EVENTS_DISABLED = 0;
	return rxlen;
}

bool rf_link_slot_live(int slot)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS || !g_link_ms[slot]) {
		return false;
	}
	/* 500 ms, matching the Arduino build: long enough to ride out a few
	 * missed polls, short enough that a powered-off controller stops being
	 * reported as connected almost immediately.
	 */
	return (k_uptime_get_32() - g_link_ms[slot]) < 500u;
}

void rf_link_set_active(bool active)
{
	g_active = active;
	if (active) {
		g_beacons = 0;
		g_replies = 0;
	}
}

void rf_link_task(void)
{
	static uint32_t last_report;
	uint32_t now = k_uptime_get_32();

	if (!g_active) {
		return;
	}

	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		bool live;

		if (!puck_hid_bond(s)) {
			continue;
		}

		live = rf_link_slot_live(s);

		/*
		 * BACK OFF ON A DARK SLOT. A bonded slot whose controller is
		 * switched off, or which holds a stale bond, otherwise burns
		 * the full RX window every cycle and re-beacons on channel 2,
		 * dragging the radio off the session channel where a live
		 * controller is waiting. With one stale bond alongside one live
		 * one, the reply rate fell from 99% to 63% with 596 CRC errors
		 * and the input stuttered.
		 *
		 * So a slot silent for a while gets probed a few times a second
		 * instead. A live slot is never throttled.
		 */
		if (!live && (now - g_probe_ms[s]) < RF_DARK_PROBE_MS) {
			continue;
		}
		if (!live) {
			g_probe_ms[s] = now;
		}

		rf_poll_slot(s);

		/* Discovery is how a controller that has just powered on finds
		 * this puck's session address. Sent only while the slot is
		 * dark: once it answers polls, the beacon is pure airtime and a
		 * hop to channel 2 and back.
		 */
		if (!rf_link_slot_live(s) && (now - g_beacon_ms[s]) >= 250u) {
			g_beacon_ms[s] = now;
			rf_link_beacon(s, true);
		}
	}

	if (now - last_report >= 2000) {
		last_report = now;
		printk("rf: %u polls, %u replies, %u crc-bad |",
		       (unsigned)g_polls, (unsigned)g_replies,
		       (unsigned)g_crc_bad);
		for (int s = 0; s < PUCK_HID_SLOTS; s++) {
			if (!puck_hid_bond(s)) {
				continue;
			}
			printk(" slot%d %s (F1 %u)", s,
			       rf_link_slot_live(s) ? "LIVE" : "dark",
			       (unsigned)g_f1[s]);
		}
		printk("\n");
	}
}

/*
 * The RF layer gets its own thread rather than a slot in main's heartbeat
 * loop, which runs at 5 s, three orders of magnitude too slow for a link
 * whose poll cycle is 4 ms. Gated on g_active, so it does nothing while BLE
 * owns the radio: beaconing over a live BLE connection would trample the
 * controller's own radio events.
 *
 * Priority is deliberately BELOW the BLE stack's: this thread busy-waits inside
 * the RX window, and it must never be able to starve the link layer during a
 * handover.
 */
#define RF_THREAD_STACK 1024
#define RF_THREAD_PRIO  7

static void rf_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (1) {
		if (!g_active) {
			/*
			 * The radio is elsewhere (BLE, or not started yet), but
			 * the USB Pro Controller still has to be serviced: the
			 * console begins its handshake the moment the cable
			 * goes in, long before any controller has associated.
			 *
			 * The idle sleep stays at 200 ms unless this boot is
			 * actually a Pro Controller, so dongle mode keeps
			 * exactly the timing it has always had.
			 */
			switch_pro_usb_task();
			switch_pro_mouse_task();
			k_sleep(puck_usb_is_pro() ? K_MSEC(10) : K_MSEC(200));
			continue;
		}

		rf_link_task();
		/* Only when the dongle interfaces exist. PRO mode has none, and
		 * hid_dev[] is NULL there, which once caused a NULL
		 * dereference on the very first RF poll. COMBO carries them, so it runs there.
		 */
		if (puck_usb_has_dongle()) {
			puck_hid_conn_task();
		}

		/*
		 * Serviced from here rather than a thread of its own.
		 *
		 * A 2 ms loop at priority 6, above this thread and the console
		 * at 7, wedged the CDC console a few seconds after RF came up,
		 * hard enough that the host could neither open the port ("the
		 * semaphore timeout period has expired") nor kill the reader.
		 * Priority 8 at 8 ms did not help either.
		 *
		 * STILL OPEN: with RF running, Pro streaming can wedge USB even
		 * from here. The untested difference is the interrupt OUT
		 * endpoint, which the dongle interfaces do not have.
		 *
		 * Inert in dongle mode: pro_dev is NULL unless this boot
		 * registered a Pro Controller.
		 */
		switch_pro_usb_task();
		switch_pro_mouse_task();

		k_sleep(K_MSEC(10));
	}
}

K_THREAD_DEFINE(rf_thread, RF_THREAD_STACK, rf_thread_fn, NULL, NULL, NULL,
		RF_THREAD_PRIO, 0, 0);

void rf_link_dump(void)
{
	printk("\n--- bond slots ---\n");
	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		const uint8_t *rec = puck_hid_bond(s);

		if (!rec) {
			printk("  slot %d: empty\n", s);
			continue;
		}
		printk("  slot %d: uuid ", s);
		for (int i = 0; i < 8; i++) {
			printk("%02x", rec[i]);
		}
		printk(" serial %.16s addr %02x%02x%02x%02x/%02x %s (F1 %u)\n",
		       (char *)rec + 8, g_sess_base[s][0], g_sess_base[s][1],
		       g_sess_base[s][2], g_sess_base[s][3], g_sess_prefix[s],
		       rf_link_slot_live(s) ? "LIVE" : "dark", (unsigned)g_f1[s]);
	}
}

void rf_link_bond_changed(int slot)
{
	const uint8_t *rec;

	if (slot < 0 || slot >= PUCK_HID_SLOTS) {
		return;
	}

	rec = puck_hid_bond(slot);
	if (!rec) {
		/* Cleared: wipe the address so a stale one cannot be beaconed
		 * or polled if the slot is later reused.
		 */
		memset(g_sess_base[slot], 0, 4);
		g_sess_prefix[slot] = 0;
		g_link_ms[slot] = 0;
		return;
	}

	derive_session_addr(slot, rec + 4);
	printk("rf: slot %d session addr %02x%02x%02x%02x/%02x ch%u\n", slot,
	       g_sess_base[slot][0], g_sess_base[slot][1], g_sess_base[slot][2],
	       g_sess_base[slot][3], g_sess_prefix[slot], RF_SESS_CH);
}

bool rf_link_slot_conn(int slot)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS || !g_link_ms[slot]) {
		return false;
	}
	/*
	 * Tighter than rf_link_slot_live()'s 500 ms. This one decides what
	 * Steam is TOLD, and a controller that has been switched off should
	 * disappear from its list promptly rather than linger.
	 */
	return (k_uptime_get_32() - g_link_ms[slot]) < 300u;
}
