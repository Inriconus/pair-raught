/*
 * The BLE half of the puck: wakes a sleeping Nintendo Switch 2, and presents
 * itself to it as a controller.
 *
 * WHAT THIS IS FOR. The product is a Steam Controller talking to this puck over
 * RF, with the puck plugged into the Switch's USB port acting as a controller.
 * BLE exists for one job: when asked, wake the console, clear its lock screen,
 * and hand the radio straight back to RF. Waking is the success condition,
 * not "a controller appears on screen".
 *
 * WHY ZEPHYR AT ALL. The Switch 2 opens controller connections with a
 * CONNECT_IND carrying interval=4 (5.00 ms), below the 7.5 ms Bluetooth spec
 * floor. Nordic's closed SoftDevice silently refuses to join. Measured, 213
 * such CONNECT_INDs went completely unanswered. Zephyr's controller is source
 * open to patch, which is the entire reason this exists. See
 * patches/ull_peripheral-accept-5ms.patch; note it needs BOTH halves,
 * because below the spec floor the controller also reinterprets the interval in
 * low-latency units and would run the link at 2.5 ms.
 *
 * THE WAKE, in the order it happens:
 *   off air  ->  advertise (payload[9] = 0x81, ~21 ms interval)  <- the wake action
 *   ->  sleeping console connects at 5 ms  ->  encrypts from the LTK in NVS
 *   ->  report stream  ->  press B four times to clear the lock screen
 *   ->  disconnect and go silent, or the console can never sleep again.
 *
 * Console keys:
 *
 *   p  advertise for pairing        w  advertise in wake mode
 *   x  go silent, then advertise    h  arm a HOME press
 *   u  cycle the unlock button      m  switch USB identity
 *   r  hand the radio to RF         e  hand the radio to BLE
 *   b  reboot to the bootloader drive
 *   t  play a 440 Hz haptic tone     T  the same on the rumble motor
 *   k  play the wake chime, without waking anything
 *
 *   g  dump the GATT table          s  dump RF link and HID TX state
 *   i  dump controller input        j  dump Pro Controller state
 *   c  dump settings                 ?  this list
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_rtc.h>
#include <hal/nrf_radio.h>
#include <zephyr/sys/reboot.h>

#include "switch2_gatt.h"
#include "switch2_store.h"
#include "rf_radio.h"
#include "rf_link.h"
#include "puck_input.h"
#include "switch_pro.h"
#include "puck_usb.h"
#include "puck_hid.h"
#include "puck_trace.h"
#include "puck_settings.h"
#include "switch_pro_usb.h"
#include "puck_melody.h"

LOG_MODULE_REGISTER(s2ll, LOG_LEVEL_INF);

/* The puck's public address, derived from this chip's DEVICEID. See puck_addr()
 * in switch2_gatt.c.
 */

/*
 * Manufacturer data in wake mode, as captured off the wire:
 *
 *   0100037e05662000 01 00 <console addr, 6 bytes> 0f00 0000000000
 *
 * build_adv_data() below fills it in and explains the two fields that move.
 */
/*
 * The bonded console's address, wire order. Learned at registration and persisted
 * to NVS, never compiled in: the wake advert carries this address in the
 * reconnect field the console matches against, so a hardcoded one would have
 * every puck inviting the same Switch.
 *
 * Empty until a registration completes.
 */
static uint8_t g_console_mac[6];
static bool g_have_host;
static bool g_repair_req;

/* True only while a wake is deliberately being asked for. Drives the
 * 0x81 wake byte; see build_adv_data(). Left set, it wakes the console again
 * the instant it goes to sleep, forever.
 */
static bool g_wake_request;

static uint8_t g_mfg[26];

/*
 * WAKE mode on boot: "I belong to the console I am bonded to".
 *
 * A pairing advertiser broadcasts a zeroed reconnect MAC and the generic 0xF0
 * byte, and the console only picks that form up from Change Grip/Order. An idle
 * or sleeping Switch scans for ITS OWN address in that field, so a pairing
 * advertiser is invisible to it by design.
 *
 * An autonomous return therefore needs both halves of the bond back after a
 * power cut: the console's address (g_console_mac) and the link key. The
 * console does not re-run the PAIR ceremony on reconnect, and without the
 * stored key the link dies in ~0.4 s with LL_REJECT_EXT_IND 0x06, "PIN or Key
 * Missing". Both live in NVS; see switch2_store_load_key().
 *
 * Press `p` when the console has genuinely forgotten the puck and a fresh
 * registration from Change Grip/Order is needed.
 */
static bool g_pairing;

/*
 * Rebuild the manufacturer data. g_mfg[i] is payload[i-2], because the first
 * two bytes are the company ID.
 *
 * The two modes differ in two places:
 *
 *   payload[10:16] (g_mfg[12..17])  the console being invited, wire order.
 *                                   Zero in pairing mode.
 *   the advert byte                 PAIRING, generic, g_mfg[19] = 0xF0: the
 *                                     console discovers it. Needed to register
 *                                     in the first place.
 *                                   WAKE, real, g_mfg[18] = 0x0F: what genuine
 *                                     hardware broadcasts, and how the console
 *                                     recognises a controller it already knows,
 *                                     from any screen.
 *
 * Everything else is fixed. Identity is Joy-Con 2 Right (VID 0x057E / PID
 * 0x2066). There is deliberately no name AD structure: the real advertisement
 * has none, and at 31 bytes there is no room for one anyway.
 */
static void build_adv_data(void)
{
	memset(g_mfg, 0, sizeof(g_mfg));

	g_mfg[0] = 0x53;	/* Nintendo company ID 0x0553, little endian */
	g_mfg[1] = 0x05;
	g_mfg[2] = 0x01;
	g_mfg[3] = 0x00;
	g_mfg[4] = 0x03;
	g_mfg[5] = 0x7E;	/* VID 0x057E */
	g_mfg[6] = 0x05;
	g_mfg[7] = 0x66;	/* PID 0x2066, Joy-Con 2 (R) */
	g_mfg[8] = 0x20;
	g_mfg[9] = 0x00;
	g_mfg[10] = 0x01;

	if (g_pairing) {
		g_mfg[19] = 0xF0;
	} else {
		memcpy(&g_mfg[12], g_console_mac, sizeof(g_console_mac));
		g_mfg[18] = 0x0F;

		/*
		 * THE WAKE BYTE, payload[9]. ONLY while a wake is being asked
		 * for.
		 *
		 * 0x81 means "wake up". Left set, it puts the console in a loop:
		 * it sleeps, sees a standing wake request, and wakes straight
		 * back up. Ordinary wake-mode advertising, a registered
		 * controller making itself available, carries 0x00.
		 *
		 * Real hardware escalates rather than shouting. It opens a burst
		 * with 0x00 and switches to 0x81 only once the console has not
		 * answered.
		 */
		g_mfg[11] = g_wake_request ? 0x81 : 0x00;
	}
}

static const struct bt_data ADV[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, g_mfg, sizeof(g_mfg)),
};


/*
 * Reboot into the Adafruit nRF52 bootloader without touching the board. The
 * Arduino uploader did this with a 1200 baud touch; Zephyr's USB stack has no
 * such hook, so without this every reflash needs a double-tap.
 *
 * The bootloader reads GPREGRET on startup:
 *   0x4E  DFU_MAGIC_SERIAL_ONLY_RESET   serial DFU, what adafruit-nrfutil wants
 *   0x57  DFU_MAGIC_UF2_RESET           UF2 mass-storage mode
 *
 * UF2, so reflashing is the same act as flashing: a drive appears and you copy
 * the .uf2 onto it. When adafruit-nrfutil is installed, build.sh also packages
 * the same zephyr.hex as a serial-DFU .zip, for a board sitting in serial DFU
 * instead.
 */
#define DFU_MAGIC_UF2_RESET 0x57

static void reboot_to_bootloader(void)
{
	nrf_power_gpregret_set(NRF_POWER, 0, DFU_MAGIC_UF2_RESET);
	NVIC_SystemReset();
}

/*
 * Console reader. Its own thread because console_getchar() blocks.
 *
 * uart_poll_in() from the main loop never sees a byte on a CDC ACM console:
 * the device is interrupt driven, so keys are silently discarded.
 * CONFIG_CONSOLE_GETCHAR selects UART_INTERRUPT_DRIVEN and gives the blocking
 * getchar this thread sits on.
 */
static void set_adv_mode(bool pairing);
static void enter_rf_mode(void);
static void request_rf_mode(void);
static void enter_ble_mode(void);
static void start_adv(void);
static volatile int g_adv_err;
static bool g_silent;

static void console_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	/*
	 * Single keypresses drive everything a person needs at a terminal. A
	 * line beginning with '$' is a config command instead, buffered until
	 * Enter, the channel the configuration UI speaks, and keeping
	 * it behind a prefix means every existing shortcut still works
	 * untouched.
	 */
	/* Room for the longest config command, "$chime dongle set" with every
	 * note at its widest: sixteen of "2000:1000:4 " is 192 characters.
	 */
	static char line[256];
	static uint8_t used;
	static bool in_cmd;

	console_init();

	while (1) {
		int ch = console_getchar();

		if (in_cmd) {
			if (ch == '\r' || ch == '\n') {
				line[used] = '\0';
				in_cmd = false;
				used = 0;
				puck_settings_command(line);
			} else if (ch == 8 || ch == 127) {	/* backspace */
				if (used) {
					used--;
				}
			} else if (used < sizeof(line) - 1 && ch >= ' ') {
				line[used++] = (char)ch;
			}
			continue;
		}

		if (ch == '$') {
			in_cmd = true;
			used = 0;
			continue;
		}

		switch (ch) {
		case 'N':
			/* Request a re-pair: the NEXT boot comes up in BLE
			 * pairing mode even though a console is stored. This is
			 * what the configuration screen's re-pair control will
			 * set; the flag is consumed by that boot, so an
			 * abandoned attempt cannot strand the puck off RF.
			 */
			if (switch2_store_request_repair() == 0) {
				printk("\nre-pair armed: reboot to enter BLE pairing mode\n");
			} else {
				printk("\nre-pair could NOT be stored (NVS)\n");
			}
			break;

		case 's':
		case 'S':
			rf_link_dump();
			puck_hid_dump_tx();
			puck_trace_dump();
			puck_usb_dump_mode();
			switch_pro_usb_dump();
			break;

		case 'i':
		case 'I':
			/* Raw decoded input, straight off the RF report. */
			for (int s = 0; s < PUCK_HID_SLOTS; s++) {
				puck_input_dump(s);
			}
			break;

		case 'j':
		case 'J':
			/* The same input mapped to the Pro Controller, which is
			 * what the console will see once the USB device exists.
			 */
			for (int s = 0; s < PUCK_HID_SLOTS; s++) {
				switch_pro_dump(s);
			}
			break;

		case 'c':
		case 'C':
			puck_settings_dump();
			break;

		case 'm':
		case 'M': {
			/*
			 * Cycle the USB identity: PRO -> DONGLE -> MOUSE ONLY.
			 * Applied on reboot, since a device declares itself
			 * before any host software exists and becoming
			 * something else means going away and coming back.
			 *
			 * Only PRO persists. A dock power-cycles the puck every
			 * time the console sleeps, and neither the dongle nor
			 * mouse-only is a state to wake up in.
			 */
			uint8_t now = switch2_store_load_usb_mode();
			uint8_t next;
			const char *name;

			if (now == PUCK_USB_MODE_PRO) {
				next = PUCK_USB_MODE_DONGLE;
				name = "VALVE DONGLE (Steam, one boot)";
			} else if (now == PUCK_USB_MODE_DONGLE) {
				next = PUCK_USB_MODE_MOUSE;
				name = "MOUSE ONLY (one boot)";
			} else {
				next = PUCK_USB_MODE_PRO;
				name = "PRO CONTROLLER";
			}

			if (switch2_store_save_usb_mode(next)) {
				printk("\nUSB mode: SAVE FAILED, staying as "
				       "we are\n");
				break;
			}
			printk("\n--- USB mode -> %s ---\n", name);
			printk("    reboot to apply ('b' for bootloader, or "
			       "just replug)\n");
			break;
		}
		case 'b':
		case 'B':
			printk("\nrebooting into the bootloader\n");
			k_sleep(K_MSEC(150));
			reboot_to_bootloader();
			break;
		case 'p':
		case 'P':
			set_adv_mode(true);
			break;
		case 'w':
		case 'W':
			set_adv_mode(false);
			break;
		case 'g':
		case 'G':
			switch2_gatt_dump();
			switch2_gatt_check();
			break;
		case 'h':
		case 'H':
			switch2_gatt_arm_wake();
			break;
		case 'u':
		case 'U':
			switch2_gatt_cycle_unlock_button();
			break;
		case 'r':
		case 'R':
			request_rf_mode();
			break;
		case 'e':
		case 'E':
			enter_ble_mode();
			break;
		case 'x':
		case 'X':
			/*
			 * Go silent, then come back on air. Advertising IS the
			 * wake action: a real Joy-Con sits silent, and when its
			 * button is pressed and it starts advertising, the
			 * sleeping console connects about 1.4 s later.
			 *
			 * A puck that advertises forever is permanent
			 * background by the time the console sleeps, so this
			 * pair of keys reproduces the appearing part.
			 */
			if (!g_silent) {
				g_silent = true;	/* before stopping: the
							 * heartbeat retries a
							 * failed advert, and
							 * would undo this. */
				printk("\n--- going SILENT (stop advertising) -> %d ---\n",
				       bt_le_adv_stop());
				g_adv_err = -1;
			} else {
				printk("\n--- advertising again (the wake macro) ---\n");
				g_silent = false;
				/* This advert IS the wake action: set the wake
				 * byte, and expect the unlock sequence once the
				 * console opens the stream.
				 */
				g_wake_request = true;
				switch2_gatt_set_wake_pending(true);
				build_adv_data();
				start_adv();
			}
			break;
		case 't':
		case 'T': {
			/*
			 * HAPTIC TONE TEST: 440 Hz for half a second on the
			 * right trackpad, or on the right rumble motor for 'T'.
			 * The command, and why gain stays at medium, are in
			 * puck_melody.c.
			 *
			 * A stop first, and a gap after every stop: the relay
			 * holds one command per slot, so anything queued
			 * straight behind a stop burst overwrites it before it
			 * has gone out. Steam or the console writing haptics
			 * meanwhile overwrites the relay too, so test with
			 * neither driving it.
			 */
			uint8_t haptic = (ch == 'T') ? PUCK_HAPTIC_RUMBLE_R
						     : PUCK_HAPTIC_PAD_R;
			int slot = -1;

			for (int s = 0; s < PUCK_HID_SLOTS; s++) {
				if (rf_link_slot_live(s)) {
					slot = s;
					break;
				}
			}
			if (slot < 0) {
				printk("\ntone: no controller answering polls\n");
				break;
			}

			printk("\ntone: slot%d haptic %u, 440 Hz for 500 ms\n",
			       slot, haptic);
			puck_haptic_stop(slot, haptic);
			k_sleep(K_MSEC(100));
			puck_haptic_tone(slot, haptic, 440,
					 PUCK_HAPTIC_GAIN_MEDIUM);
			k_sleep(K_MSEC(500));
			puck_haptic_stop(slot, haptic);
			printk("tone: stop sent\n");
			break;
		}
		case 'k':
		case 'K': {
			/* The wake chime on its own, without the wake behind it,
			 * to hear the melody without waking anything.
			 */
			int slot = -1;

			for (int s = 0; s < PUCK_HID_SLOTS; s++) {
				if (rf_link_slot_live(s)) {
					slot = s;
					break;
				}
			}
			if (slot < 0) {
				printk("\nchime: no controller answering polls\n");
			} else if (puck_melody_play_wake(slot, NULL)) {
				printk("\nchime: already playing\n");
			}
			break;
		}
		case '?':
			printk("keys:\n"
			       "  p advertise for pairing    w advertise in wake mode\n"
			       "  x go silent then advertise h arm a HOME press\n"
			       "  u cycle unlock button      m switch USB identity\n"
			       "  r radio to RF              e radio to BLE\n"
			       "  b bootloader drive         t tone test (T: rumble)\n"
			       "  k play the wake chime\n"
			       "  g dump GATT   s dump RF/TX   i dump input\n"
			       "  j dump Pro    c dump settings  ? this list\n");
			break;
		default:
			break;
		}
	}
}

K_THREAD_DEFINE(console_tid, 1024, console_thread, NULL, NULL, NULL, 7, 0, 0);

/*
 * State that survives dropped log messages.
 *
 * Deferred logging silently discarded the "advertising started" line on the
 * first two runs (the log jumped mid-sentence from boot straight to
 * the heartbeat) so a one-shot LOG_INF is not evidence of anything. The heartbeat
 * re-reports these every time instead.
 */
/* g_adv_err and g_silent are declared up by the console thread, which uses
 * them; g_silent means "deliberately off air" (the `x` key) and suppresses the
 * heartbeat's retry, which exists to recover from the -ENOMEM race and would
 * otherwise start advertising again immediately.
 */
static volatile uint32_t g_adv_calls;

/* The current link, referenced, so the console thread can drop it on a mode
 * change. NULL whenever nothing is connected.
 */
static struct bt_conn *g_conn;

/*
 * Count the connection objects the host currently holds.
 *
 * On this build bt_le_adv_start() can only return -ENOMEM from one place:
 * le_adv_start_add_conn() -> bt_conn_add_le() -> NULL, every slot in
 * acl_conns[] already referenced. The other -ENOMEM sources in adv.c sit
 * behind CONFIG_BT_EXT_ADV, which is off.
 *
 * So a non-zero count here is the failure. Zero means it came from somewhere
 * this has not found yet.
 */
static void count_conn(struct bt_conn *conn, void *data)
{
	unsigned int *n = data;
	struct bt_conn_info info;

	(*n)++;
	if (!bt_conn_get_info(conn, &info)) {
		/* The LIVE interval, not the one seen at connect. Whether a
		 * sub-spec link is actually being HELD is the whole point of
		 * this port, and it was previously only ever observed once, at
		 * connection setup.
		 *
		 * The peer is shown by its last octet only, as in connected().
		 */
		printk("  conn[%u] state=%d role=%u peer=..:%02X interval=%u (%u.%02u ms)%s\n",
		       *n - 1, info.state, info.role, info.le.dst->a.val[0],
		       info.le.interval,
		       (info.le.interval * 125) / 100,
		       (info.le.interval * 125) % 100,
		       info.le.interval < 6 ? "  <<< SUB-SPEC, HELD" : "");
	}
}

static unsigned int conn_objects(const char *when)
{
	unsigned int n = 0;

	bt_conn_foreach(BT_CONN_TYPE_LE, count_conn, &n);
	printk("conn objects %s: %u of %d\n", when, n, CONFIG_BT_MAX_CONN);
	return n;
}

/*
 * The same census without the printing, for the handover to poll. A connection
 * object stays allocated until Zephyr has released its last reference, so zero
 * here means nothing is left holding the controller's only slot.
 */
static void count_held(struct bt_conn *conn, void *data)
{
	ARG_UNUSED(conn);
	(*(unsigned int *)data)++;
}

static unsigned int conn_held(void)
{
	unsigned int n = 0;

	bt_conn_foreach(BT_CONN_TYPE_LE, count_held, &n);
	return n;
}

/*
 * ADVERTISING INTERVAL, measured off a real Joy-Con 2: 41 of 50 advert gaps on
 * channel 37 are 21 ms. Zephyr's BT_GAP_ADV_FAST_INT_*_2, which this used
 * before, measured 103 ms on air, five times slower.
 *
 * The Switch 2 scans passively. Across a 75-minute capture it never sent one
 * SCAN_REQ, only CONNECT_INDs, and a passive scanner only sees adverts that
 * land inside its own scan windows. A sleeping console at a low duty cycle is
 * about five times more likely to catch a 21 ms advertiser. An awake one scans
 * hard enough that 103 ms still gets picked up in a second, which is why this
 * only shows up when the console is asleep.
 *
 * Units are 0.625 ms: 32 = 20.0 ms, 36 = 22.5 ms, bracketing the measured 21.
 */
#define ADV_INT_MIN 32
#define ADV_INT_MAX 36

static void start_adv(void)
{
	const struct bt_le_adv_param param = {
		.id = BT_ID_DEFAULT,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = ADV_INT_MIN,
		.interval_max = ADV_INT_MAX,
		.peer = NULL,
	};

	conn_objects("before start_adv");

	g_adv_calls++;
	g_adv_err = bt_le_adv_start(&param, ADV, ARRAY_SIZE(ADV), NULL, 0);

	/* printk, NOT LOG_INF: deferred logging dropped this exact line on three
	 * separate runs, which hid a hard -ENOMEM failure behind a message that
	 * simply never appeared. printk goes straight out.
	 */
	printk("start_adv #%u -> %d  (ad_len=%u)\n",
	       g_adv_calls, g_adv_err, (unsigned)ARRAY_SIZE(ADV));

	if (g_adv_err == -ENOMEM) {
		/* Isolate the cause: retry WITHOUT the connectable option. If a
		 * non-connectable advert starts, the failure is in the connection
		 * object allocation (le_adv_start_add_conn -> bt_conn_add_le), not
		 * in the advertising data or the radio.
		 */
		const struct bt_le_adv_param nconn = {
			.id = BT_ID_DEFAULT,
			.options = 0,
			.interval_min = ADV_INT_MIN,
			.interval_max = ADV_INT_MAX,
			.peer = NULL,
		};
		int e2 = bt_le_adv_start(&nconn, ADV, ARRAY_SIZE(ADV), NULL, 0);

		printk("  non-connectable retry -> %d %s\n", e2,
		       e2 == 0 ? "(so the ADV DATA and radio are fine; "
				 "the connectable path is what fails)"
			       : "(fails too: data or radio)");

		/* Leave the radio idle either way: a non-connectable advert on
		 * air would be worse than silence here, because the console
		 * would see a controller it cannot connect to, and the sniffer
		 * would show a half-broken state.
		 */
		if (e2 == 0) {
			printk("  stopping the non-connectable advert -> %d\n",
			       bt_le_adv_stop());
		}
	}
}

/*
 * Switch between pairing and wake advertising without a reflash.
 *
 * A live advertiser will NOT pick up new data: the old payload keeps going out
 * after the mode is changed. So stop first, then rebuild, then start.
 *
 * A connected central means nothing is being advertised, so the payload cannot
 * reach anyone until that link ends. Disconnect and let disconnected() restart
 * the advertiser with the new bytes.
 */
/*
 * Registration finished: become a controller this console OWNS.
 *
 * Only the payload is rebuilt here, deliberately. The link is up at this point
 * so nothing is being advertised anyway, and the next start_adv() on disconnect
 * picks the new bytes up. Tearing the link down to apply them would abandon the
 * registration that just completed.
 */
/*
 * Wake finished: let the console go.
 *
 * A connected controller keeps a Switch awake. Held open, the puck would
 * stream reports for as long as it was plugged in and the console could never
 * sleep again.
 *
 * So once the lock screen is cleared: stop streaming, drop the link, and go off
 * air. This is where BLE mode ends and RF resumes. Advertising restarts on the
 * next wake request (`x`).
 */
/*
 * ---------------------------------------------------------------------------
 * RADIO OWNERSHIP: the seam the RF port folds into.
 * ---------------------------------------------------------------------------
 *
 * rf_link drives NRF_RADIO directly, hand-rolled ESB over raw registers, and it
 * wants the same peripheral the BLE controller does. The two can never run at
 * once, so the puck owns the radio in one of two modes and hands it over
 * deliberately. RF is the normal state; BLE exists to register with a console
 * and to wake it.
 *
 * The handover is real: bt_disable() reaches ll_deinit(), which runs
 * ll_reset() -> lll_deinit() -> ticker_deinit() and releases the radio.
 */
enum puck_mode { MODE_BLE, MODE_RF };
static enum puck_mode g_mode = MODE_BLE;

static void enter_rf_mode(void)
{
	int err;
	unsigned int held;

	if (g_mode == MODE_RF) {
		printk("already in RF mode\n");
		return;
	}

	printk("\n--- handing the radio to RF ---\n");

	g_silent = true;
	bt_le_adv_stop();

	/*
	 * Normally nothing is held by now: to_rf_work_fn() waits for the link
	 * to be released before calling this. A connection object still held
	 * survives bt_disable() and fills the only slot, so the next wake cannot
	 * advertise. Say so rather than fail quietly later.
	 */
	held = conn_held();
	if (held) {
		printk("  %u conn object(s) still held at handover: the next "
		       "wake may fail to advertise (-ENOMEM)\n", held);
	}

	/*
	 * Drop our reference rather than carry it across bt_disable(), where
	 * the next bt_conn_disconnect() on it would be a use-after-free.
	 */
	if (g_conn) {
		bt_conn_unref(g_conn);
		g_conn = NULL;
	}
	switch2_gatt_radio_release();

	err = bt_disable();
	printk("  bt_disable -> %d\n", err);
	if (err) {
		printk("  RADIO NOT RELEASED: rf_link cannot own it; the port\n"
		       "  would need MPSL timeslots instead of a mode switch\n");
		return;
	}

	g_mode = MODE_RF;
	printk("  RADIO.STATE=%u (0 = DISABLED, i.e. free for rf_link)\n",
	       (unsigned)nrf_radio_state_get(NRF_RADIO));

	/*
	 * Bring up what the radio needs that BLE was previously providing.
	 * HFCLK especially: with the controller down nothing else requests the
	 * 16 MHz crystal, and without it the radio looks alive and transmits
	 * NOTHING, the same silent failure as the LFCLK problem at the start
	 * of this port, so it is checked rather than assumed.
	 */
	rf_time_init();
	if (rf_hfclk_on() == 0) {
		printk("  HFCLK running, 1 MHz us-clock up (rf_micros=%u)\n",
		       rf_micros());
	}

	rf_config(RF_PAIR_CH);
	printk("  radio configured: 2Mbit, ch%u, CRC16/0x11021, addr \"ibex\"+0x%02X\n",
	       RF_PAIR_CH, RF_PAIR_PREFIX);
	/* Hand the radio to the RF layer and start beaconing for whatever Steam
	 * has bonded. Nothing happens if no slot is bonded. The beacon carries
	 * the bond's uuids, so there is nothing to say without one.
	 */
	rf_link_init();
	rf_link_set_active(true);
	puck_trace_stage(TR_RF_UP);
	printk("  RF mode: NRF_RADIO is now ours. Press 'e' to take it back for BLE.\n");
}

static void enter_ble_mode(void)
{
	int err;

	if (g_mode == MODE_BLE) {
		printk("already in BLE mode\n");
		return;
	}

	printk("\n--- taking the radio back for BLE ---\n");
	/* RF must let go BEFORE bt_enable(): both drive NRF_RADIO directly and
	 * the beacon thread would trample the link layer's radio events.
	 */
	rf_link_set_active(false);

	err = bt_enable(NULL);
	printk("  bt_enable -> %d\n", err);
	if (err) {
		printk("  BLE DID NOT COME BACK: the handover is one-way, which\n"
		       "  would mean the puck cannot return to RF and wake again\n");
		return;
	}

	/* The attribute table is registered dynamically, so it has to be put
	 * back after a disable/enable cycle, and the stored link key reloaded
	 * with it.
	 */
	err = switch2_gatt_init();
	printk("  GATT re-registered -> %d\n", err);

	g_mode = MODE_BLE;
	g_silent = false;
	build_adv_data();
	start_adv();
}

/*
 * Hand the radio back to RF, once the stack has let go of the link.
 *
 * Deferred rather than immediate: the callers finish inside a GATT callback or
 * a timeout, and tearing the BLE stack down from within its own callback is not
 * safe.
 *
 * Zephyr runs its connection teardown on the system workqueue, the same queue
 * as this: the disconnected callback, and the release of the host's last
 * reference to the connection. Disconnecting, sleeping and calling bt_disable()
 * inside one work item gave neither a chance to run first, so a link still up
 * at that moment survived bt_disable() holding the only connection slot.
 * Registration always handed over with the console still connected, and every
 * wake after it then failed to advertise with -ENOMEM until a reboot.
 *
 * So the first pass stops the stream and asks for the disconnect, and each pass
 * after that goes back to the queue for RF_HANDOVER_POLL_MS until nothing holds
 * a connection object. RF_HANDOVER_WAIT_MS bounds the wait. It is longer than
 * the console's 2 s supervision timeout, which is how long a link it never
 * acknowledges takes to be declared gone.
 */
#define RF_HANDOVER_POLL_MS 20
#define RF_HANDOVER_WAIT_MS 3000

static bool g_handover_waiting;
static uint32_t g_handover_since;

static void to_rf_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(g_to_rf_work, to_rf_work_fn);

static void to_rf_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (g_mode == MODE_RF) {
		g_handover_waiting = false;
		return;
	}

	if (!g_handover_waiting) {
		g_handover_waiting = true;
		g_handover_since = k_uptime_get_32();
		g_silent = true;	/* disconnected() must not re-advertise */
		bt_le_adv_stop();
		switch2_gatt_radio_release();
		if (g_conn) {
			printk("\n  handover: releasing the link first -> %d\n",
			       bt_conn_disconnect(g_conn,
						  BT_HCI_ERR_REMOTE_USER_TERM_CONN));
		}
	}

	if (conn_held() &&
	    k_uptime_get_32() - g_handover_since < RF_HANDOVER_WAIT_MS) {
		k_work_schedule(&g_to_rf_work, K_MSEC(RF_HANDOVER_POLL_MS));
		return;
	}

	g_handover_waiting = false;
	enter_rf_mode();
}

/* Hand over now, through the same wait. The one way in from anywhere else. */
static void request_rf_mode(void)
{
	k_work_reschedule(&g_to_rf_work, K_NO_WAIT);
}

/*
 * THE WAKE MACRO: holding QAM on the controller wakes a sleeping console.
 *
 * This is the whole reason BLE exists in this firmware. The puck lives on RF
 * as a controller; when the macro fires it borrows the radio, advertises with
 * the wake byte set, lets the console connect and clear its lock screen, then
 * gives the radio straight back.
 */
/* Console never answered, come home. Configurable: g_cfg.wake_giveup_s. */

/* Let USB settle before Pro Controller mode takes the radio to RF. */
#define PRO_RF_DELAY_S 5

static bool g_wake_active;	/* a macro-driven wake is in flight */

static void wake_timeout_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(g_wake_timeout_work, wake_timeout_fn);

static void wake_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	printk("\n*** WAKE MACRO: borrowing the radio to call the console ***\n");

	/*
	 * Set the wake byte BEFORE the handover: enter_ble_mode() builds the
	 * advert and starts it in one go, and an advert without 0x81 is just a
	 * controller saying hello. The 0x81 is what asks a sleeping console to
	 * wake, and it must be transient. switch2_wake_complete() clears it
	 * again, or the console would wake the instant it managed to sleep.
	 */
	g_wake_active = true;
	g_wake_request = true;
	switch2_gatt_set_wake_pending(true);

	enter_ble_mode();

	/*
	 * If the console never answers (flat battery, out of range, or
	 * simply not there) the puck must NOT sit on BLE forever. RF is dead while
	 * BLE owns the radio, so the Steam Controller would go silent with no
	 * way back short of a replug. Always come home on a timer.
	 *
	 * A wake that failed because the console was flat is indistinguishable
	 * from a firmware fault, and has been mistaken for one before.
	 */
	k_work_schedule(&g_wake_timeout_work, K_SECONDS(g_cfg.wake_giveup_s));
}

static K_WORK_DELAYABLE_DEFINE(g_wake_work, wake_work_fn);

static void wake_timeout_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!g_wake_active) {
		return;		/* the wake completed normally */
	}

	printk("\nwake: no answer after %d s: handing the radio back to RF\n",
	       g_cfg.wake_giveup_s);
	printk("      (if the console should have woken, check it is not flat)\n");

	g_wake_active = false;
	g_wake_request = false;
	switch2_gatt_set_wake_pending(false);
	g_silent = true;
	build_adv_data();

	request_rf_mode();
}

/*
 * Ask for a wake. Called from the RF thread when the chord fires.
 *
 * Does no work itself: entering BLE means telling RF to stop, and the RF thread
 * cannot safely tear down the loop it is running in. Everything real happens on
 * the work queue, for the same reason registration defers its handover.
 */
/*
 * Registration as a temporary excursion, the same shape as the wake macro.
 *
 * Registering needs BLE, using the controller needs RF, and there is one
 * radio. Booting into BLE whenever no console was stored left the Steam
 * Controller dead with no way back. Borrowing the radio and handing it straight
 * back leaves RF working either side of the excursion.
 *
 * This exists so registration can happen where you would naturally do it,
 * plugged into the console with the controller in your hands. Every other way
 * in needs the puck attached to a PC: the config page button reboots into
 * pairing mode, and the `N` console key arms the same for the next boot.
 */
#define REGISTER_GIVEUP_S 120

static void register_timeout_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	/*
	 * Pending means g_pairing, not "no console stored". Testing
	 * g_have_host made this do nothing on a puck that had registered
	 * before, so re-registering and not finishing left it on BLE with the
	 * controller dead until a reboot.
	 */
	if (g_mode != MODE_BLE || !g_pairing) {
		return;		/* registered, or already handed back */
	}

	printk("\n*** registration gave up after %d s: back to RF ***\n",
	       REGISTER_GIVEUP_S);
	g_pairing = false;
	switch2_gatt_set_registering(false);
	request_rf_mode();
}
static K_WORK_DELAYABLE_DEFINE(g_register_timeout_work, register_timeout_fn);

static void register_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	printk("\n*** REGISTER: borrowing the radio. Open Change Grip/Order "
	       "on the console ***\n");

	g_pairing = true;
	switch2_gatt_set_registering(true);
	enter_ble_mode();

	/* Coming home is not optional. RF is dead while BLE owns the radio, so
	 * a registration nobody answers would leave the controller silent.
	 */
	k_work_schedule(&g_register_timeout_work, K_SECONDS(REGISTER_GIVEUP_S));
}
static K_WORK_DELAYABLE_DEFINE(g_register_work, register_work_fn);

void switch2_request_register(void)
{
	if (g_mode != MODE_RF) {
		printk("register: already on BLE, ignoring\n");
		return;
	}
	if (k_work_delayable_is_pending(&g_register_work) ||
	    k_work_delayable_is_pending(&g_register_timeout_work)) {
		printk("register: already registering, ignoring\n");
		return;
	}

	k_work_schedule(&g_register_work, K_NO_WAIT);
}

/* The chime has finished and its last stop has left: now take the radio. */
static void wake_chime_done(void)
{
	k_work_schedule(&g_wake_work, K_NO_WAIT);
}

void switch2_request_wake(int slot)
{
	if (!g_cfg.wake_enabled) {
		printk("wake: disabled in settings, ignoring\n");
		return;
	}
	if (g_mode != MODE_RF) {
		printk("wake: already on BLE, ignoring the macro\n");
		return;
	}
	if (!g_have_host) {
		printk("wake: no console stored, so there is nothing to wake --\n"
		       "      register with one first ('p', then Change Grip/Order)\n");
		return;
	}
	if (g_wake_active || k_work_delayable_is_pending(&g_wake_work) ||
	    puck_melody_busy()) {
		printk("wake: already waking, ignoring\n");
		return;
	}

	/*
	 * The chime plays BEFORE the handover, because once BLE has the radio
	 * nothing reaches the controller until the wake is over. So it delays
	 * the wake by its own length, about half a second, and it is the
	 * chime that schedules the wake once its last stop has left.
	 */
	if (g_cfg.wake_sound && puck_melody_play_wake(slot, wake_chime_done) == 0) {
		return;
	}

	k_work_schedule(&g_wake_work, K_NO_WAIT);
}


void switch2_wake_complete(void)
{
	/* The wake is over. Clear this as well as going off air: advertising
	 * again while still carrying 0x81 wakes the console the moment it
	 * manages to sleep.
	 */
	g_wake_request = false;
	build_adv_data();

	g_silent = true;

	if (g_conn) {
		printk("  releasing the link -> %d\n",
		       bt_conn_disconnect(g_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN));
	}
	if (g_wake_active) {
		/*
		 * The macro drove this wake, so the job is done: give the radio
		 * back and go on being a controller. This hand-off is the shape
		 * of the whole product. BLE is a visit, not a home. A puck
		 * left on BLE is a Steam Controller that has stopped working.
		 */
		g_wake_active = false;
		k_work_cancel_delayable(&g_wake_timeout_work);
		printk("  wake done: handing the radio back to RF\n");
		k_work_schedule(&g_to_rf_work, K_SECONDS(2));
	} else {
		printk("  going off air; press 'x' to advertise again\n");
	}
}

void switch2_registration_complete(void)
{
	if (!g_pairing) {
		return;		/* already in wake mode */
	}

	/*
	 * LEARN which console just registered this puck, from the peer address
	 * of this very connection, and persist it. This is the only moment it
	 * can be known, and without it the wake advert carries the wrong
	 * address.
	 */
	if (g_conn) {
		struct bt_conn_info info;

		if (!bt_conn_get_info(g_conn, &info) && info.type == BT_CONN_TYPE_LE) {
			memcpy(g_console_mac, info.le.dst->a.val, 6);
			g_have_host = true;
			switch2_store_save_host(g_console_mac);
		}
	}

	g_pairing = false;
	build_adv_data();

	/* Registration is done, so stop holding SL+SR. A real Joy-Con presses
	 * nothing once it is registered.
	 */
	switch2_gatt_set_registering(false);

	printk("\n*** REGISTERED: advertising switches to WAKE mode ***\n");

	/*
	 * Registration is the moment the puck becomes an ordinary paired puck,
	 * so it stops living on BLE and hands the radio to RF. The advert was
	 * rebuilt in wake mode above, so the next time BLE is needed, for the
	 * wake macro, it comes back up inviting this console.
	 *
	 * An in-place handover rather than a reboot, which would drop the USB
	 * dongle while the arming scaffolding is still up.
	 */
	k_work_schedule(&g_to_rf_work, K_SECONDS(2));
}

static void set_adv_mode(bool pairing)
{
	printk("\n--- advert mode -> %s ---\n", pairing ? "PAIRING" : "WAKE");
	g_pairing = pairing;
	build_adv_data();

	/* Going back to PAIRING means forgetting the bonded host: the console
	 * has dropped this puck and a fresh registration is needed. Both halves
	 * of the bond go. A stale key answers the next LL_ENC_REQ with something
	 * the console no longer holds, which is worse than having no key at all,
	 * and a stale address keeps inviting a console that is no longer
	 * listening.
	 */
	if (pairing) {
		switch2_store_clear_key();
		switch2_store_clear_host();
		g_have_host = false;
		memset(g_console_mac, 0, sizeof(g_console_mac));
	}

	/* SL+SR only while the console is registering. */
	switch2_gatt_set_registering(pairing);

	/* These keys ask for advertising, including after `x` went silent:
	 * disconnected() only restarts it while not silent.
	 */
	g_silent = false;

	if (g_conn) {
		printk("  disconnecting first -> %d\n",
		       bt_conn_disconnect(g_conn,
					  BT_HCI_ERR_REMOTE_USER_TERM_CONN));
		return;	/* disconnected() restarts advertising */
	}

	printk("  bt_le_adv_stop -> %d\n", bt_le_adv_stop());
	start_adv();
}

/*
 * Report the low-frequency clock directly from the hardware register.
 *
 * The BLE controller cannot schedule a radio event without a running LFCLK, and
 * when it is missing bt_enable() and bt_le_adv_start() BOTH still return
 * success. The failure is completely silent. These boards have no 32.768 kHz
 * crystal, so XTAL never starts; this prints what the hardware actually
 * settled on rather than what Kconfig asked for.
 */
static void report_clock(void)
{
	nrf_clock_lfclk_t src = NRF_CLOCK_LFCLK_RC;
	bool running = nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &src);
	const char *name = (src == NRF_CLOCK_LFCLK_RC)    ? "RC" :
			   (src == NRF_CLOCK_LFCLK_XTAL)  ? "XTAL" :
			   (src == NRF_CLOCK_LFCLK_SYNTH) ? "SYNTH" : "?";

	LOG_INF("LFCLK: running=%d src=%s(%u)   adv_start=%d  keystore=%s",
		running, name, src, g_adv_err,
		switch2_store_ready() ? "mounted" : "NOT MOUNTED");

	/* Retry a failed mount, which also RE-PRINTS the reason. The original
	 * attempt happens before USB has enumerated, so its error never reaches
	 * a terminal and the failure is silent.
	 */
	if (!switch2_store_ready()) {
		switch2_store_init();
	}
}

/*
 * Probe the controller's timing engine directly.
 *
 * For a silent radio where every API returns success. Zephyr's software link
 * layer schedules radio events from its ticker, driven by RTC0 at 32768 Hz. A
 * stopped RTC0 schedules nothing at all, and that looks identical from above:
 * bt_le_adv_start() returns 0 and a sniffer sees nothing.
 *
 * 5 s should advance the counter by ~163840 ticks. It is 24 bits and wraps
 * every 512 s, which the mask handles.
 *
 * RADIO.STATE is sampled for completeness. It reads DISABLED(0) between
 * events even on a healthy link, so only a *changing* RTC0 is evidence.
 */
static void report_ticker(void)
{
	static uint32_t prev;
	static bool have_prev;

	uint32_t now = nrf_rtc_counter_get(NRF_RTC0);
	uint32_t delta = have_prev ? ((now - prev) & 0xFFFFFF) : 0;
	nrf_clock_hfclk_t hfsrc = NRF_CLOCK_HFCLK_LOW_ACCURACY;
	bool hf = nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &hfsrc);

	/*
	 * A stopped ticker is only news while BLE owns the radio. In RF mode the
	 * controller is torn down and RTC0 correctly stops, so flagging it there
	 * printed "TICKER IS DEAD" every five seconds on a healthy puck and
	 * trained everyone to ignore the one line that matters.
	 */
	printk("RTC0=%u delta=%u (expect ~163840)%s  RADIO.STATE=%u  "
	       "HFCLK running=%d src=%u\n",
	       now, delta,
	       (have_prev && delta == 0 && g_mode == MODE_BLE)
		       ? "  <<< TICKER IS DEAD" : "",
	       (unsigned)nrf_radio_state_get(NRF_RADIO), hf, (unsigned)hfsrc);

	prev = now;
	have_prev = true;
}

static void report_interval(struct bt_conn *conn, const char *when)
{
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) || info.type != BT_CONN_TYPE_LE) {
		return;
	}

	/* interval is in 1.25 ms units. 4 == 5.00 ms == the whole point. */
	LOG_INF("%s: interval=%u (%u.%02u ms) latency=%u timeout=%u ms%s",
		when, info.le.interval,
		(info.le.interval * 125) / 100, (info.le.interval * 125) % 100,
		info.le.latency, info.le.timeout * 10,
		info.le.interval < 6 ? "   <<< SUB-SPEC INTERVAL ACCEPTED" : "");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed (0x%02x)", err);
		start_adv();
		return;
	}

	/*
	 * Log WHO connected, not just the interval.
	 *
	 * The console is the address stored at registration. Anything else is a
	 * PC or phone that has seen "Joy-Con 2 (R)" before and reconnects on
	 * sight, and it does real damage: a peripheral in a connection is not
	 * advertising, so a stray central quietly takes the puck off air and the
	 * console can never reach it. That is how this build once looked silent
	 * on a sniffer while advertising had started perfectly well.
	 *
	 * Only the address's last octet is printed. That is enough to match it
	 * against the bonded console shown at boot, or to tell two consoles
	 * apart, without writing a console's full address into a log that may
	 * end up in a bug report.
	 */
	{
		struct bt_conn_info info;

		if (!bt_conn_get_info(conn, &info)) {
			printk("\n*** CONNECTED to ..:%02X ***\n",
			       info.le.dst->a.val[0]);
		} else {
			printk("\n*** CONNECTED ***\n");
		}
	}

	g_conn = bt_conn_ref(conn);
	switch2_gatt_connected(conn);

	LOG_INF("*** CONNECTED ***");
	report_interval(conn, "on connect");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	/* 0x08 = supervision timeout, the link went deaf. 0x13 = the console
	 * hung up deliberately. They mean very different things.
	 */
	printk("\n*** DISCONNECTED reason=0x%02x ***\n", reason);
	LOG_INF("*** DISCONNECTED reason=0x%02x ***", reason);

	switch2_gatt_disconnected();

	if (g_conn) {
		bt_conn_unref(g_conn);
		g_conn = NULL;
	}

	/* Not while deliberately off air. A handover or a finished wake is
	 * waiting for this link to go, and a fresh connectable advert would take
	 * the slot straight back, or let the console reconnect.
	 */
	if (!g_silent) {
		start_adv();
	}
}

static void param_updated(struct bt_conn *conn, uint16_t interval,
			  uint16_t latency, uint16_t timeout)
{
	LOG_INF("params updated: interval=%u (%u.%02u ms) latency=%u timeout=%u ms",
		interval, (interval * 125) / 100, (interval * 125) % 100,
		latency, timeout * 10);
}

/*
 * The console pushes LL_CONNECTION_UPDATE_IND to 5 ms on connections that start
 * at 15 ms. Answering LL_UNKNOWN_RSP gets the link dropped ~11.7 s later, so
 * accepting it is the other half of the 5 ms work.
 */
static bool param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("param request: interval %u-%u latency=%u timeout=%u: ACCEPTING",
		param->interval_min, param->interval_max,
		param->latency, param->timeout);
	return true;
}

/*
 * DID THE LINK ACTUALLY ENCRYPT?
 *
 * Measured here rather than inferred from the arrival of phase-6 commands,
 * which only shows the console's ordering, not this stack's behaviour.
 *
 * The console's LL_ENC_REQ carries EDIV=0/Rand=0 and is answered from the LTK
 * injected on the PAIR/LTK1 write. If that answer is wrong, the console keeps
 * no usable bond, and the next reconnect arrives with no handshake and no
 * LL_ENC_REQ at all.
 *
 * err != 0 here is the useful case: encryption was attempted and failed, so
 * the key did not match.
 */
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	ARG_UNUSED(conn);

	printk("\n*** SECURITY level=%d err=%d %s ***\n", (int)level, (int)err,
	       err ? "<<< ENCRYPTION FAILED" : "(link is encrypted)");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = param_req,
	.le_param_updated = param_updated,
	.security_changed = security_changed,
};

int main(void)
{
	int err;
	/*
	 * Storage FIRST, before USB and before anything that prints. The
	 * bonds live in NVS and the USB layer reports them to Steam as soon as
	 * it is up, so the mount has to happen first. Nothing printed here
	 * reaches a terminal, because USB is not up yet.
	 */
	switch2_store_init();
	puck_trace_boot_report();
	puck_trace_stage(TR_STORE);
	puck_settings_init();

	/*
	 * USB next, and before anything else that wants to print: this builds
	 * the device carrying BOTH the four dongle HID interfaces and the CDC
	 * console. Zephyr's auto-init is disabled (see prj.conf), so nothing
	 * else brings the console up. If this fails there is no console at
	 * all, and recovery is a physical double-tap into the bootloader.
	 */
	puck_usb_init();
	puck_trace_stage(TR_USB_UP);


	/* Must be called before bt_enable(). */
	bt_ctlr_set_public_addr(puck_addr());

	err = bt_enable(NULL);
	puck_trace_stage(TR_BT_UP);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return 0;
	}

	err = switch2_gatt_init();
	if (err) {
		LOG_ERR("GATT registration failed (%d)", err);
		return 0;
	}

	LOG_INF("=== Pair-Raught ===");
	LOG_INF("waiting for the console; PASS is a connection at interval=4");

	/*
	 * Do NOT advertise yet. The CDC console has not enumerated this early
	 * and the log thread is held for 4 s, so the backlog overflows and the
	 * first failure on air is thrown away with it, HCI warnings included.
	 * Waiting puts every line of it on screen.
	 */
	k_sleep(K_SECONDS(8));
	printk("\n--- console up; starting the radio in %s mode ---\n",
	       g_pairing ? "PAIRING" : "WAKE");
	printk("keys: p=pairing advert  w=wake advert  g=dump GATT  "
	       "h=HOME/wake  b=bootloader drive\n");

	/* Print the table BEFORE going on air. Zephyr numbers its attributes
	 * differently from the SoftDevice, so whether the report channel landed
	 * on the real device's 0x000E/0x000F/0x0010 has to be measured. If it is
	 * misaligned the console's hardcoded writes go to the wrong attribute,
	 * and that is invisible from the console's side.
	 */
	switch2_gatt_dump();
	switch2_gatt_check();

	/* Which console to invite, if any. Empty until a registration. */
	g_have_host = switch2_store_load_host(g_console_mac);

	/*
	 * WHICH RADIO OWNS THE BOOT: RF, unless registration was asked for.
	 * BLE is never the default, stored console or not.
	 *
	 * Booting to BLE whenever no console was stored stranded two ordinary
	 * cases. A puck used only as a Steam dongle on a PC has no console and
	 * never will, so it sat on BLE holding the radio with every bonded
	 * controller unable to connect. And bonding a controller before
	 * registering a console gave a bond that read back perfectly and a
	 * controller that never linked, because the bond is written over USB
	 * but connecting needs the radio BLE was holding.
	 *
	 * Registration is requested instead: the config page's "Pair with a
	 * console" button, or `N` here and a reboot, sets the re-pair flag, and
	 * the next boot acts on it in either USB mode. Bonding a controller and
	 * registering a console are then independent, in whichever order suits.
	 */
	g_repair_req = switch2_store_take_repair();
	g_pairing = g_repair_req;
	if (g_repair_req) {
		printk("registration requested: PAIRING mode "
		       "(register from Change Grip/Order)\n");
	} else if (g_have_host) {
		printk("bonded console ..:%02X: booting to RF\n", g_console_mac[0]);
	} else {
		printk("no bonded console: booting to RF, ask for "
		       "registration when you want it\n");
	}

	/* Wake mode means already registered, so no presses. Pairing mode means
	 * the console has yet to register the puck, which needs the SL+SR slot
	 * confirmation.
	 */
	puck_trace_stage(TR_GATT);
	switch2_gatt_set_registering(g_pairing);

	build_adv_data();
	/*
	 * A puck that already knows its console hands the radio straight to RF,
	 * so a searching controller is picked up immediately. BLE comes back
	 * only for the wake macro or a requested re-pair.
	 *
	 * PRO CONTROLLER mode has no say in it. The puck is a wired pad in the
	 * console's USB port and RF is the only way input reaches it, so
	 * sitting on BLE would enumerate a perfect pad with every button dead.
	 */
	puck_trace_stage(TR_BOOTMODE);
	puck_trace_report();	/* console exists by now */

	if (g_repair_req) {
		/*
		 * Registration asked for, in either USB mode: advertise for
		 * pairing now, and come home on the same timeout as the
		 * register chord.
		 *
		 * Pro Controller mode used to skip this and hand straight to
		 * RF, consuming the request unused. A puck with a controller
		 * bonded always boots as a Pro Controller, dongle mode lasting
		 * one boot, so "Pair with a console" rebooted into that branch
		 * and did nothing.
		 */
		start_adv();
		k_work_schedule(&g_register_timeout_work,
				K_SECONDS(REGISTER_GIVEUP_S));
	} else if (puck_usb_needs_rf()) {
		/*
		 * PRO CONTROLLER always ends up on RF, but not from here.
		 * Handing over inline at boot killed the CDC console: the
		 * device enumerated and then the port failed to open with "the
		 * semaphore timeout period has expired". Deferred like every
		 * other mode transition, so USB finishes coming up first.
		 */
		printk("USB mode is PRO CONTROLLER: handing to RF in %d s\n",
		       PRO_RF_DELAY_S);
		k_work_schedule(&g_to_rf_work, K_SECONDS(PRO_RF_DELAY_S));
	} else {
		enter_rf_mode();
	}

	/*
	 * Heartbeat, so a silent log is distinguishable from a dead board.
	 * While advertising is failing, retry it here: each retry re-prints the
	 * conn-pool census and the host's own error, so the state is visible
	 * live rather than reconstructed from a lost backlog.
	 */
	while (1) {
		k_sleep(K_SECONDS(5));
		report_clock();
		report_ticker();
		conn_objects("heartbeat");

		if (g_adv_err != 0 && !g_silent) {
			printk("\n--- retry %u ---\n", (unsigned)g_adv_calls + 1);
			start_adv();
		}
	}

	return 0;
}

/* Has a console ever registered with this puck? The config surface needs to
 * say so without a serial terminal.
 */
bool switch2_have_host(void)
{
	return g_have_host;
}
