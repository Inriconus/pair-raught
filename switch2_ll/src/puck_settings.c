/*
 * Persisted settings. See puck_settings.h for why this is one struct.
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "puck_settings.h"
#include "switch2_store.h"
#include <zephyr/sys/reboot.h>
#include "puck_hid.h"
#include "puck_usb.h"
#include "rf_link.h"
#include "switch_pro_usb.h"
#include "switch2_gatt.h"
#include "puck_melody.h"

struct puck_settings g_cfg;

void puck_settings_defaults(struct puck_settings *s)
{
	memset(s, 0, sizeof(*s));
	s->version = PUCK_SETTINGS_VERSION;

	/*
	 * The wake sequence, measured rather than chosen. The lock screen wants
	 * the SAME button pressed repeatedly, B (0x0001) worked first try,
	 * and setting every bit at once does NOT work, because that reads as
	 * many different buttons at once. The delay matters as much: pressing
	 * immediately achieved nothing, because the console had barely woken
	 * and the lock screen was not up yet.
	 */
	s->wake_enabled = 1;
	s->unlock_enabled = 1;
	s->unlock_mask = 0x0001;	/* B */
	s->unlock_presses = 4;
	s->unlock_delay = 60;		/* ~1.8 s */
	s->unlock_down = 6;		/* ~180 ms, a human-length press */
	s->unlock_up = 10;		/* ~300 ms between presses */
	s->unlock_settle = 60;		/* ~1.8 s before releasing the link */
	s->wake_giveup_s = 30;

	/* The chime before a wake. On, because a hold that does something
	 * silent for seconds is indistinguishable from a hold that did nothing.
	 */
	s->wake_sound = 1;

	/* 100 is the loudest a chime is allowed, tuned by ear; see
	 * CHIME_GAIN_MAX in puck_melody.c.
	 */
	s->chime_volume = 100;

	/* A chime before the mode chord reboots, for the same reason as the
	 * wake's: the reboot itself is silent, and the controller drops out.
	 */
	s->mode_sound = 1;

	/* Chords. The wake is a single button so it needs less of a hold than
	 * a two-handed combination, but not none, waking takes the radio off
	 * RF for seconds, and an accidental brush would drop the controller.
	 */
	s->chord_hold_ms = 2000;
	s->wake_hold_ms = 1000;

	/* Mouse. Lower divisor is faster; friction is the coast after release
	 * only, since the cursor tracks one-to-one while a finger is down.
	 */
	s->mouse_div = 4;
	s->mouse_fric = 85;

	/* Haptics: the decoded amplitude doubled, which is what the Arduino
	 * build shipped as its default rumble strength.
	 */
	s->rumble_enabled = 1;
	s->rumble_scale_pct = 200;

	/*
	 * How the pad looks on the console. These are the values a real Pro
	 * Controller reports: a near-black body with light grey buttons.
	 */
	s->col_body[0] = s->col_body[1] = s->col_body[2] = 0x32;
	s->col_buttons[0] = s->col_buttons[1] = s->col_buttons[2] = 0xE6;
	s->col_grip_l[0] = s->col_grip_l[1] = s->col_grip_l[2] = 0x32;
	s->col_grip_r[0] = s->col_grip_r[1] = s->col_grip_r[2] = 0x32;

	/* Extra buttons start unmapped, which is what they have always been. */
	s->map_l4 = s->map_r4 = s->map_l5 = s->map_r5 = s->map_qam = 0;
}

void puck_settings_init(void)
{
	struct puck_settings stored;

	puck_settings_defaults(&g_cfg);

	/* The chime's notes are their own record, loaded here so everything
	 * the config page edits is ready at the same point in boot.
	 */
	puck_chime_init();

	if (!switch2_store_load_settings(&stored, sizeof(stored))) {
		printk("cfg: no stored settings: using defaults\n");
		return;
	}

	if (stored.version == 0 || stored.version > PUCK_SETTINGS_VERSION) {
		/*
		 * Either blank flash or a record from a FUTURE firmware. Both
		 * mean "do not trust these fields"; defaults are already in
		 * place, and overwriting the record now would destroy settings
		 * that a newer build could still read.
		 */
		printk("cfg: stored settings version %u is not usable: "
		       "using defaults\n", stored.version);
		return;
	}

	/*
	 * Take it wholesale, then upgrade what an OLDER version could not have
	 * set. The record keeps its size across versions, so an older one
	 * reads zero in any field added since, and zero is not always the
	 * default.
	 */
	g_cfg = stored;
	if (stored.version < 3) {
		g_cfg.wake_sound = 1;
	}
	if (stored.version < 4) {
		g_cfg.chime_volume = 100;
	}
	if (stored.version < 5) {
		g_cfg.mode_sound = 1;
	}
	g_cfg.version = PUCK_SETTINGS_VERSION;
	printk("cfg: settings loaded (version %u)\n", stored.version);
}

int puck_settings_save(void)
{
	g_cfg.version = PUCK_SETTINGS_VERSION;
	return switch2_store_save_settings(&g_cfg, sizeof(g_cfg));
}

void puck_settings_dump(void)
{
	printk("\n--- settings (version %u) ---\n", g_cfg.version);
	printk("  wake        : %s  button=0x%04X  presses=%u\n",
	       g_cfg.wake_enabled ? "enabled" : "DISABLED", g_cfg.unlock_mask,
	       g_cfg.unlock_presses);
	printk("  chimes      : wake %s, mode switch %s, volume=%u (gain %d)\n",
	       g_cfg.wake_sound ? "on" : "off", g_cfg.mode_sound ? "on" : "off",
	       g_cfg.chime_volume, puck_chime_gain());
	for (int id = 0; id < PUCK_CHIME_COUNT; id++) {
		printk("  chime %s: %u notes\n", puck_chime_name(id),
		       puck_chime_get(id)->count);
	}
	printk("  wake timing : delay=%u down=%u up=%u settle=%u reports, "
	       "give up after %us\n",
	       g_cfg.unlock_delay, g_cfg.unlock_down, g_cfg.unlock_up,
	       g_cfg.unlock_settle, g_cfg.wake_giveup_s);
	printk("  chords      : hold=%ums  wake hold=%ums\n",
	       g_cfg.chord_hold_ms, g_cfg.wake_hold_ms);
	printk("  mouse       : div=%u (lower is faster)  friction=%u%%\n",
	       g_cfg.mouse_div, g_cfg.mouse_fric);
	printk("  rumble      : %u%% of decoded amplitude\n",
	       g_cfg.rumble_scale_pct);
}

/*
 * THE MACHINE-READABLE SIDE.
 *
 * The console is shared with a firmware printing RF counters and heartbeats
 * constantly, so a UI cannot just read whatever comes back: the human dump
 * arrives interleaved with log lines and split across chunk boundaries. So
 * every reply is wrapped in markers and each value is one key=value line, and a
 * reader can find its answer in the noise without the firmware going quiet.
 *
 * Commands are lines beginning with '$', which leaves the single-keypress
 * shortcuts untouched:
 *
 *   $get              dump all settings as key=value
 *   $fields           dump each setting's range and default
 *   $status           mode, bonds, console, USB identity
 *   $set <key> <val>  change one setting, applied and saved immediately
 *   $defaults         restore shipped defaults and save
 *   $action <what>    pair, factory, reboot, usbmode
 *   $chime <what>     the wake chime's notes; see cfg_chime()
 *
 * Values are decimal, or hex with a 0x prefix.
 */
#define CFG_BEGIN "<<<CFG"
#define CFG_END   "CFG>>>"
/*
 * What an extra button can be mapped to.
 *
 * The four back paddles and QAM have no Pro Controller equivalent, so each one
 * carries an index into this table. Index 0 is deliberately "unmapped", that
 * is what they have always been, and an arbitrary default would be worse than
 * nothing. The values are the Pro Controller's own button bits.
 */
static const struct {
	const char *name;
	uint32_t bit;
} PAD_TARGETS[] = {
	{ "none",    0 },
	{ "a",       1u << 3 },
	{ "b",       1u << 2 },
	{ "x",       1u << 1 },
	{ "y",       1u << 0 },
	{ "l",       1u << 22 },
	{ "r",       1u << 6 },
	{ "zl",      1u << 23 },
	{ "zr",      1u << 7 },
	{ "minus",   1u << 8 },
	{ "plus",    1u << 9 },
	{ "lstick",  1u << 11 },
	{ "rstick",  1u << 10 },
	{ "home",    1u << 12 },
	{ "capture", 1u << 13 },
	{ "up",      1u << 17 },
	{ "down",    1u << 16 },
	{ "left",    1u << 19 },
	{ "right",   1u << 18 },
};

#define PAD_TARGET_MAX (ARRAY_SIZE(PAD_TARGETS) - 1)

uint32_t puck_settings_pad_bit(uint8_t target)
{
	return target <= PAD_TARGET_MAX ? PAD_TARGETS[target].bit : 0;
}


struct cfg_field {
	const char *key;
	void *val;
	uint8_t bytes;		/* 1 or 2 */
	uint16_t min, max;
};

/* One table drives get, set and validation, adding a setting here is enough. */
static const struct cfg_field CFG_FIELDS[] = {
	{ "wake.enabled",     &g_cfg.wake_enabled,     1, 0, 1 },
	{ "wake.unlock",      &g_cfg.unlock_enabled,   1, 0, 1 },
	{ "wake.sound",       &g_cfg.wake_sound,       1, 0, 1 },
	{ "chime.volume",     &g_cfg.chime_volume,     1, 0, 100 },
	{ "chime.mode",       &g_cfg.mode_sound,       1, 0, 1 },
	{ "wake.button",      &g_cfg.unlock_mask,      2, 0, 0xFFFF },
	{ "wake.presses",     &g_cfg.unlock_presses,   1, 1, 20 },
	{ "wake.delay",       &g_cfg.unlock_delay,     1, 0, 255 },
	{ "wake.press_time",  &g_cfg.unlock_down,      1, 1, 255 },
	{ "wake.gap",         &g_cfg.unlock_up,        1, 1, 255 },
	{ "wake.settle",      &g_cfg.unlock_settle,    1, 0, 255 },
	{ "wake.give_up_s",   &g_cfg.wake_giveup_s,    1, 5, 240 },
	{ "wake.hold_ms",     &g_cfg.wake_hold_ms,     2, 100, 10000 },
	{ "chord.hold_ms",    &g_cfg.chord_hold_ms,    2, 250, 10000 },
	{ "mouse.speed",      &g_cfg.mouse_div,        1, 1, 255 },
	{ "mouse.glide",      &g_cfg.mouse_fric,       1, 0, 99 },
	{ "rumble.enabled",   &g_cfg.rumble_enabled,   1, 0, 1 },
	{ "rumble.strength",  &g_cfg.rumble_scale_pct, 1, 0, 255 },
	{ "color.body.r",     &g_cfg.col_body[0],      1, 0, 255 },
	{ "color.body.g",     &g_cfg.col_body[1],      1, 0, 255 },
	{ "color.body.b",     &g_cfg.col_body[2],      1, 0, 255 },
	{ "color.buttons.r",  &g_cfg.col_buttons[0],   1, 0, 255 },
	{ "color.buttons.g",  &g_cfg.col_buttons[1],   1, 0, 255 },
	{ "color.buttons.b",  &g_cfg.col_buttons[2],   1, 0, 255 },
	{ "color.gripl.r",    &g_cfg.col_grip_l[0],    1, 0, 255 },
	{ "color.gripl.g",    &g_cfg.col_grip_l[1],    1, 0, 255 },
	{ "color.gripl.b",    &g_cfg.col_grip_l[2],    1, 0, 255 },
	{ "color.gripr.r",    &g_cfg.col_grip_r[0],    1, 0, 255 },
	{ "color.gripr.g",    &g_cfg.col_grip_r[1],    1, 0, 255 },
	{ "color.gripr.b",    &g_cfg.col_grip_r[2],    1, 0, 255 },
	{ "map.l4",           &g_cfg.map_l4,           1, 0, PAD_TARGET_MAX },
	{ "map.r4",           &g_cfg.map_r4,           1, 0, PAD_TARGET_MAX },
	{ "map.l5",           &g_cfg.map_l5,           1, 0, PAD_TARGET_MAX },
	{ "map.r5",           &g_cfg.map_r5,           1, 0, PAD_TARGET_MAX },
	{ "map.qam",          &g_cfg.map_qam,          1, 0, PAD_TARGET_MAX },
};

static uint16_t field_get(const struct cfg_field *f)
{
	return f->bytes == 2 ? *(uint16_t *)f->val : *(uint8_t *)f->val;
}

static void field_set(const struct cfg_field *f, uint16_t v)
{
	if (f->bytes == 2) {
		*(uint16_t *)f->val = v;
	} else {
		*(uint8_t *)f->val = (uint8_t)v;
	}
}

/*
 * Describe every field and every mapping target.
 *
 * The config screen needs ranges to build its sliders and names to fill its
 * mapping menus. It could carry its own copy of both, but then a firmware that
 * moved a limit would be silently mismatched with the page in front of it,
 * so the firmware says what it accepts and the page renders whatever it is
 * told. Fetched once; $get carries only values.
 */
static void cfg_emit_fields(void)
{
	printk("%s\n", CFG_BEGIN);
	for (size_t i = 0; i < ARRAY_SIZE(CFG_FIELDS); i++) {
		printk("field=%s,%u,%u\n", CFG_FIELDS[i].key,
		       CFG_FIELDS[i].min, CFG_FIELDS[i].max);
	}
	for (size_t i = 0; i < ARRAY_SIZE(PAD_TARGETS); i++) {
		printk("target=%u,%s\n", (unsigned)i, PAD_TARGETS[i].name);
	}
	printk("%s\n", CFG_END);
}

static void cfg_emit_all(void)
{
	printk("%s\n", CFG_BEGIN);
	printk("version=%u\n", g_cfg.version);
	for (size_t i = 0; i < ARRAY_SIZE(CFG_FIELDS); i++) {
		printk("%s=%u\n", CFG_FIELDS[i].key,
		       field_get(&CFG_FIELDS[i]));
	}
	printk("%s\n", CFG_END);
}

/*
 * Restore and report in ONE block. Two blocks for one command would leave a
 * reader that pairs replies with commands permanently one behind, every
 * later answer would belong to the previous question.
 */
static void cfg_restore_defaults(void)
{
	puck_settings_defaults(&g_cfg);

	printk("%s\n", CFG_BEGIN);
	if (puck_settings_save()) {
		printk("error=could not save\n");
	} else {
		printk("ok=defaults\n");
	}
	printk("version=%u\n", g_cfg.version);
	for (size_t i = 0; i < ARRAY_SIZE(CFG_FIELDS); i++) {
		printk("%s=%u\n", CFG_FIELDS[i].key,
		       field_get(&CFG_FIELDS[i]));
	}
	printk("%s\n", CFG_END);
}

/* Parse a decimal or 0x-prefixed value. Returns false if it is not a number. */
static bool parse_val(const char *s, uint32_t *out)
{
	uint32_t v = 0;
	int base = 10;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		base = 16;
		s += 2;
		if (!*s) {
			return false;
		}
	}
	for (; *s; s++) {
		int d;

		if (*s >= '0' && *s <= '9') {
			d = *s - '0';
		} else if (base == 16 && *s >= 'a' && *s <= 'f') {
			d = *s - 'a' + 10;
		} else if (base == 16 && *s >= 'A' && *s <= 'F') {
			d = *s - 'A' + 10;
		} else {
			return false;
		}
		v = v * (uint32_t)base + (uint32_t)d;
		if (v > 0xFFFF) {
			return false;
		}
	}
	*out = v;
	return true;
}

static void cfg_set(char *args)
{
	char *key = args;
	char *val;
	uint32_t v;

	while (*key == ' ') {
		key++;
	}
	val = key;
	while (*val && *val != ' ') {
		val++;
	}
	if (!*val) {
		printk("%s\nerror=usage: $set <key> <value>\n%s\n", CFG_BEGIN,
		       CFG_END);
		return;
	}
	*val++ = '\0';
	while (*val == ' ') {
		val++;
	}

	if (!parse_val(val, &v)) {
		printk("%s\nerror=not a number: %s\n%s\n", CFG_BEGIN, val,
		       CFG_END);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(CFG_FIELDS); i++) {
		const struct cfg_field *f = &CFG_FIELDS[i];

		if (strcmp(key, f->key) != 0) {
			continue;
		}
		if (v < f->min || v > f->max) {
			printk("%s\nerror=%s must be %u..%u\n%s\n", CFG_BEGIN,
			       f->key, f->min, f->max, CFG_END);
			return;
		}
		field_set(f, (uint16_t)v);
		if (puck_settings_save()) {
			printk("%s\nerror=could not save\n%s\n", CFG_BEGIN,
			       CFG_END);
			return;
		}
		printk("%s\nok=%s\n%s=%u\n%s\n", CFG_BEGIN, f->key, f->key,
		       field_get(f), CFG_END);
		return;
	}

	printk("%s\nerror=unknown key: %s\n%s\n", CFG_BEGIN, key, CFG_END);
}

/*
 * READ-ONLY STATUS. Everything a person would otherwise need the serial console
 * to find out: which controller is bonded and whether it is actually answering,
 * whether a console has been registered, what identity this boot came up as,
 * and whether a motion calibration survived.
 */
static void cfg_status(void)
{
	const uint8_t *bond = puck_hid_bond(0);

	printk("%s\n", CFG_BEGIN);
	printk("mode=%s\n", puck_usb_is_pro() ? "pro" : "dongle");
	if (bond) {
		printk("controller=%.16s\n", (const char *)bond + 8);
		printk("controller_live=%u\n", rf_link_slot_conn(0) ? 1 : 0);
	} else {
		printk("controller=\n");
		printk("controller_live=0\n");
	}
	printk("console_paired=%u\n", switch2_have_host() ? 1 : 0);
	printk("motion_cal=%u\n", switch_pro_usb_cal_bytes());
	printk("%s\n", CFG_END);
}

/*
 * ACTIONS. Things that DO something rather than store a value.
 *
 * `pair` is the one the goal notes ask for by name: someone building a puck for
 * another person needs to put it into pairing mode and register it with THEIR
 * console, without a terminal. It arms a sticky flag and reboots, because BLE
 * pairing mode is a boot-time decision, and the flag is consumed by the boot
 * that acts on it, so an abandoned attempt cannot strand the puck off RF.
 */
static void cfg_action(const char *what)
{
	if (!strcmp(what, "pair")) {
		if (switch2_store_request_repair()) {
			printk("%s\nerror=could not arm pairing\n%s\n",
			       CFG_BEGIN, CFG_END);
			return;
		}
		printk("%s\nok=pairing armed, rebooting\n%s\n", CFG_BEGIN,
		       CFG_END);
		k_sleep(K_MSEC(250));
		sys_reboot(SYS_REBOOT_COLD);
	} else if (!strcmp(what, "factory")) {
		/*
		 * Everything the puck remembers, gone. Deliberately not
		 * reachable by accident: the config page asks first, and there
		 * is no chord for it.
		 */
		if (switch2_store_erase_all()) {
			printk("%s\nerror=could not erase\n%s\n", CFG_BEGIN,
			       CFG_END);
			return;
		}
		printk("%s\nok=erased, rebooting\n%s\n", CFG_BEGIN, CFG_END);
		k_sleep(K_MSEC(250));
		sys_reboot(SYS_REBOOT_COLD);
	} else if (!strcmp(what, "reboot")) {
		printk("%s\nok=rebooting\n%s\n", CFG_BEGIN, CFG_END);
		k_sleep(K_MSEC(250));
		sys_reboot(SYS_REBOOT_COLD);
	} else if (!strcmp(what, "usbmode")) {
		uint8_t next = puck_usb_is_pro() ? PUCK_USB_MODE_DONGLE
						 : PUCK_USB_MODE_PRO;

		if (switch2_store_save_usb_mode(next)) {
			printk("%s\nerror=could not save mode\n%s\n", CFG_BEGIN,
			       CFG_END);
			return;
		}
		printk("%s\nok=mode %s, rebooting\n%s\n", CFG_BEGIN,
		       next == PUCK_USB_MODE_PRO ? "pro" : "dongle", CFG_END);
		k_sleep(K_MSEC(250));
		sys_reboot(SYS_REBOOT_COLD);
	} else {
		printk("%s\nerror=unknown action: %s\n%s\n", CFG_BEGIN, what,
		       CFG_END);
	}
}

/*
 * THE CHIMES' notes. A command of their own because each chime is a record of
 * its own, and because a melody is saved whole: a note-at-a-time $set would
 * store every half-edited melody on the way to the finished one.
 *
 *   $chime names                  names=<name>,<name>,...
 *   $chime <name> get             limits, channels=<haptic>,... (the default
 *                                 first), then note=hz,ms,haptic per note
 *   $chime <name> set <notes>     check, save and use it; replies as get does
 *   $chime <name> test <notes>    check and play it now, without saving
 *   $chime <name> defaults        restore that chime's default and save it
 *
 * <notes> is hz:ms:haptic separated by spaces; a note without :haptic plays on
 * the default. hz 0 is a rest. The names, limits and haptics come from the
 * firmware, so the page builds its editor from what this firmware has and
 * accepts, as it does sliders.
 */
static void cfg_emit_chime(const struct puck_chime *c)
{
	const uint8_t *channels;
	size_t n = puck_chime_channels(&channels);

	printk("max_notes=%u\n", PUCK_CHIME_MAX_NOTES);
	printk("hz_min=%u\nhz_max=%u\n", PUCK_CHIME_HZ_MIN, PUCK_CHIME_HZ_MAX);
	printk("ms_min=%u\nms_max=%u\nrest_ms_min=%u\n", PUCK_CHIME_MS_MIN,
	       PUCK_CHIME_MS_MAX, PUCK_CHIME_REST_MS_MIN);
	printk("total_ms_max=%u\n", PUCK_CHIME_TOTAL_MS_MAX);
	printk("channels=");
	for (size_t i = 0; i < n; i++) {
		printk("%s%u", i ? "," : "", channels[i]);
	}
	printk("\n");
	for (int i = 0; i < c->count; i++) {
		printk("note=%u,%u,%u\n", c->notes[i].hz, c->notes[i].ms,
		       c->notes[i].haptic);
	}
}

static void cfg_chime(const char *args)
{
	/* Static: this runs on the console thread, whose stack is 1 KB. */
	static struct puck_chime c;
	static char err[64];
	static char name[16];
	const char *verb = strchr(args, ' ');
	size_t len = verb ? (size_t)(verb - args) : strlen(args);
	int id;

	if (!strcmp(args, "names")) {
		printk("%s\nnames=", CFG_BEGIN);
		for (int i = 0; i < PUCK_CHIME_COUNT; i++) {
			printk("%s%s", i ? "," : "", puck_chime_name(i));
		}
		printk("\n%s\n", CFG_END);
		return;
	}

	len = MIN(len, sizeof(name) - 1);
	memcpy(name, args, len);
	name[len] = '\0';
	id = puck_chime_lookup(name);
	if (id < 0 || !verb) {
		printk("%s\nerror=unknown chime: %s (see $chime names)\n%s\n",
		       CFG_BEGIN, name, CFG_END);
		return;
	}
	verb++;

	if (!strcmp(verb, "get")) {
		printk("%s\n", CFG_BEGIN);
		cfg_emit_chime(puck_chime_get(id));
		printk("%s\n", CFG_END);
	} else if (!strncmp(verb, "set ", 4) || !strcmp(verb, "defaults")) {
		int ret;

		if (verb[0] == 'd') {
			puck_chime_defaults(id, &c);
		} else if (puck_chime_parse(verb + 4, &c, err, sizeof(err))) {
			printk("%s\nerror=%s\n%s\n", CFG_BEGIN, err, CFG_END);
			return;
		}
		ret = puck_chime_set(id, &c);
		printk("%s\n%s\n", CFG_BEGIN,
		       ret ? "error=could not save the chime" : "ok=chime saved");
		cfg_emit_chime(puck_chime_get(id));
		printk("%s\n", CFG_END);
	} else if (!strncmp(verb, "test ", 5)) {
		int slot = -1;

		if (puck_chime_parse(verb + 5, &c, err, sizeof(err))) {
			printk("%s\nerror=%s\n%s\n", CFG_BEGIN, err, CFG_END);
			return;
		}
		for (int s = 0; s < PUCK_HID_SLOTS; s++) {
			if (rf_link_slot_live(s)) {
				slot = s;
				break;
			}
		}
		if (slot < 0) {
			printk("%s\nerror=no controller connected to play it on\n%s\n",
			       CFG_BEGIN, CFG_END);
		} else if (puck_melody_play(slot, &c, puck_chime_gain(), NULL)) {
			printk("%s\nerror=a chime is already playing\n%s\n",
			       CFG_BEGIN, CFG_END);
		} else {
			printk("%s\nok=playing\n%s\n", CFG_BEGIN, CFG_END);
		}
	} else {
		printk("%s\nerror=unknown chime command: %s\n%s\n", CFG_BEGIN,
		       args, CFG_END);
	}
}

void puck_settings_command(char *line)
{
	if (!strcmp(line, "get")) {
		cfg_emit_all();
	} else if (!strncmp(line, "chime ", 6)) {
		cfg_chime(line + 6);
	} else if (!strcmp(line, "fields")) {
		cfg_emit_fields();
	} else if (!strncmp(line, "set ", 4)) {
		cfg_set(line + 4);
	} else if (!strcmp(line, "status")) {
		cfg_status();
	} else if (!strncmp(line, "action ", 7)) {
		cfg_action(line + 7);
	} else if (!strcmp(line, "defaults")) {
		cfg_restore_defaults();
	} else {
		printk("%s\nerror=unknown command: %s\n%s\n", CFG_BEGIN, line,
		       CFG_END);
	}
}
