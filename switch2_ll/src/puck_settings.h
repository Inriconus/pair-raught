/*
 * Everything about the puck that a person might reasonably want to change.
 *
 * ONE VERSIONED STRUCT, ONE NVS ID. Settings arrive in batches over time, and a
 * cell-per-setting scheme means a growing pile of ids, partial writes, and no
 * way to tell a missing setting from a zero one. A single record with a version
 * byte upgrades cleanly: an older struct is recognised, its known fields kept,
 * and the rest filled from defaults.
 *
 * These exist because the alternative is a serial console, which this project's
 * own notes call a developer affordance rather than a product one. Someone
 * building a puck for another person needs to set the wake behaviour and put it
 * in pairing mode without a terminal.
 *
 * Defaults are the values the firmware shipped with as constants, so an
 * unconfigured puck behaves exactly as before.
 */

#ifndef PUCK_SETTINGS_H
#define PUCK_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 3 added wake_sound, 4 chime_volume and 5 mode_sound, all carved out of
 * reserved[] so the record stays the same size, which it must: a record is only
 * read back if its length matches. An older record reads 0 in a field added
 * since, so loading one sets those to their defaults rather than trusting the
 * zero.
 */
#define PUCK_SETTINGS_VERSION 5

struct puck_settings {
	uint8_t version;

	/* --- waking a sleeping console ------------------------------------ */
	uint8_t wake_enabled;		/* 0 = the chord does nothing */
	uint8_t unlock_enabled;		/* 0 = wake, but do not press anything */
	uint16_t unlock_mask;		/* which button clears the lock screen */
	uint8_t unlock_presses;		/* how many times to press it */
	uint8_t unlock_delay;		/* reports to wait for the lock screen */
	uint8_t unlock_down;		/* reports a press is held */
	uint8_t unlock_up;		/* reports between presses */
	uint8_t unlock_settle;		/* reports to hold before letting go */
	uint8_t wake_giveup_s;		/* give up and return to RF after this */

	/* --- chords ------------------------------------------------------- */
	uint16_t chord_hold_ms;		/* power-off and mode chords */
	uint16_t wake_hold_ms;		/* the QAM wake button */

	/* --- trackpad mouse ----------------------------------------------- */
	uint8_t mouse_div;		/* pad units per pixel; lower = faster */
	uint8_t mouse_fric;		/* % glide kept per pass after release */

	/* --- haptics ------------------------------------------------------ */
	uint8_t rumble_enabled;
	uint8_t rumble_scale_pct;	/* 200 = the Arduino build's default */

	/*
	 * --- how the pad LOOKS on the console -----------------------------
	 *
	 * The console reads these during its handshake, from the SPI block at
	 * 0x6050, and the device-info reply points it there. Four RGB triples:
	 * body, buttons, left grip, right grip.
	 */
	uint8_t col_body[3];
	uint8_t col_buttons[3];
	uint8_t col_grip_l[3];
	uint8_t col_grip_r[3];

	/*
	 * --- extra buttons ------------------------------------------------
	 *
	 * The four back paddles and QAM have no Pro Controller equivalent, so
	 * each carries a target instead: an index into PAD_TARGETS (0 = leave
	 * it unmapped). This is what the Arduino build resolves through
	 * per-user remap codes; here the choice is simply stored.
	 */
	uint8_t map_l4, map_r4, map_l5, map_r5, map_qam;

	/*
	 * --- chimes, versions 3 to 5 --------------------------------------
	 *
	 * Their notes are records of their own: see puck_melody.h.
	 */
	uint8_t wake_sound;		/* 1 = the wake chime before a wake */
	uint8_t chime_volume;		/* 0..100 for every chime, 100 the
					 * loudest that played clean */
	uint8_t mode_sound;		/* 1 = a chime before the mode chord
					 * reboots into another identity */

	uint8_t reserved[3];
};

/* The live settings. Read freely; change through puck_settings_save(). */
extern struct puck_settings g_cfg;

/*
 * Load from NVS into g_cfg, falling back to defaults for anything a older
 * version did not have. Call once at boot, before anything reads g_cfg.
 */
void puck_settings_init(void);

/* Persist g_cfg. Returns 0 on success. */
int puck_settings_save(void);

/* Restore every field to its shipped default (does not save). */
void puck_settings_defaults(struct puck_settings *s);

/* Print the current settings. Bound to console key 'c'. */
void puck_settings_dump(void);

/*
 * Handle one .$. command line (without the .$.). Replies are wrapped in
 * <<<CFG / CFG>>> markers so a UI can find them among the running firmware.s
 * log output, which never stops.
 */
void puck_settings_command(char *line);

/*
 * The Pro Controller button bit a mapping target selects, or 0 for "none".
 * Targets are indices into the table the config surface offers, so the
 * firmware stays the single authority on what a mapping may be set to.
 */
uint32_t puck_settings_pad_bit(uint8_t target);

#endif /* PUCK_SETTINGS_H */
