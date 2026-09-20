/*
 * Tones on the Steam Controller's haptics, and the chimes the puck plays.
 *
 * The controller plays a pitch on any of its four haptics when sent output
 * report 0x83, the command SteamHapticsSinger uses to play notes on a Steam
 * Controller (2026) (BSD 3-clause, github.com/CrazyCritic89/SteamHapticsSinger).
 * The puck relays it over RF like any other haptic. Tones on a trackpad and on
 * a rumble motor have both been heard through it, at the same pitch and with a
 * different tonal quality.
 */

#ifndef PUCK_MELODY_H
#define PUCK_MELODY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Which haptic plays a tone. There is no haptic 2. */
#define PUCK_HAPTIC_PAD_L	0
#define PUCK_HAPTIC_PAD_R	1
#define PUCK_HAPTIC_RUMBLE_L	3
#define PUCK_HAPTIC_RUMBLE_R	4

/* Gain is signed: 0 is medium and -128 is silent. */
#define PUCK_HAPTIC_GAIN_MEDIUM	0
#define PUCK_HAPTIC_GAIN_SILENT	(-128)

/*
 * Start a tone on one haptic. It plays until stopped. Gain above medium is
 * clamped to medium: SteamHapticsSinger warns full gain may damage actuators.
 */
void puck_haptic_tone(int slot, uint8_t haptic, uint16_t hz, int8_t gain);

/* Stop one haptic. Sent as a burst, because a lost stop leaves it playing. */
void puck_haptic_stop(int slot, uint8_t haptic);

/*
 * THE CHIMES, each stored as its own record and edited from the config page.
 * The limits are both what the firmware accepts and what the page is told
 * ($chime <name> get), so the page never carries its own copy.
 *
 * hz 0 is a rest. A rest is played as a stop burst, so it has a longer minimum
 * than a note: a note queued before the burst has gone out would overwrite it.
 */
enum puck_chime_id {
	PUCK_CHIME_WAKE,	/* QAM held, before the radio goes to BLE */
	PUCK_CHIME_DONGLE,	/* the mode chord, before rebooting as the dongle */
	PUCK_CHIME_PRO,		/* the mode chord, before rebooting as Pro */
	PUCK_CHIME_COUNT,
};

/* 2 gave every note a haptic. Version 1 records still load: see puck_melody.c. */
#define PUCK_CHIME_VERSION	2
#define PUCK_CHIME_MAX_NOTES	16
#define PUCK_CHIME_HZ_MIN	50
#define PUCK_CHIME_HZ_MAX	2000
#define PUCK_CHIME_MS_MIN	20
#define PUCK_CHIME_REST_MS_MIN	60
#define PUCK_CHIME_MS_MAX	1000
#define PUCK_CHIME_TOTAL_MS_MAX	4000	/* whatever follows waits for all of it */

/* The haptic a note plays on when it does not say. */
#define PUCK_CHIME_CHANNEL_DEFAULT	PUCK_HAPTIC_RUMBLE_R

struct puck_note {
	uint16_t hz;
	uint16_t ms;
	uint8_t haptic;		/* PUCK_HAPTIC_*; ignored for a rest */
	uint8_t reserved;
};

struct puck_chime {
	uint8_t version;
	uint8_t count;
	struct puck_note notes[PUCK_CHIME_MAX_NOTES];
};

/* The haptics a note may play on, the default first. Returns how many. */
size_t puck_chime_channels(const uint8_t **list);

/* A chime's name in the config protocol, or NULL for an id out of range. */
const char *puck_chime_name(int id);

/* The id for a name, or -1. */
int puck_chime_lookup(const char *name);

/* Load every stored chime, or its default. The store must be mounted. */
void puck_chime_init(void);

/* A chime as it will play, or NULL for an id out of range. */
const struct puck_chime *puck_chime_get(int id);

/* Fill *c with a chime's default. */
void puck_chime_defaults(int id, struct puck_chime *c);

/* Check a chime against the limits. Returns 0, or -EINVAL with a reason in err
 * (which may be NULL).
 */
int puck_chime_check(const struct puck_chime *c, char *err, size_t errlen);

/* Parse "hz:ms[:haptic] ..." into *c and check it. A note without a haptic
 * plays on the default. Returns 0, or -EINVAL with a reason in err.
 */
int puck_chime_parse(const char *s, struct puck_chime *c, char *err,
		     size_t errlen);

/* Make *c a chime and persist it. Returns 0, -EINVAL for a bad id or a chime
 * that fails the checks, or -EIO if it could not be saved (it still plays until
 * reboot).
 */
int puck_chime_set(int id, const struct puck_chime *c);

/* chime.volume as a tone gain, shared by every chime: 0 is silent, 100 the
 * loudest that played clean.
 */
int8_t puck_chime_gain(void);

/*
 * Play a chime on a slot at a gain, then call done() from the system work queue
 * once its last stop has left. The chime is planned into commands before this
 * returns, so *c may change afterwards. Returns 0 if it started, or -EBUSY
 * while a chime is already playing, in which case done() is never called.
 *
 * RF has to own the radio for the whole chime: nothing reaches the controller
 * over BLE.
 */
int puck_melody_play(int slot, const struct puck_chime *c, int8_t gain,
		     void (*done)(void));

/* A stored chime at the configured volume. -EINVAL for an id out of range. */
int puck_melody_play_chime(int id, int slot, void (*done)(void));

/* The wake chime at the configured volume. */
int puck_melody_play_wake(int slot, void (*done)(void));

/* Whether a chime is playing. */
bool puck_melody_busy(void);

#endif /* PUCK_MELODY_H */
