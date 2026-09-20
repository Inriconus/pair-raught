/*
 * Haptic tones and the chimes. See puck_melody.h.
 */

#include <zephyr/kernel.h>
#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include "puck_melody.h"
#include "puck_settings.h"
#include "rf_link.h"
#include "switch2_store.h"

/*
 * Output report 0x83, nine bytes:
 *
 *   tone  [haptic][gain][freq Hz, le16][FF 7F][00 00 00]
 *   stop  [haptic][80][00 00][00 80][00 00 00]
 *
 * gain is signed and 0x00 is medium. SteamHapticsSinger warns that full gain
 * may damage the actuators, so nothing here goes above medium.
 */
#define HAPTIC_RID	0x83
#define STOP_SHOTS	3

void puck_haptic_tone(int slot, uint8_t haptic, uint16_t hz, int8_t gain)
{
	uint8_t p[9] = { haptic, 0, hz & 0xFF, hz >> 8, 0xFF, 0x7F };

	if (gain > PUCK_HAPTIC_GAIN_MEDIUM) {
		gain = PUCK_HAPTIC_GAIN_MEDIUM;
	}
	p[1] = (uint8_t)gain;

	/* Once, the way Steam's own haptics are relayed. Whether a repeat
	 * restarts the waveform is unknown, and a lost tone is a missed note,
	 * where a lost stop would be a stuck one.
	 */
	rf_link_relay(slot, HAPTIC_RID, p, sizeof(p), 1);
}

void puck_haptic_stop(int slot, uint8_t haptic)
{
	uint8_t p[9] = { haptic, 0x80, 0x00, 0x00, 0x00, 0x80 };

	rf_link_relay(slot, HAPTIC_RID, p, sizeof(p), STOP_SHOTS);
}

/*
 * The haptics a note may name, the default first. The right rumble motor is
 * the default because it is what the chimes were tuned on: the trackpads were
 * too quiet on a real wake, and both rumble motors at once sounded distorted.
 */
static const uint8_t CHANNELS[] = {
	PUCK_HAPTIC_RUMBLE_R,
	PUCK_HAPTIC_RUMBLE_L,
	PUCK_HAPTIC_PAD_R,
	PUCK_HAPTIC_PAD_L,
};

size_t puck_chime_channels(const uint8_t **list)
{
	*list = CHANNELS;
	return ARRAY_SIZE(CHANNELS);
}

static bool channel_ok(uint8_t haptic)
{
	for (size_t i = 0; i < ARRAY_SIZE(CHANNELS); i++) {
		if (CHANNELS[i] == haptic) {
			return true;
		}
	}
	return false;
}

/*
 * THE DEFAULT CHIMES, all on the default haptic.
 *
 * Frequencies are whole Hz, which is all the command carries, so each note is
 * within half a hertz of true pitch.
 */
#define NOTE(h, m) { .hz = (h), .ms = (m), .haptic = PUCK_CHIME_CHANNEL_DEFAULT }

static const struct puck_note DEFAULT_WAKE[] = {
	NOTE(1109,  90),	/* C#6 */
	NOTE( 554,  90),	/* C#5 */
	NOTE( 740,  90),	/* F#5 */
	NOTE( 932,  90),	/* A#5 */
	NOTE( 831, 220),	/* G#5 */
};

/* The mode chimes are two notes a fifth apart, so they cannot be mistaken for
 * the wake: falling into the dongle, rising into Pro Controller.
 */
static const struct puck_note DEFAULT_DONGLE[] = {
	NOTE( 831,  90),	/* G#5 */
	NOTE( 554, 220),	/* C#5 */
};

static const struct puck_note DEFAULT_PRO[] = {
	NOTE( 554,  90),	/* C#5 */
	NOTE( 831, 220),	/* G#5 */
};

static const struct {
	const char *name;
	const struct puck_note *notes;
	uint8_t count;
} CHIMES[PUCK_CHIME_COUNT] = {
	[PUCK_CHIME_WAKE]   = { "wake",   DEFAULT_WAKE,   ARRAY_SIZE(DEFAULT_WAKE) },
	[PUCK_CHIME_DONGLE] = { "dongle", DEFAULT_DONGLE, ARRAY_SIZE(DEFAULT_DONGLE) },
	[PUCK_CHIME_PRO]    = { "pro",    DEFAULT_PRO,    ARRAY_SIZE(DEFAULT_PRO) },
};

/*
 * Version 1 records, from before a note named its haptic, when every note
 * played on the right rumble motor. Read at boot and converted, so a chime saved
 * then still plays as it did. The stored length is what tells the two apart.
 */
struct puck_chime_v1 {
	uint8_t version;
	uint8_t count;
	struct {
		uint16_t hz;
		uint16_t ms;
	} notes[PUCK_CHIME_MAX_NOTES];
};

BUILD_ASSERT(sizeof(struct puck_chime_v1) == 66, "version 1 chime layout");
BUILD_ASSERT(sizeof(struct puck_chime) == 98, "version 2 chime layout");

/*
 * The loudest a chime is allowed, which is chime.volume 100. Tuned by ear on
 * the right rumble motor: medium (0) sounded slightly distorted, -16 was clean
 * but a little too quiet, and -8 was right. Volume maps linearly from silent up
 * to this; whether loudness follows gain linearly is not known.
 */
#define CHIME_GAIN_MAX	(-8)

/*
 * How long a stop burst takes to leave. With RF up a slot is polled about every
 * 10.6 ms, so three shots are gone in ~32 ms. The relay holds ONE command per
 * slot and the next one overwrites it, so nothing is queued behind a stop until
 * this has passed.
 */
#define BURST_MS	50

/*
 * The same limit for a single-shot tone: anything queued sooner overwrites it
 * before it goes out. Two polls and some, because the RF thread also blocks
 * while it prints its counters.
 */
#define TONE_GAP_MS	25

static struct puck_chime g_chimes[PUCK_CHIME_COUNT];

/* --- the stored chimes -------------------------------------------------- */

const char *puck_chime_name(int id)
{
	return (id >= 0 && id < PUCK_CHIME_COUNT) ? CHIMES[id].name : NULL;
}

int puck_chime_lookup(const char *name)
{
	for (int id = 0; id < PUCK_CHIME_COUNT; id++) {
		if (!strcmp(name, CHIMES[id].name)) {
			return id;
		}
	}
	return -1;
}

void puck_chime_defaults(int id, struct puck_chime *c)
{
	memset(c, 0, sizeof(*c));
	c->version = PUCK_CHIME_VERSION;
	if (id < 0 || id >= PUCK_CHIME_COUNT) {
		return;
	}
	c->count = CHIMES[id].count;
	memcpy(c->notes, CHIMES[id].notes, c->count * sizeof(c->notes[0]));
}

static int reject(char *err, size_t errlen, const char *fmt, ...)
{
	va_list ap;

	if (err && errlen) {
		va_start(ap, fmt);
		vsnprintk(err, errlen, fmt, ap);
		va_end(ap);
	}
	return -EINVAL;
}

int puck_chime_check(const struct puck_chime *c, char *err, size_t errlen)
{
	uint32_t total = 0;

	if (c->count < 1 || c->count > PUCK_CHIME_MAX_NOTES) {
		return reject(err, errlen, "a chime has 1 to %u notes",
			      PUCK_CHIME_MAX_NOTES);
	}

	for (int i = 0; i < c->count; i++) {
		const struct puck_note *n = &c->notes[i];
		uint16_t ms_min = n->hz ? PUCK_CHIME_MS_MIN
					: PUCK_CHIME_REST_MS_MIN;

		if (n->hz && (n->hz < PUCK_CHIME_HZ_MIN ||
			      n->hz > PUCK_CHIME_HZ_MAX)) {
			return reject(err, errlen,
				      "note %d: %u Hz is outside %u to %u", i + 1,
				      n->hz, PUCK_CHIME_HZ_MIN,
				      PUCK_CHIME_HZ_MAX);
		}
		if (n->hz && !channel_ok(n->haptic)) {
			return reject(err, errlen, "note %d: no haptic %u", i + 1,
				      n->haptic);
		}
		if (n->ms < ms_min || n->ms > PUCK_CHIME_MS_MAX) {
			return reject(err, errlen,
				      "note %d: %u ms is outside %u to %u%s",
				      i + 1, n->ms, ms_min, PUCK_CHIME_MS_MAX,
				      n->hz ? "" : " for a rest");
		}
		total += n->ms;
	}

	if (total > PUCK_CHIME_TOTAL_MS_MAX) {
		return reject(err, errlen, "the chime is %u ms, over the %u limit",
			      total, PUCK_CHIME_TOTAL_MS_MAX);
	}
	return 0;
}

/* One decimal number up to 65535, advancing *s past it. */
static bool parse_u16(const char **s, uint16_t *out)
{
	const char *p = *s;
	uint32_t v = 0;

	if (*p < '0' || *p > '9') {
		return false;
	}
	while (*p >= '0' && *p <= '9') {
		v = v * 10u + (uint32_t)(*p - '0');
		if (v > 0xFFFF) {
			return false;
		}
		p++;
	}
	*out = (uint16_t)v;
	*s = p;
	return true;
}

int puck_chime_parse(const char *s, struct puck_chime *c, char *err,
		     size_t errlen)
{
	memset(c, 0, sizeof(*c));
	c->version = PUCK_CHIME_VERSION;

	for (;;) {
		struct puck_note n = { .haptic = PUCK_CHIME_CHANNEL_DEFAULT };
		uint16_t haptic;

		while (*s == ' ') {
			s++;
		}
		if (!*s) {
			break;
		}
		if (c->count == PUCK_CHIME_MAX_NOTES) {
			return reject(err, errlen, "more than %u notes",
				      PUCK_CHIME_MAX_NOTES);
		}
		if (!parse_u16(&s, &n.hz) || *s != ':') {
			return reject(err, errlen,
				      "note %u: expected hz:ms or hz:ms:haptic",
				      c->count + 1);
		}
		s++;
		if (!parse_u16(&s, &n.ms)) {
			return reject(err, errlen,
				      "note %u: expected hz:ms or hz:ms:haptic",
				      c->count + 1);
		}
		if (*s == ':') {
			s++;
			if (!parse_u16(&s, &haptic) || haptic > 0xFF) {
				return reject(err, errlen,
					      "note %u: expected a haptic after "
					      "the second colon", c->count + 1);
			}
			n.haptic = (uint8_t)haptic;
		}
		if (*s && *s != ' ') {
			return reject(err, errlen,
				      "note %u: expected hz:ms or hz:ms:haptic",
				      c->count + 1);
		}
		c->notes[c->count++] = n;
	}

	return puck_chime_check(c, err, errlen);
}

/* A stored chime, converted from version 1 if that is what was saved. */
static bool load_stored(int id, struct puck_chime *c)
{
	struct puck_chime_v1 old;

	if (switch2_store_load_chime(id, c, sizeof(*c))) {
		return true;
	}
	if (!switch2_store_load_chime(id, &old, sizeof(old)) || old.version != 1) {
		return false;
	}

	memset(c, 0, sizeof(*c));
	c->version = PUCK_CHIME_VERSION;
	c->count = MIN(old.count, PUCK_CHIME_MAX_NOTES);
	for (int i = 0; i < c->count; i++) {
		c->notes[i].hz = old.notes[i].hz;
		c->notes[i].ms = old.notes[i].ms;
		c->notes[i].haptic = PUCK_CHIME_CHANNEL_DEFAULT;
	}
	printk("chime: %s was saved before notes had a haptic: all on the "
	       "default\n", CHIMES[id].name);
	return true;
}

void puck_chime_init(void)
{
	struct puck_chime stored;

	for (int id = 0; id < PUCK_CHIME_COUNT; id++) {
		puck_chime_defaults(id, &g_chimes[id]);

		if (!load_stored(id, &stored)) {
			continue;	/* never edited: the default */
		}

		/*
		 * A record this build would refuse came from one with other
		 * limits. Playing notes the page could never have sent is worse
		 * than the default.
		 */
		if (stored.version != PUCK_CHIME_VERSION ||
		    puck_chime_check(&stored, NULL, 0)) {
			printk("chime: stored %s chime is not usable: using the "
			       "default\n", CHIMES[id].name);
			continue;
		}

		g_chimes[id] = stored;
		printk("chime: %s, %u notes loaded\n", CHIMES[id].name,
		       stored.count);
	}
}

const struct puck_chime *puck_chime_get(int id)
{
	return (id >= 0 && id < PUCK_CHIME_COUNT) ? &g_chimes[id] : NULL;
}

int puck_chime_set(int id, const struct puck_chime *c)
{
	if (id < 0 || id >= PUCK_CHIME_COUNT || puck_chime_check(c, NULL, 0)) {
		return -EINVAL;
	}

	g_chimes[id] = *c;
	g_chimes[id].version = PUCK_CHIME_VERSION;
	return switch2_store_save_chime(id, &g_chimes[id], sizeof(g_chimes[id]));
}

int8_t puck_chime_gain(void)
{
	int32_t span = CHIME_GAIN_MAX - PUCK_HAPTIC_GAIN_SILENT;
	int32_t vol = MIN(g_cfg.chime_volume, 100);

	return (int8_t)(PUCK_HAPTIC_GAIN_SILENT + (span * vol + 50) / 100);
}

/* --- the player --------------------------------------------------------- */

/*
 * A chime is planned into commands before it plays, one per relay, because the
 * relay carries one at a time. Each command waits `ms` before the next is sent.
 */
enum { ACT_TONE, ACT_STOP, ACT_WAIT };

struct action {
	uint8_t op;
	uint8_t haptic;
	uint16_t hz;
	uint16_t ms;
};

/* Lead and closing stops on every haptic, and at most two commands a note. */
#define MAX_ACTIONS	(2 * ARRAY_SIZE(CHANNELS) + 2 * PUCK_CHIME_MAX_NOTES)

static atomic_t g_busy;
static int g_slot;
static int8_t g_gain;
static struct action g_actions[MAX_ACTIONS];
static int g_nactions;
static int g_next;
static void (*g_done)(void);

static void step_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(g_step_work, step_fn);

static void push(uint8_t op, uint8_t haptic, uint16_t hz, int ms)
{
	if (g_nactions < MAX_ACTIONS) {
		g_actions[g_nactions++] = (struct action){
			.op = op, .haptic = haptic, .hz = hz,
			.ms = (uint16_t)MAX(ms, 1),
		};
	}
}

static void plan(const struct puck_chime *c)
{
	uint32_t used = 0;	/* a bit per haptic that plays a note */
	int sounding = -1;	/* the haptic still playing, if any */
	int count = MIN(c->count, PUCK_CHIME_MAX_NOTES);

	g_nactions = 0;

	for (int i = 0; i < count; i++) {
		if (c->notes[i].hz) {
			used |= BIT(c->notes[i].haptic);
		}
	}

	/* Stops lead, on every haptic the chime uses. SteamHapticsSinger found
	 * the rumble motors could reboot the controller on a note with nothing
	 * stopped before it.
	 */
	for (int h = 0; h < 8; h++) {
		if (used & BIT(h)) {
			push(ACT_STOP, h, 0, BURST_MS);
		}
	}

	for (int i = 0; i < count; i++) {
		const struct puck_note *n = &c->notes[i];

		if (n->hz == 0) {
			/* A rest stops whatever is playing, and waits out the
			 * burst like any stop. The rest minimum covers it.
			 */
			if (sounding >= 0) {
				push(ACT_STOP, sounding, 0, MAX(n->ms, BURST_MS));
				sounding = -1;
			} else {
				push(ACT_WAIT, 0, 0, n->ms);
			}
		} else if (sounding < 0 || sounding == n->haptic) {
			/* The same haptic, or nothing playing: back to back,
			 * with no stop between notes. A new tone replaces the one
			 * playing, which is how SteamHapticsSinger plays too.
			 */
			push(ACT_TONE, n->haptic, n->hz, n->ms);
			sounding = n->haptic;
		} else {
			/* Another haptic. Start the new note first so it lands
			 * on time, then stop the old one once that tone has gone
			 * out. The stop is a burst and nothing may be queued
			 * behind it until it has left, so a note shorter than
			 * the two together is stretched to fit.
			 */
			push(ACT_TONE, n->haptic, n->hz, TONE_GAP_MS);
			push(ACT_STOP, sounding, 0,
			     MAX((int)n->ms - TONE_GAP_MS, BURST_MS));
			sounding = n->haptic;
		}
	}

	/* Stops close, on every haptic used, which also catches a mid-chime stop
	 * that a later command overwrote.
	 */
	for (int h = 0; h < 8; h++) {
		if (used & BIT(h)) {
			push(ACT_STOP, h, 0, BURST_MS);
		}
	}
}

/*
 * One command per run, paced by the work queue rather than by sleeping, so the
 * chime never holds up anything else queued there.
 */
static void step_fn(struct k_work *work)
{
	const struct action *a;

	ARG_UNUSED(work);

	if (g_next >= g_nactions) {
		void (*done)(void) = g_done;

		atomic_set(&g_busy, 0);
		if (done) {
			done();
		}
		return;
	}

	a = &g_actions[g_next++];
	if (a->op == ACT_TONE) {
		puck_haptic_tone(g_slot, a->haptic, a->hz, g_gain);
	} else if (a->op == ACT_STOP) {
		puck_haptic_stop(g_slot, a->haptic);
	}
	k_work_schedule(&g_step_work, K_MSEC(a->ms));
}

int puck_melody_play(int slot, const struct puck_chime *c, int8_t gain,
		     void (*done)(void))
{
	if (!atomic_cas(&g_busy, 0, 1)) {
		return -EBUSY;
	}

	g_slot = slot;
	g_gain = gain;
	g_done = done;
	plan(c);
	g_next = 0;
	printk("chime: slot%d, %u notes as %d commands, gain %d\n", slot,
	       c->count, g_nactions, gain);
	k_work_schedule(&g_step_work, K_NO_WAIT);
	return 0;
}

int puck_melody_play_chime(int id, int slot, void (*done)(void))
{
	if (id < 0 || id >= PUCK_CHIME_COUNT) {
		return -EINVAL;
	}
	return puck_melody_play(slot, &g_chimes[id], puck_chime_gain(), done);
}

int puck_melody_play_wake(int slot, void (*done)(void))
{
	return puck_melody_play_chime(PUCK_CHIME_WAKE, slot, done);
}

bool puck_melody_busy(void)
{
	return atomic_get(&g_busy) != 0;
}
