/*
 * HD rumble decode. See puck_rumble.h for the shape of the problem.
 */

#include <zephyr/kernel.h>
#include <math.h>

#include "puck_rumble.h"
#include "puck_usb.h"

/* -8.0 log2 units: silent. Amplitudes live in [HDR_AMP_MIN, 0]. */
#define HDR_AMP_MIN (-256)
#define HDR_AMP_OFF (-256)

/* exp2(units/32) scaled to a 16-bit motor level, built once at boot. The two
 * lowest steps are treated as silent (the curve floors out there), which is
 * what makes an idle frame, decoding to minimum amplitude. Read as off.
 */
static uint16_t g_level[257];

/* Per-slot, per-motor (0 = left, 1 = right) running band amplitudes. The packed
 * 5-bit commands are RELATIVE, so this state must persist between frames.
 */
struct hdr_bands {
	int16_t lo;
	int16_t hi;
};

static struct hdr_bands g_state[PUCK_HID_SLOTS][2];

void puck_rumble_init(void)
{
	for (int u = HDR_AMP_MIN; u <= 0; u++) {
		float lin = (float)u / 32.0f;
		float amp = (lin >= -7.9375f) ? exp2f(lin) : 0.0f;
		uint32_t v;

		if (amp > 1.0f) {
			amp = 1.0f;
		}
		v = (uint32_t)(amp * 65535.0f + 0.5f);
		g_level[u - HDR_AMP_MIN] = (v > 0xFFFF) ? 0xFFFF : (uint16_t)v;
	}

	for (int s = 0; s < PUCK_HID_SLOTS; s++) {
		puck_rumble_reset(s);
	}
}

void puck_rumble_reset(int slot)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS) {
		return;
	}
	g_state[slot][0].lo = g_state[slot][0].hi = HDR_AMP_OFF;
	g_state[slot][1].lo = g_state[slot][1].hi = HDR_AMP_OFF;
}

/* Absolute 7-bit amplitude, in 1/32 log2 units. Three slopes. */
static int16_t amp7(uint8_t code)
{
	if (code == 0) {
		return HDR_AMP_MIN;
	}
	if (code < 16) {
		return (int16_t)(8 * (int)code - 248);	/* slope 1/4 */
	}
	if (code < 32) {
		return (int16_t)(2 * (int)code - 158);	/* slope 1/16 */
	}
	return (int16_t)((int)code - 127);		/* slope 1/32 */
}

/*
 * Apply a compact 5-bit command to a running amplitude:
 *   0        silence
 *   1..11    substitute an absolute preset: 0, -0.5, -1.0 ... -5.0
 *   17..22   step up   (+0.125 for 17-19, +0.03125 for 20-22)
 *   26..31   step down (-0.03125 for 26-28, -0.125 for 29-31)
 *   other    amplitude unchanged. The code carries only a frequency command
 */
static int16_t amp5(uint8_t code, int16_t cur)
{
	int step = 0;
	int v;

	if (code == 0) {
		return HDR_AMP_OFF;
	}
	if (code <= 11) {
		return (int16_t)(-16 * (int)(code - 1));
	}
	if (code >= 17 && code <= 19) {
		step = 4;
	} else if (code >= 20 && code <= 22) {
		step = 1;
	} else if (code >= 26 && code <= 28) {
		step = -1;
	} else if (code >= 29 && code <= 31) {
		step = -4;
	}
	v = (int)cur + step;
	return v < HDR_AMP_MIN ? HDR_AMP_MIN : (v > 0 ? 0 : (int16_t)v);
}

/* Pull a bit-field out of the 32-bit rumble word. */
static uint8_t field(uint32_t w, uint8_t shift, uint8_t width)
{
	return (uint8_t)((w >> shift) & ((1u << width) - 1u));
}

uint16_t puck_rumble_decode(int slot, int motor, const uint8_t b[4])
{
	struct hdr_bands *s;
	uint32_t w;
	uint16_t peak = 0;

	if (slot < 0 || slot >= PUCK_HID_SLOTS || motor < 0 || motor > 1) {
		return 0;
	}
	s = &g_state[slot][motor];
	w = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) |
	    ((uint32_t)b[3] << 24);

#define SAMPLE()                                                     \
	do {                                                         \
		uint16_t la = g_level[s->lo - HDR_AMP_MIN];          \
		uint16_t ha = g_level[s->hi - HDR_AMP_MIN];          \
		uint16_t lv = la > ha ? la : ha;                     \
		if (lv > peak) {                                     \
			peak = lv;                                   \
		}                                                    \
	} while (0)

	switch (field(w, 30, 2)) {
	case 0:		/* hold */
		SAMPLE();
		break;
	case 1:
		if ((w & 0xFFFFF) == 0) {		/* one 5-bit update */
			s->lo = amp5(field(w, 25, 5), s->lo);
			s->hi = amp5(field(w, 20, 5), s->hi);
			SAMPLE();
		} else if ((w & 0x3) == 0) {		/* one 7-bit absolute */
			s->lo = amp7(field(w, 23, 7));
			s->hi = amp7(field(w, 9, 7));
			SAMPLE();
		} else {	/* 7-bit for one band, then two 5-bit updates */
			bool want_hi = (w & 1) != 0;
			bool is_freq = ((w >> 2) & 1) != 0;

			if (!is_freq) {		/* else it is a frequency: ignore */
				if (want_hi) {
					s->hi = amp7(field(w, 23, 7));
				} else {
					s->lo = amp7(field(w, 23, 7));
				}
			}
			SAMPLE();
			s->lo = amp5(field(w, 18, 5), s->lo);
			s->hi = amp5(field(w, 13, 5), s->hi);
			SAMPLE();
			s->lo = amp5(field(w, 8, 5), s->lo);
			s->hi = amp5(field(w, 3, 5), s->hi);
			SAMPLE();
		}
		break;
	case 2:
		if ((w & 0x3FF) == 0) {			/* two 5-bit updates */
			s->lo = amp5(field(w, 25, 5), s->lo);
			s->hi = amp5(field(w, 20, 5), s->hi);
			SAMPLE();
			s->lo = amp5(field(w, 15, 5), s->lo);
			s->hi = amp5(field(w, 10, 5), s->hi);
			SAMPLE();
		} else {		/* 7-bit + 5-bit, then a 5-bit update */
			if (w & 1) {
				s->hi = amp7(field(w, 23, 7));
				s->lo = amp5(field(w, 18, 5), s->lo);
			} else {
				s->lo = amp7(field(w, 23, 7));
				s->hi = amp5(field(w, 18, 5), s->hi);
			}
			SAMPLE();
			s->lo = amp5(field(w, 13, 5), s->lo);
			s->hi = amp5(field(w, 8, 5), s->hi);
			SAMPLE();
		}
		break;
	case 3:						/* three 5-bit updates */
		s->lo = amp5(field(w, 25, 5), s->lo);
		s->hi = amp5(field(w, 20, 5), s->hi);
		SAMPLE();
		s->lo = amp5(field(w, 15, 5), s->lo);
		s->hi = amp5(field(w, 10, 5), s->hi);
		SAMPLE();
		s->lo = amp5(field(w, 5, 5), s->lo);
		s->hi = amp5(field(w, 0, 5), s->hi);
		SAMPLE();
		break;
	}
#undef SAMPLE

	return peak;
}
