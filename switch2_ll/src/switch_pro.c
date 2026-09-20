/*
 * Steam Controller to Pro Controller mapping.
 *
 * Ported from mode_switch_pro.cpp. The mapping itself is the valuable part and
 * is reproduced faithfully. The configurable layer arrived later, with the
 * config screen: the back paddles and QAM read their targets from settings
 * below. A/B swap is still fixed.
 */

#include <zephyr/kernel.h>

#include "switch_pro.h"
#include "puck_settings.h"

/*
 * THE STEAM BUTTON AS HOME, decided on release.
 *
 * Steam + Y is the power-off chord, and HOME cannot be taken back once the
 * console has seen it. Nobody presses or releases two buttons inside one 10 ms
 * report, so reading Steam straight through as HOME tapped HOME at the console
 * on the way into and out of every power-off.
 *
 *   released before HOME_HOLD_MS    a HOME tap, HOME_TAP_MS long
 *   still held at HOME_HOLD_MS      HOME held until Steam is released, which
 *                                   is what opens quick settings
 *   Y down before either of those   nothing until Steam is released
 *   controller not connected        nothing, and any HOME is let go
 *
 * Only Y cancels, because Steam + Y is the only chord on this button. Steam
 * with any other button still sends HOME.
 */
#define HOME_HOLD_MS 500u
#define HOME_TAP_MS  100u

enum home_state {
	HOME_IDLE,	/* Steam up */
	HOME_PENDING,	/* Steam down, not yet a tap or a hold */
	HOME_TAP,	/* released in time: HOME for HOME_TAP_MS */
	HOME_HELD,	/* past HOME_HOLD_MS: HOME until Steam is up */
	HOME_CHORD,	/* Y joined in: nothing until Steam is up */
};

static enum home_state g_home;
static uint32_t g_home_ms;

void switch_pro_home_update(const struct puck_input *in)
{
	uint32_t now = k_uptime_get_32();
	bool steam, y;

	if (!in) {
		g_home = HOME_IDLE;
		return;
	}
	steam = (in->buttons & TB_STEAM) != 0;
	y = (in->buttons & TB_Y) != 0;

	switch (g_home) {
	case HOME_IDLE:
		if (steam) {
			g_home = y ? HOME_CHORD : HOME_PENDING;
			g_home_ms = now;
		}
		break;
	case HOME_PENDING:
		if (!steam) {
			g_home = HOME_TAP;
			g_home_ms = now;
		} else if (y) {
			g_home = HOME_CHORD;
		} else if (now - g_home_ms >= HOME_HOLD_MS) {
			g_home = HOME_HELD;
		}
		break;
	case HOME_TAP:
		if (now - g_home_ms >= HOME_TAP_MS) {
			g_home = HOME_IDLE;
		}
		break;
	case HOME_HELD:
	case HOME_CHORD:
		if (!steam) {
			g_home = HOME_IDLE;
		}
		break;
	}
}

uint32_t switch_pro_buttons(const struct puck_input *in)
{
	uint32_t b = in->buttons;
	uint32_t jc = 0;

	/*
	 * CHORD GUARDS. A chord is an instruction to the puck, not gameplay, so
	 * the buttons that make one up must not reach the console while it is
	 * being held. EVERY chord in rf_link.c needs an entry here:
	 *
	 *   back-4 + L3 + R3   register with a console      guarded below
	 *   back-4 + L1 + R1   swap the USB identity        guarded below
	 *   Steam + Y          power the controller off     Y guarded below;
	 *                                                   HOME waits for the
	 *                                                   release, see above
	 *   QAM held           wake a console               deliberately not,
	 *                                                   see the note at the
	 *                                                   paddle mappings
	 *
	 * The first two share the four-paddle hold, so they also suppress the
	 * face and d-pad buttons: a paddle mapped to A would otherwise put A
	 * straight back, because holding the paddles IS the chord.
	 */
	const bool back4 = (b & CHORD_BACK4) == CHORD_BACK4;
	const bool poweroff = (b & (TB_STEAM | TB_Y)) == (TB_STEAM | TB_Y);

	if (back4) {
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y | TB_DUP | TB_DDN |
				 TB_DLF | TB_DRT |
				 /* The bits these two chords are MADE OF, or
				  * holding one clicks both sticks or both
				  * bumpers at the console for two seconds.
				  */
				 TB_L3 | TB_R3 | TB_LB | TB_RB);
	}

	if (poweroff) {
		/*
		 * Only Y is masked here. HOME is never read from this word: the
		 * release rule above decides it, and sends nothing for a chord
		 * that starts before the hold threshold.
		 */
		b &= ~(uint32_t)TB_Y;
	}

	/* Face buttons, straight through: the Steam Controller's A is the
	 * console's A. Swapping A/B and X/Y for the Nintendo layout would belong
	 * with the other mappings on the config page, which does not offer it.
	 */
	if (b & TB_A) {
		jc |= JC_BTN_A;
	}
	if (b & TB_B) {
		jc |= JC_BTN_B;
	}
	if (b & TB_X) {
		jc |= JC_BTN_X;
	}
	if (b & TB_Y) {
		jc |= JC_BTN_Y;
	}

	/* Shoulders. */
	if (b & TB_LB) {
		jc |= JC_BTN_L;
	}
	if (b & TB_RB) {
		jc |= JC_BTN_R;
	}

	/*
	 * Triggers: the Pro Controller's ZL/ZR are DIGITAL, so an analog pull
	 * past the threshold counts, and so does the controller's own
	 * full-pull digital bit.
	 */
	if (in->lt >= TRIG_ON || (b & TB_L2)) {
		jc |= JC_BTN_ZL;
	}
	if (in->rt >= TRIG_ON || (b & TB_R2)) {
		jc |= JC_BTN_ZR;
	}

	/* Note the crossing: VIEW maps to PLUS and MENU to MINUS, matching the
	 * Arduino build rather than the names' apparent symmetry.
	 */
	if (b & TB_VIEW) {
		jc |= JC_BTN_PLUS;
	}
	if (b & TB_MENU) {
		jc |= JC_BTN_MINUS;
	}

	if (b & TB_L3) {
		jc |= JC_BTN_LSTICK;
	}
	if (b & TB_R3) {
		jc |= JC_BTN_RSTICK;
	}
	if (g_home == HOME_TAP || g_home == HOME_HELD) {
		jc |= JC_BTN_HOME;
	}

	if (b & TB_DUP) {
		jc |= JC_BTN_UP;
	}
	if (b & TB_DDN) {
		jc |= JC_BTN_DOWN;
	}
	if (b & TB_DLF) {
		jc |= JC_BTN_LEFT;
	}
	if (b & TB_DRT) {
		jc |= JC_BTN_RIGHT;
	}

	/*
	 * The extra buttons the Pro Controller does not have. Each carries a
	 * configured target rather than a fixed one. They were left unmapped
	 * until the config surface existed to choose, because an arbitrary
	 * default is worse than nothing on a button someone may be resting a
	 * finger on.
	 */
	if (!back4) {
		if (in->buttons & TB_L4) {
			jc |= puck_settings_pad_bit(g_cfg.map_l4);
		}
		if (in->buttons & TB_R4) {
			jc |= puck_settings_pad_bit(g_cfg.map_r4);
		}
		if (in->buttons & TB_L5) {
			jc |= puck_settings_pad_bit(g_cfg.map_l5);
		}
		if (in->buttons & TB_R5) {
			jc |= puck_settings_pad_bit(g_cfg.map_r5);
		}
	}

	/*
	 * QAM is outside the guard. It is not part of any chord, so holding the
	 * paddles says nothing about whether a QAM press was meant. The wake
	 * macro is a hold on QAM alone, and a hold has no other buttons in it
	 * to suppress.
	 */
	if (in->buttons & TB_QAM) {
		jc |= puck_settings_pad_bit(g_cfg.map_qam);
	}

	return jc;
}

uint16_t switch_pro_axis(int16_t v)
{
	/*
	 * s16 (centred 0) -> 12-bit centred at 2048, matching the Arduino
	 * build's jcStick12().
	 *
	 * The shift is by FOUR, not five. A 12-bit axis spans 4096, so the
	 * s16 range has to be divided by 16 to fill it; dividing by 32 fills
	 * only half. This was measured, not reasoned: with >> 5 a full sweep
	 * of the left stick produced X 1024..3071 and Y 1024..3071, exactly
	 * 2048 +/- 1024, so the console would have seen a controller
	 * physically unable to push past half deflection. At rest both shifts
	 * read ~2048, which is why an at-rest check could not see it.
	 *
	 * The clamp stays: a stick reading harder than full scale is a value
	 * real hardware cannot produce, and the console does notice
	 * out-of-range axes.
	 */
	int32_t c = 2048 + (v >> 4);

	if (c < 0) {
		c = 0;
	} else if (c > 4095) {
		c = 4095;
	}
	return (uint16_t)c;
}

void switch_pro_dump(int slot)
{
	struct puck_input in;
	uint32_t jc;

	if (!puck_input_get(slot, &in)) {
		printk("switch-pro: slot %d has no report yet\n", slot);
		return;
	}

	jc = switch_pro_buttons(&in);

	printk("switch-pro slot%d: jc=%06x L(%u,%u) R(%u,%u)", slot,
	       (unsigned)jc, switch_pro_axis(in.lx), switch_pro_axis(in.ly),
	       switch_pro_axis(in.rx), switch_pro_axis(in.ry));

	if (jc & JC_BTN_A) { printk(" A"); }
	if (jc & JC_BTN_B) { printk(" B"); }
	if (jc & JC_BTN_X) { printk(" X"); }
	if (jc & JC_BTN_Y) { printk(" Y"); }
	if (jc & JC_BTN_L) { printk(" L"); }
	if (jc & JC_BTN_R) { printk(" R"); }
	if (jc & JC_BTN_ZL) { printk(" ZL"); }
	if (jc & JC_BTN_ZR) { printk(" ZR"); }
	if (jc & JC_BTN_MINUS) { printk(" -"); }
	if (jc & JC_BTN_PLUS) { printk(" +"); }
	if (jc & JC_BTN_LSTICK) { printk(" L3"); }
	if (jc & JC_BTN_RSTICK) { printk(" R3"); }
	if (jc & JC_BTN_HOME) { printk(" HOME"); }
	if (jc & JC_BTN_UP) { printk(" UP"); }
	if (jc & JC_BTN_DOWN) { printk(" DOWN"); }
	if (jc & JC_BTN_LEFT) { printk(" LEFT"); }
	if (jc & JC_BTN_RIGHT) { printk(" RIGHT"); }
	printk("\n");
}
