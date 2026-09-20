/*
 * Nintendo Switch Pro Controller presentation.
 *
 * This is the puck's job in normal use: plugged into the console's USB port,
 * presenting as a Pro Controller, driven by the Steam Controller over RF. BLE
 * is not involved. That exists only to wake a sleeping console.
 *
 * Only the Pro Controller is ported. The Arduino build also has Xbox, PS3, PS5,
 * Hori, lizard and hidgyro modes; those are deliberately out of scope here.
 */

#ifndef SWITCH_PRO_H
#define SWITCH_PRO_H

#include <stdint.h>

#include "puck_input.h"

/*
 * Pro Controller button bits, as they sit in the 24-bit field of the 0x30 input
 * report. Copied from the Arduino build rather than re-derived, these are
 * measured positions, and the BLE-era guesses at Switch 2 button bits are NOT a
 * substitute for them.
 */
#define JC_BTN_Y       (1u << 0)
#define JC_BTN_X       (1u << 1)
#define JC_BTN_B       (1u << 2)
#define JC_BTN_A       (1u << 3)
#define JC_BTN_R       (1u << 6)
#define JC_BTN_ZR      (1u << 7)
#define JC_BTN_MINUS   (1u << 8)
#define JC_BTN_PLUS    (1u << 9)
#define JC_BTN_RSTICK  (1u << 10)
#define JC_BTN_LSTICK  (1u << 11)
#define JC_BTN_HOME    (1u << 12)
#define JC_BTN_CAPTURE (1u << 13)
#define JC_BTN_DOWN    (1u << 16)
#define JC_BTN_UP      (1u << 17)
#define JC_BTN_RIGHT   (1u << 18)
#define JC_BTN_LEFT    (1u << 19)
#define JC_BTN_L       (1u << 22)
#define JC_BTN_ZL      (1u << 23)

/* Map decoded Steam Controller input to the Pro Controller's button word. */
uint32_t switch_pro_buttons(const struct puck_input *in);

/*
 * Advance the Steam button's HOME decision (see switch_pro.c). Call once per
 * pass from the Pro Controller task with the controller's current input, or
 * NULL while it is not connected. switch_pro_buttons() only reads the result,
 * so the debug dump and subcommand replies cannot move it.
 */
void switch_pro_home_update(const struct puck_input *in);

/*
 * Convert a signed 16-bit stick axis to the Pro Controller's 12-bit form,
 * centred at 2048.
 */
uint16_t switch_pro_axis(int16_t v);

/* Print the mapped Pro Controller state for one slot. Bound to console key 'j'. */
void switch_pro_dump(int slot);

#endif /* SWITCH_PRO_H */
