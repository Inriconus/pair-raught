/*
 * The controller's decoded input, shared by every output mode.
 *
 * The RF layer fills this from the Steam Controller's 0x45/0x42 report; the
 * output modes read it. One source of truth, mirroring g_in[] in the Arduino
 * build: a mode never parses the wire format itself.
 *
 * This is deliberately independent of how the puck is presenting itself. The
 * puck's job in RF mode is to be a USB controller to whatever it is plugged
 * into (a Switch 2 as a Pro Controller, or a PC through the Steam dongle
 * interfaces) and all of those consume the same decoded state.
 */

#ifndef PUCK_INPUT_H
#define PUCK_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "puck_usb.h"

/*
 * Steam Controller button bits, as they arrive in the report's 32-bit button
 * word. Names and positions copied from the Arduino build's triton.h.
 */
#define TB_A       0x00000001u
#define TB_B       0x00000002u
#define TB_X       0x00000004u
#define TB_Y       0x00000008u
#define TB_QAM     0x00000010u	/* the "three dots" button */
#define TB_R3      0x00000020u
#define TB_VIEW    0x00000040u
#define TB_R4      0x00000080u	/* back paddles */
#define TB_R5      0x00000100u
#define TB_RB      0x00000200u
#define TB_DDN     0x00000400u
#define TB_DRT     0x00000800u
#define TB_DLF     0x00001000u
#define TB_DUP     0x00002000u
#define TB_MENU    0x00004000u
#define TB_L3      0x00008000u
#define TB_STEAM   0x00010000u
#define TB_L4      0x00020000u
#define TB_L5      0x00040000u
#define TB_LB      0x00080000u
#define TB_RPADT   0x00200000u	/* right pad touch */
#define TB_RPADC   0x00400000u	/* right pad click */
#define TB_R2      0x00800000u	/* full trigger pull, digital */
#define TB_LPADT   0x02000000u
#define TB_LPADC   0x04000000u
#define TB_L2      0x08000000u

/* All four back paddles, the mode-switch chord guard. */
#define CHORD_BACK4 (TB_R4 | TB_L4 | TB_R5 | TB_L5)

/* Analog-trigger fraction of 0xFF at which a digital ZL/ZR trips. */
#define TRIG_ON 40

struct puck_input {
	uint32_t buttons;	/* TB_* bits */
	int16_t lx, ly;		/* left stick, signed 16-bit */
	int16_t rx, ry;		/* right stick */
	uint8_t lt, rt;		/* triggers, full-scale 0..255 */
	int16_t rpx, rpy;	/* right trackpad, absolute; see TB_RPADT */
	int16_t ax, ay, az;	/* accelerometer, raw sensor frame */
	int16_t gx, gy, gz;	/* gyroscope, raw sensor frame */
	bool valid;		/* a report has been decoded for this slot */
};

/* Decode one 0x45/0x42 report body into `slot`'s state. `rep` points at the
 * report id, so field offsets are relative to rep[2] exactly as in the
 * Arduino build's decoders.
 */
void puck_input_update(int slot, const uint8_t *rep);

/* Read a slot's current state. Returns false if nothing has been decoded. */
bool puck_input_get(int slot, struct puck_input *out);

/* Print one slot's live input. Bound to console key 'i'. */
void puck_input_dump(int slot);

#endif /* PUCK_INPUT_H */
