/*
 * Decode the Steam Controller's input report.
 *
 * Field offsets are relative to rep[2], the buttons' low byte, matching the
 * Arduino build's s16off()/u16off() decoders so the two firmwares read the same
 * wire format the same way:
 *
 *   +0   buttons u32 LE
 *   +4   left trigger  u16
 *   +6   right trigger u16
 *   +8   left stick  X s16   +10  left stick  Y s16
 *   +12  right stick X s16   +14  right stick Y s16
 *   +22  right pad X s16     +24  right pad Y s16
 *   (IMU: accel @0x22, gyro @0x28 from the report start)
 *
 * Report id 0x45 is the legacy form and 0x42 the one shipped in the mid-2026
 * controller update; bytes [0..45] are byte-for-byte identical between them, so
 * both decode through this one path.
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "puck_input.h"

static struct puck_input g_in[PUCK_HID_SLOTS];

static int s16off(const uint8_t *r, int off)
{
	int v = r[2 + off] | (r[2 + off + 1] << 8);

	return (v & 0x8000) ? v - 0x10000 : v;
}

static int u16off(const uint8_t *r, int off)
{
	return r[2 + off] | (r[2 + off + 1] << 8);
}

/*
 * The controller's trigger u16 tops out near HALF scale (~0x8000) on a full
 * pull, so a straight >>8 reads only ~0x80 and the host sees a half-pressed
 * trigger. Scale by two and saturate.
 */
static uint8_t trig_u8(int v16)
{
	int v = v16 >> 7;

	return (uint8_t)(v > 255 ? 255 : v);
}

void puck_input_update(int slot, const uint8_t *rep)
{
	struct puck_input *in;

	if (slot < 0 || slot >= PUCK_HID_SLOTS) {
		return;
	}
	in = &g_in[slot];

	in->buttons = (uint32_t)rep[2] | ((uint32_t)rep[3] << 8) |
		      ((uint32_t)rep[4] << 16) | ((uint32_t)rep[5] << 24);

	in->lt = trig_u8(u16off(rep, 4));
	in->rt = trig_u8(u16off(rep, 6));

	in->lx = (int16_t)s16off(rep, 8);
	in->ly = (int16_t)s16off(rep, 10);
	in->rx = (int16_t)s16off(rep, 12);
	in->ry = (int16_t)s16off(rep, 14);

	/* Right trackpad, absolute. Only meaningful while TB_RPADT says a
	 * finger is down. The coordinates hold their last value on release,
	 * so anything reading them must gate on the touch bit or it will see a
	 * jump the next time a finger lands somewhere else.
	 */
	in->rpx = (int16_t)s16off(rep, 22);
	in->rpy = (int16_t)s16off(rep, 24);

	/* IMU: accel at 32/34/36, gyro at 38/40/42 (PROTOCOL.md section 8, via
	 * the Arduino build imuFrom45). Raw sensor frame, whoever consumes it
	 * does the axis permutation, because each output mode wants a different
	 * one.
	 */
	in->ax = (int16_t)s16off(rep, 32);
	in->ay = (int16_t)s16off(rep, 34);
	in->az = (int16_t)s16off(rep, 36);
	in->gx = (int16_t)s16off(rep, 38);
	in->gy = (int16_t)s16off(rep, 40);
	in->gz = (int16_t)s16off(rep, 42);

	in->valid = true;
}

bool puck_input_get(int slot, struct puck_input *out)
{
	if (slot < 0 || slot >= PUCK_HID_SLOTS || !g_in[slot].valid) {
		return false;
	}
	*out = g_in[slot];
	return true;
}

void puck_input_dump(int slot)
{
	struct puck_input in;

	if (!puck_input_get(slot, &in)) {
		printk("input: slot %d has no report yet\n", slot);
		return;
	}

	printk("input slot%d: btn=%08x L(%6d,%6d) R(%6d,%6d) LT=%3u RT=%3u\n",
	       slot, (unsigned)in.buttons, in.lx, in.ly, in.rx, in.ry, in.lt,
	       in.rt);
}
