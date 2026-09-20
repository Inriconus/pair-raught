/*
 * Nintendo HD rumble, decoded into something a Steam Controller can feel.
 *
 * The console does not send "motor at 60%". It sends a packed, RELATIVE stream:
 * four bytes per motor per frame, carrying amplitude commands in 1/32 log2
 * units that step a running state up and down, with occasional absolute
 * presets. So the decode is stateful (frames only make sense in sequence)
 * and it has to run for every frame even when nothing is relayed onward.
 *
 * Ported from the Arduino build's mode_switch_pro.cpp, which measured all of
 * this against real hardware.
 */

#ifndef PUCK_RUMBLE_H
#define PUCK_RUMBLE_H

#include <stdint.h>

/* Build the exp2 level table. Call once at boot. */
void puck_rumble_init(void);

/* Forget a slot's running amplitudes, on a fresh host handshake where the
 * relative stream restarts and stale state would decode as noise.
 */
void puck_rumble_reset(int slot);

/*
 * Decode one motor's four rumble bytes, advancing that motor's state, and
 * return the PEAK motor level across the frame's updates.
 *
 * Peak rather than the final sample: a frame can pack three updates, and a
 * short pulse living in the middle of one would vanish if only the last were
 * taken. Idle and neutral frames still resolve to 0.
 */
uint16_t puck_rumble_decode(int slot, int motor, const uint8_t b[4]);

#endif /* PUCK_RUMBLE_H */
