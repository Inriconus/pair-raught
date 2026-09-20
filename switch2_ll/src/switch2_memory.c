/*
 * MEMORY reads.
 *
 * The console reads several flash "pages" out of a real controller during
 * bring-up and validates what comes back. The blobs here follow the captured
 * structure, but the per-unit sensor trim is NOMINAL rather than captured: see
 * the calibration pages below.
 *
 * THE CONSOLE VALIDATES THE CONTROLLER_INFO BLOB. Perturb 16 bytes of it and
 * the console completes the PAIR ceremony, encrypts, then hangs up with
 * reason=0x13 about 13 ms later and retries in a loop. Serving the captured
 * original restores normal bring-up. It is the only lever found that provably
 * changes the console's behaviour, so treat these bytes as load bearing.
 *
 * The Arduino firmware carries runtime-selectable "own" variants of most of
 * these pages for de-cloning, every one defaulting to off. Only the captured
 * forms are ported. Re-introduce the variants when there is a reason to.
 */

#include <zephyr/kernel.h>
#include <string.h>

#include "switch2_memory.h"

/* Addresses the console reads. */
#define ADDR_CONTROLLER_INFO 0x00013000UL

/*
 * CONTROLLER_INFO, the blob the console validates.
 *
 *   [0:2]    unclassified, load bearing (see the region bisect below)
 *   [2:16]   14-char serial. The template already carries the synthetic one;
 *              build_info() re-stamps it so the two cannot drift.
 *   [16:18]  unclassified, load bearing
 *   [18:20]  vendor id, little endian
 *   [20:22]  product id, little endian
 *   [22:25]  unclassified, load bearing
 *   [25:37]  FOUR RGB triples: body / buttons / left grip / right grip
 *   [37:64]  0xFF tail. The erased-flash convention used throughout
 *
 * A region bisect narrowed the validated bytes to [0:2], [16:18], [22:25] or
 * [25:37]. The serial, vendor/product and 0xFF tail were all perturbed without
 * triggering rejection.
 *
 * Colours avoid 0x00 and 0xFF in every channel, since 0xFF is this protocol's
 * "unset" convention. See the note on the colour table below.
 */
static const uint8_t CONTROLLER_INFO[64] = {
	0x01, 0x00, 0x48, 0x43, 0x50, 0x37, 0x31, 0x30, 0x39, 0x39, 0x38, 0x38, 0x37, 0x37, 0x30,
	0x31, 0x00, 0x00, 0x7e, 0x05, 0x66, 0x20, 0x01, 0x08, 0x02, 0x32, 0x32, 0x32, 0xaa, 0xaa,
	0xaa, 0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff
};

/*
 * The puck's own serial, not the captured unit's.
 *
 * Real serials are 14 chars: a 3-char model code then 11 digits. A value that
 * matches neither the charset nor the shape fails even a trivial format check.
 */
static const char SERIAL[15] = "HCP71099887701";

/* Identity, kept in step with what goes out in the advert. A reported
 * vendor/product that disagrees with the advertised one is a self-inflicted
 * confound. Joy-Con 2 Right, the working default.
 */
#define ID_VID 0x057E
#define ID_PID 0x2066

/*
 * Gyro calibration: a nominal scale followed by three per-axis offsets.
 *
 * The offsets are zero here on purpose. A capture carries the manufacturing
 * trim of one physical controller, measured in thousandths, which fingerprints
 * that unit and describes hardware this puck does not have. The puck's motion
 * is a replayed capture and its sticks come from a Steam Controller, so a
 * perfectly-trimmed device is the honest thing to declare.
 */
static const uint8_t MEM_00013040[16] = {
	0xec, 0xbb, 0xdb, 0x41, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*
 * ECHO PROBE. The console reads this page and writes its first 8 bytes
 * straight back inside its 0x0A/0x08 command, so `69 09 00 00` is not something
 * the console knows independently. It is this page reflected.
 */
static const uint8_t MEM_00013060[32] = {
	0x69, 0x09, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* Stick calibration.
 *
 * [40:49] is the calibration itself: three packed 12-bit pairs, the centre,
 * then the travel above it, then the travel below it, packed the same way as
 * pack_stick_xy() in switch2_gatt.c. It is NOMINAL for the reason given at
 * MEM_00013040: centre 2048 with 1600 of travel each way on both axes. The
 * replayed report stream rests at about (2123, 2013), well inside that.
 *
 * [0:40] is left as captured. Those fields are undecoded, and these are the
 * bytes the console accepted on hardware.
 */
static const uint8_t MEM_00013080[64] = {
	0x04, 0xac, 0xca, 0xaa, 0x53, 0x35, 0x55, 0x93, 0x30, 0x09, 0x93, 0x30, 0x09, 0xd2, 0x20, 0x0d,
	0xd2, 0x20, 0x0d, 0xcc, 0xac, 0xd9, 0xcc, 0xac, 0xd9, 0xd4, 0x42, 0x2d, 0xd4, 0x42, 0x2d, 0x8f,
	0xf4, 0x48, 0x8f, 0xf4, 0x48, 0x0f, 0xff, 0xff, 0x00, 0x08, 0x80, 0x40, 0x06, 0x64, 0x40, 0x06,
	0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* 0x000130C0 was the one address the console reads that had no blob, so it
 * fell through to the unknown-address default and got 64 bytes of 0xFF. The
 * captured page is erased apart from [40:49], which held the same bytes as the
 * stick calibration in MEM_00013080, so it carries the same nominal values.
 */
static const uint8_t MEM_000130C0[64] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x08, 0x80, 0x40, 0x06, 0x64, 0x40, 0x06,
	0x64, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

/* Accelerometer calibration: standard gravity, zero bias.
 *
 * 9.80665 is the defined standard value. A captured page carries whatever that
 * one unit measured, which is per-unit trim; see MEM_00013040.
 */
static const uint8_t MEM_00013100[24] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x41,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* User calibration area: entirely erased, which is what makes the console fall
 * back to the factory calibration above.
 */
static const uint8_t MEM_001FC040[64] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static bool lookup(uint32_t addr, const uint8_t **out, uint8_t *out_len)
{
	switch (addr) {
	case 0x00013040UL: *out = MEM_00013040; *out_len = sizeof(MEM_00013040); return true;
	case 0x00013060UL: *out = MEM_00013060; *out_len = sizeof(MEM_00013060); return true;
	case 0x00013080UL: *out = MEM_00013080; *out_len = sizeof(MEM_00013080); return true;
	case 0x000130C0UL: *out = MEM_000130C0; *out_len = sizeof(MEM_000130C0); return true;
	case 0x00013100UL: *out = MEM_00013100; *out_len = sizeof(MEM_00013100); return true;
	case 0x001FC040UL: *out = MEM_001FC040; *out_len = sizeof(MEM_001FC040); return true;
	default: return false;
	}
}

/*
 * CONTROLLER COLOURS. [25:37] is four RGB triples, confirmed on screen with a
 * red/green/blue/yellow test pattern:
 *
 *   [25:28] body   [28:31] buttons   [31:34] top accent bar   [34:37] BOTH grips
 *
 * Note the third slot is an accent strip, not a left grip, and the fourth
 * colours both grips together. The obvious reading of "four slots = body,
 * buttons, left grip, right grip" is wrong.
 *
 * GameCube indigo: the iconic combination is the indigo body, the green A
 * button and the yellow C-stick, so those take the three visible slots.
 *
 * No channel is 0x00 or 0xFF, which is this protocol's erased-flash "no data"
 * convention everywhere else. Pure FF0000/00FF00/0000FF left the on-screen icon
 * black: the console read those channels as unset and fell back to its default.
 */
static const uint8_t COLORS_RGBY[12] = {
	0x5c, 0x4b, 0x99,	/* body    INDIGO. The GameCube purple */
	0x43, 0xb0, 0x2a,	/* buttons GREEN   the A button */
	0xe8, 0xc0, 0x22,	/* accent  YELLOW. The C-stick */
	0x4a, 0x3d, 0x80	/* grips   INDIGO, slightly darker than the body */
};

static void build_info(uint8_t *content, uint8_t content_len)
{
	uint8_t copy_len = MIN(content_len, (uint8_t)sizeof(CONTROLLER_INFO));

	memcpy(content, CONTROLLER_INFO, copy_len);
	if (content_len > copy_len) {
		memset(content + copy_len, 0xFF, content_len - copy_len);
	}

	if (content_len >= 16) {
		memcpy(content + 2, SERIAL, 14);
	}
	if (content_len >= 22) {
		content[18] = ID_VID & 0xFF;
		content[19] = ID_VID >> 8;
		content[20] = ID_PID & 0xFF;
		content[21] = ID_PID >> 8;
	}

	/* The puck's colours, not the captured unit's near-black. */
	if (content_len >= 37) {
		memcpy(content + 25, COLORS_RGBY, sizeof(COLORS_RGBY));
	}
}

void switch2_memory_build(uint32_t addr, uint8_t *content, uint8_t content_len)
{
	const uint8_t *blob;
	uint8_t blob_len;

	memset(content, 0, content_len);

	if (addr == ADDR_CONTROLLER_INFO) {
		build_info(content, content_len);
		return;
	}

	if (lookup(addr, &blob, &blob_len)) {
		memcpy(content, blob, MIN(content_len, blob_len));
		if (content_len > blob_len) {
			/* Should never fire: the console asks for exactly the
			 * blob length every time (0x13040 len=16, 0x13060
			 * len=32, 0x13100 len=24). If it does, that blob needs
			 * page-sizing rather than silent 0xFF padding.
			 */
			printk("  *** SHORT BLOB addr=0x%08X: asked %u, have %u ***\n",
			       (unsigned)addr, content_len, blob_len);
			memset(content + blob_len, 0xFF, content_len - blob_len);
		}
		return;
	}

	/* Unknown address. 0xFF is the erased-flash convention and a safer
	 * default than zeros. An all-zero tail was previously confirmed to
	 * look anomalous to the console.
	 */
	memset(content, 0xFF, content_len);
	printk("  memory read: UNKNOWN addr=0x%08X: answering 0xFF fill\n",
	       (unsigned)addr);
}
