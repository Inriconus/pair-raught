/*
 * Pair-Raught: the Switch 2 BLE peripheral.
 *
 * The attribute table, the PAIR ceremony, key agreement, and the input report
 * stream. The console discovers this table by UUID, runs an eight step command
 * ceremony over it, derives a link key, encrypts, and then reads reports from
 * d5a9e01e.
 *
 * WHY THE HANDLES MATTER
 *
 * The console picks its write targets by UUID, so those work at any handle.
 * Notifications go the other way: the peripheral picks the handle reports
 * arrive on and the console gets no say, and a real Joy-Con 2 notifies from
 * 0x000E. So the
 * report characteristic (d5a9e01e) is declared first in the main service, and
 * the table is positioned so that:
 *
 *     0x000C  main service declaration
 *     0x000D  d5a9 characteristic declaration
 *     0x000E  d5a9 value          <-- reports notify from here
 *     0x000F  d5a9 CCCD           <-- console writes 0100
 *     0x0010  d5a9 679d descriptor<-- console writes 8500, the stream trigger
 *
 * On the SoftDevice that needed an empty "spacer" service at 0x000B, since its
 * reserved block ended at 0x000A. Zephyr numbers attributes differently, so no
 * spacer is assumed. switch2_gatt_dump() prints the handles the stack actually
 * assigned and switch2_gatt_check() says whether they landed. Add a spacer only
 * if the dump asks for one, rather than guessing.
 *
 * UUID byte order: Zephyr's BT_UUID_DECLARE_128 takes the 16 bytes in exactly
 * the little-endian order the Arduino arrays already use, so they are copied
 * across verbatim rather than retyped in display order.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/crypto.h>
/* Host-internal: no public API exists for injecting an out-of-band LTK.
 * See the include-path note in CMakeLists.txt.
 */
#include "keys.h"
#include <string.h>

#include "switch2_gatt.h"
#include "switch2_memory.h"
#include "switch2_store.h"
#include "puck_settings.h"

/* ---- UUIDs, byte-identical to BleSwitch2Poc.ino ------------------------- */

/* Main service ab7de9be-89fe-49ad-828f-118f09df7fd0. Ground truth from an
 * nRF Connect Mobile browse of a real Joy-Con 2. It differs from INPUT_REPORT
 * only in the last hex digit (fd0 vs fd2). The real controller groups all
 * eleven characteristics under this one service.
 */
#define UUID_SWITCH2_SERVICE BT_UUID_DECLARE_128( \
	0xd0, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82, \
	0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab)

/* The report channel. NOTIFY|READ. */
#define UUID_UNK_D5A9 BT_UUID_DECLARE_128( \
	0x42, 0xf4, 0x2b, 0x14, 0x67, 0x8b, 0x0c, 0xb2, \
	0xca, 0x4c, 0xfc, 0x2f, 0x1e, 0xe0, 0xa9, 0xd5)

/* Vibration, cc483f51-9258-427d-a939-630c31f72b05, the PRO CONTROLLER 2
 * value. This UUID differs per controller model, and the puck registers as a
 * Pro Controller, so exposing a Joy-Con's vibration characteristic would
 * contradict the identity in the rest of the table. It is read during
 * discovery, before any input.
 */
#define UUID_VIBRATION_PRO BT_UUID_DECLARE_128( \
	0x05, 0x2b, 0xf7, 0x31, 0x0c, 0x63, 0x39, 0xa9, \
	0x7d, 0x42, 0x58, 0x92, 0x51, 0x3f, 0x48, 0xcc)

#define UUID_COMMAND_WRITE BT_UUID_DECLARE_128( \
	0x05, 0xf0, 0xe5, 0x4f, 0xa5, 0x1e, 0x44, 0xaf, \
	0x6c, 0x4e, 0xb7, 0x8e, 0xc9, 0x4a, 0x9d, 0x64)

/* The handshake channel. The console's PAIR/LTK1 and PAIR/LTK2 writes are
 * 42 bytes each, so this must accept at least that. A 42-byte write to a
 * 33-byte characteristic is rejected with ATT 0x0D and the key exchange can
 * never complete, invisibly. 64 leaves headroom.
 */
#define UUID_UNK_65A7 BT_UUID_DECLARE_128( \
	0xff, 0x27, 0x6b, 0x37, 0x42, 0xa3, 0x78, 0x80, \
	0x61, 0x4a, 0xe7, 0xf1, 0xb3, 0x24, 0xa7, 0x65)

#define UUID_UNK_4147 BT_UUID_DECLARE_128( \
	0x8d, 0x9f, 0xf5, 0x5d, 0x3e, 0xd2, 0xf7, 0xa4, \
	0xf7, 0x4d, 0xae, 0xfd, 0x3d, 0x42, 0x47, 0x41)

#define UUID_COMMAND_RESPONSE BT_UUID_DECLARE_128( \
	0x6a, 0x83, 0x11, 0xb1, 0x15, 0x53, 0x0a, 0xa2, \
	0x36, 0x4d, 0xd8, 0xd9, 0x61, 0xa9, 0x65, 0xc7)

#define UUID_UNK_640C BT_UUID_DECLARE_128( \
	0x0b, 0x69, 0x2b, 0xaf, 0x6f, 0x42, 0xf3, 0xa7, \
	0x0c, 0x41, 0x88, 0x0e, 0x8e, 0xa5, 0x0c, 0x64)

#define UUID_UNK_D3BD BT_UUID_DECLARE_128( \
	0x80, 0x2a, 0x6d, 0x40, 0x6f, 0xf8, 0x15, 0xab, \
	0x41, 0x42, 0x1c, 0x84, 0xd2, 0x69, 0xbd, 0xd3)

#define UUID_UNK_FDE BT_UUID_DECLARE_128( \
	0xde, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82, \
	0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab)

#define UUID_UNK_FDF BT_UUID_DECLARE_128( \
	0xdf, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82, \
	0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab)

/* INPUT_REPORT (...fd2). Deliberately LAST: the console never subscribes it
 * and never writes it in any capture, so its position does not matter, and
 * giving the front position to d5a9 is what puts the reports on 0x000E.
 */
#define UUID_INPUT_REPORT BT_UUID_DECLARE_128( \
	0xd2, 0x7f, 0xdf, 0x09, 0x8f, 0x11, 0x8f, 0x82, \
	0xad, 0x49, 0xfe, 0x89, 0xbe, 0xe9, 0x7d, 0xab)

/* Non-standard descriptors that sit alongside the CCCD on the real device.
 * Purpose unknown; present to match its handle count, except 679d on d5a9,
 * which is where the console writes 0x0085 to start the report stream.
 */
#define UUID_DESC_679D BT_UUID_DECLARE_128( \
	0xcb, 0x6e, 0x48, 0x80, 0xdf, 0x95, 0x57, 0x95, \
	0xee, 0x4d, 0x24, 0x5a, 0x10, 0x55, 0x9d, 0x67)

#define UUID_DESC_B746 BT_UUID_DECLARE_128( \
	0x79, 0xf9, 0xa4, 0xed, 0xbb, 0xe3, 0xd2, 0x9c, \
	0x5b, 0x49, 0x58, 0xf3, 0x8c, 0xdf, 0x46, 0xb7)

/* Second service, 00c5af5d-1964-4e30-8f51-1956f96bd280. Its three
 * characteristics are found by UUID over a 0x0001-0xFFFF Read By Type, so they
 * work at any handle. Only their position RELATIVE to the main service
 * matters, and the main service needs the low numbers.
 */
#define UUID_SERVICE1 BT_UUID_DECLARE_128( \
	0x80, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f, \
	0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00)

#define UUID_S1_CHAR_R1 BT_UUID_DECLARE_128( \
	0x81, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f, \
	0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00)

#define UUID_S1_CHAR_W BT_UUID_DECLARE_128( \
	0x82, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f, \
	0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00)

#define UUID_S1_CHAR_R2 BT_UUID_DECLARE_128( \
	0x83, 0xd2, 0x6b, 0xf9, 0x56, 0x19, 0x51, 0x8f, \
	0x30, 0x4e, 0x64, 0x19, 0x5d, 0xaf, 0xc5, 0x00)

/* ---- Attribute value storage ------------------------------------------- */

#define REPORT_LEN 63

static uint8_t v_d5a9[REPORT_LEN];	/* a READ must return the full 63 bytes */
static uint8_t v_fd2[REPORT_LEN];
static uint8_t v_fde[1];

/* The extra descriptors are two bytes and MUST be variable length. The Arduino
 * port hit this: a fixed-length descriptor bounces the console's 2-byte 0x0085
 * write with ATT 0x0D (Invalid Attribute Value Length), invisibly, and the
 * stream never starts.
 */
static uint8_t d_d5a9_679d[2];
static uint8_t d_fd2_679d[2];
static uint8_t d_fde_679d[2];
static uint8_t d_cmdresp_b746[2];
static uint8_t d_640c_b746[2];
static uint8_t d_d3bd_b746[2];

/* bd281's value is a POINTER TO bd282, not an opaque constant:
 *
 *     04 00 05 00 01 01 00
 *     ^^^^^ ^^^^^
 *     0x0004 0x0005  = bd282's DECLARATION and VALUE handles on a real device
 *
 * The console reads bd281 wherever it finds it, parses those two little-endian
 * handles out, and writes to the second one. Copying a real device's bytes
 * verbatim therefore aims it at 0x0005, which is not bd282 in this table, so
 * the value is filled in at runtime by switch2_gatt_check().
 */
static uint8_t v_bd281[7] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00 };

/*
 * bd282, the handle bd281 points at, and the one the console WRITES during
 * its reconnect sequence (the analogue of handle 0x0005 on a real device; the
 * observed write is 2 bytes, 0x0100).
 *
 * It needs its own storage. Sharing bd283's buffer put the console's write on
 * top of the identity value and rejected it with ATT 0x0D (Invalid Attribute
 * Value Length), that buffer being a single byte.
 */
static uint8_t v_bd282[4];

/*
 * bd283, a per-device identity of 8 bytes, read by the console before any CCCD
 * activity (attribute 0x0006 on a real device, the "Read By Type -> 0x0006"
 * step in the reconnect capture).
 *
 * It has to be this device's own, and unique. The console tracks it as a
 * per-device identity and rejects a collision with a controller it already
 * knows: serving a real Joy-Con's value while that Joy-Con was registered on
 * the same console dropped the puck at LTK2 every cycle.
 *
 * So it is derived from this chip's DEVICEID at init, not baked in. A constant
 * here would put every puck ever built on the same identity, and two of them
 * on one console would collide the same way.
 */
static uint8_t v_bd283[8];

/* ---- Generic handlers --------------------------------------------------- */

static void hexdump(const char *tag, uint16_t handle, const void *buf, uint16_t len)
{
	const uint8_t *p = buf;

	printk("%s handle=0x%04X len=%u data=", tag, handle, len);
	for (uint16_t i = 0; i < len && i < 64; i++) {
		printk("%02x", p[i]);
	}
	if (len > 64) {
		printk("...");
	}
	printk("\n");
}

/* Defined with the report stream below; the stream trigger lands in wr(). */
static void stream_start(struct bt_conn *conn);
static void force_report_ccc(struct bt_conn *conn);

static ssize_t rd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		  void *buf, uint16_t len, uint16_t offset)
{
	const struct switch2_val *v = attr->user_data;

	printk("ATT READ  handle=0x%04X off=%u\n",
	       bt_gatt_attr_get_handle(attr), offset);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, v->data, v->len);
}

/*
 * Accept every write, store what fits, and log it.
 *
 * Store even the attributes nothing acts on yet. The console reads some of
 * them back, and a silently discarded write is the hardest kind of failure to
 * see from outside.
 */
static ssize_t wr(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	struct switch2_val *v = attr->user_data;
	uint16_t handle = bt_gatt_attr_get_handle(attr);

	hexdump("ATT WRITE", handle, buf, len);

	if (handle == switch2_gatt_trigger_handle()) {
		printk("  <<< THIS IS THE STREAM TRIGGER (d5a9 679d descriptor)\n");
		/* The console writes 0x0085 here to start the stream, then hangs
		 * up (reason=0x13) if nothing arrives.
		 */
		stream_start(conn);
	}

	if (offset + len > v->cap) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(v->data + offset, buf, len);
	v->len = MAX(v->len, offset + len);
	return len;
}

/* ---- Handshake ---------------------------------------------------------- */

/* Defined below; replies notify from one of its attributes. */
static struct bt_gatt_attr switch2_attrs[];

/*
 * Indices into switch2_attrs[]. BT_GATT_CHARACTERISTIC expands to two
 * attributes and BT_GATT_CCC to one, so these are counted from the table
 * below; the runtime dump confirms them (index + 0x000C == handle).
 */
#define ATTR_D5A9_VALUE   2	/* 0x000E */
#define ATTR_D5A9_CCC     3	/* 0x000F */
#define ATTR_D5A9_679D    4	/* 0x0010 */
#define ATTR_CMDRESP_VALUE 14	/* 0x001A */

/*
 * The three command bodies the console expects during the pre-encryption
 * handshake. Each is a real controller's RESPONSE to a matching Switch write,
 * byte-verified against the plaintext of two clean fresh-pairing captures AND
 * the decrypted reconnect capture, all three agreeing.
 *
 * Total reply sizes are 9 / 20 / 32 bytes: the 8-byte header plus these.
 * Answering with a bare 8-byte ack instead was measurably wrong: the console
 * worked through 0x07 -> 0x02 -> 0x10 and then stopped dead, i.e. it stalled at
 * the first reply where content was owed and none was sent.
 */
static const uint8_t ANNOUNCED_CMD07[1] = { 0x00 };

/* [3] is a side/model discriminator: 0x00 = left Joy-Con, 0x01 = right.
 * 0x01 is correct for the Joy-Con 2 Right identity this build advertises
 * (VID 0x057E / PID 0x2066). It was one of the seven things that had to be
 * true on 2026-08-13, the day the console first consumed the report stream, so
 * do not "fix" it without changing the advertised identity to match.
 */
static const uint8_t ANNOUNCED_CMD10[12] = { 0x02, 0x01, 0x04, 0x01, 0x0c, 0x00,
					     0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t ANNOUNCED_CMD16[24] = { 0 };

/*
 * Reply framing: [cmd][0x01][0x01][subcmd][0x10][0x78][0x00][0x00][payload].
 *
 * The type byte is 0x10 for EVERY notification, including the MEMORY read. The
 * one capture showing 0x20 there has a BAD CRC, and each of its seven
 * differences from the two good captures is a single-bit flip, 0x10 -> 0x20
 * among them.
 */
static void send_response(struct bt_conn *conn, uint8_t cmd, uint8_t subcmd,
			  const uint8_t *payload, uint8_t len)
{
	uint8_t buf[8 + 80];
	int err;

	if (len > 80) {
		len = 80;
	}

	buf[0] = cmd;
	buf[1] = 0x01;
	buf[2] = 0x01;
	buf[3] = subcmd;
	buf[4] = 0x10;
	buf[5] = 0x78;
	buf[6] = 0x00;
	buf[7] = 0x00;
	if (len) {
		memcpy(buf + 8, payload, len);
	}

	/* The return value is checked and always reported. A notification is
	 * dropped silently if the peer has not subscribed that CCCD or the TX
	 * queue is full, which is indistinguishable from "sent and ignored"
	 * unless it is logged.
	 */
	err = bt_gatt_notify(conn, &switch2_attrs[ATTR_CMDRESP_VALUE], buf, 8 + len);
	printk("  reply cmd=0x%02X sub=0x%02X len=%u -> %d%s\n",
	       cmd, subcmd, 8 + len, err, err ? "  <<< NOT SENT" : "");
}

#define CMD_MEMORY      0x02
#define SUB_MEMORY_READ 0x04
#define CMD_PAIR         0x15
#define SUB_PAIR_SET_MAC 0x01
#define SUB_PAIR_LTK2    0x02
#define SUB_PAIR_FINISH  0x03
#define SUB_PAIR_LTK1    0x04

/*
 * The controller announcing its OWN address. Prefix (01 04 01) copied verbatim
 * from the real capture, purpose unknown. The tail is this device's advertised
 * address in wire order and must match what goes out in the advert; on a real
 * device it is that Joy-Con's own BT address byte for byte.
 */
static uint8_t ANNOUNCED_SET_MAC[9] = { 0x01, 0x04, 0x01 };

/*
 * Both the advertised address and the SET_MAC tail come from here, so they are
 * physically incapable of drifting apart. On a real device the SET_MAC tail IS
 * the controller's address, byte for byte.
 */
const uint8_t *puck_addr(void)
{
	static uint8_t addr[6];
	static bool built;

	if (!built) {
		uint32_t h = NRF_FICR->DEVICEID[0] * 0x9E3779B1u ^
			     NRF_FICR->DEVICEID[1];

		addr[0] = (uint8_t)(h);
		addr[1] = (uint8_t)(h >> 8);
		addr[2] = (uint8_t)(h >> 16);

		/* Never let the tail land on all-zero or all-ones; both are
		 * shapes a console could plausibly treat as unset.
		 */
		if ((addr[0] | addr[1] | addr[2]) == 0x00) {
			addr[0] = 0x01;
		} else if ((addr[0] & addr[1] & addr[2]) == 0xFF) {
			addr[0] = 0xFE;
		}

		const uint8_t oui[3] = { PUCK_OUI_BYTES };

		memcpy(addr + 3, oui, sizeof(oui));
		memcpy(ANNOUNCED_SET_MAC + 3, addr, sizeof(addr));

		/* The bd283 identity, from the same source. A second hash so
		 * it does not simply restate the address the console already
		 * has.
		 */
		uint32_t g = NRF_FICR->DEVICEID[1] * 0x85EBCA6Bu ^
			     NRF_FICR->DEVICEID[0];

		for (int i = 0; i < 4; i++) {
			v_bd283[i] = (uint8_t)(h >> (i * 8));
			v_bd283[i + 4] = (uint8_t)(g >> (i * 8));
		}

		built = true;
	}
	return addr;
}

/*
 * The controller's half of the link-key agreement.
 *
 * Confirmed via THREE real captures (two pairing sessions of the right Joy-Con,
 * one of a physically different left Joy-Con): this 17-byte value is
 * byte-for-byte identical across all three, so it is a universal fixed
 * constant, not per-device or per-session crypto.
 */
static const uint8_t ANNOUNCED_LTK1[17] = {
	0x01, 0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
	0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10
};

static const uint8_t ANNOUNCED_PAIR_FINISH[1] = { 0x01 };

/*
 * The derived link key, in AES / most-significant-octet-first order.
 *
 *     LTK = reverse_bytes( <Switch's LTK1 body> XOR <controller's LTK1 body> )
 *
 * where each body is the 16 bytes after the 1-byte marker of a 0x15/0x04
 * message. The real LL_ENC_REQ carries EDIV=0 and Rand=0, an out-of-band
 * key with no SMP and no key lookup, so the GATT PAIR ceremony IS the key
 * agreement.
 */
static uint8_t g_link_key_msb[16];	/* AES / most-significant-octet first */
static uint8_t g_link_key_wire[16];	/* SMP wire order, what the host wants */
static bool g_have_link_key;

/*
 * Hand the derived key to the host so bt_smp_request_ltk() can answer the
 * controller's LTK request.
 *
 * Zephyr serves conn->le.keys->ltk.val when EDIV and Rand are both zero and the
 * entry is typed BT_KEYS_LTK_P256, which is exactly the shape of the console's
 * LL_ENC_REQ. That value is copied straight into the HCI LTK Request Reply, so
 * it must be in SMP WIRE ORDER (least significant octet first), i.e. the XOR
 * result WITHOUT the reversal. The reversed copy is kept separately because the
 * LTK2 challenge feeds it to AES, which wants most-significant-octet first.
 *
 * Timing matters: this runs on the Switch's PAIR/LTK1 write, two command
 * exchanges (~60-90 ms) before LL_ENC_REQ arrives, so the key is in place well
 * before it is needed.
 */
static void install_link_key(struct bt_conn *conn, const uint8_t wire[16])
{
	struct bt_conn_info info;
	struct bt_keys *keys;

	if (bt_conn_get_info(conn, &info) || info.type != BT_CONN_TYPE_LE) {
		printk("  cannot install link key: no LE conn info\n");
		return;
	}

	keys = bt_keys_get_addr(info.id, info.le.dst);
	if (!keys) {
		printk("  cannot install link key: key slot allocation failed "
		       "(CONFIG_BT_MAX_PAIRED too small?)\n");
		return;
	}

	memcpy(keys->ltk.val, wire, 16);
	memset(keys->ltk.rand, 0, sizeof(keys->ltk.rand));
	memset(keys->ltk.ediv, 0, sizeof(keys->ltk.ediv));
	keys->enc_size = 16;
	bt_keys_add_type(keys, BT_KEYS_LTK_P256);

	/*
	 * The key bytes are never printed. They authenticate this puck to its
	 * console, serial logs get pasted into bug reports, and anyone holding
	 * the key and the console's address could use it to pose as this
	 * controller.
	 */
	printk("  link key installed\n");
}

static void derive_link_key(struct bt_conn *conn, const uint8_t *switch_ltk1_body)
{
	uint8_t wire[16];

	/* XOR in wire order (least-significant octet first, as both halves
	 * arrive over GATT), then reverse for the AES form.
	 */
	for (uint8_t i = 0; i < 16; i++) {
		wire[i] = switch_ltk1_body[i] ^ ANNOUNCED_LTK1[1 + i];
	}
	for (uint8_t i = 0; i < 16; i++) {
		g_link_key_msb[i] = wire[15 - i];
	}
	memcpy(g_link_key_wire, wire, 16);
	g_have_link_key = true;

	/* Straight to flash. The console will expect this exact key on its next
	 * connection, and that connection may well follow a dock power cut.
	 */
	switch2_store_save_key(wire);

	printk("  link key derived (not printed, see install_link_key())\n");

	install_link_key(conn, wire);
}

/*
 * The PAIR ceremony, answered reactively. Ground truth is the decrypted
 * reconnect capture, where each of these is a write from the Switch answered by
 * exactly one notification from the controller:
 *
 *   0x15/0x01 SET_MAC -> controller's own address
 *   0x15/0x04 LTK1    -> the universal constant, and the Switch's half arrives here
 *   0x15/0x02 LTK2    -> a CHALLENGE, see below
 *   0x15/0x03 FINISH  -> single byte 0x01
 *
 * A generic empty ack is not what a real controller replies, and SET_MAC is
 * the next thing the Switch writes after the block already answered, so a wrong
 * reply stalls the link right where it needs to get through.
 */
static void handle_pair(struct bt_conn *conn, uint8_t subcmd,
			const uint8_t *d, uint16_t len)
{
	switch (subcmd) {
	case SUB_PAIR_SET_MAC:
		send_response(conn, CMD_PAIR, SUB_PAIR_SET_MAC,
			      ANNOUNCED_SET_MAC, sizeof(ANNOUNCED_SET_MAC));
		return;

	case SUB_PAIR_LTK1:
		/* This write carries the Switch's half of the link-layer key. */
		if (len >= 42) {
			derive_link_key(conn, d + 26);
		} else {
			printk("  PAIR/LTK1 too short (len=%u, need 42): no key\n", len);
		}
		send_response(conn, CMD_PAIR, SUB_PAIR_LTK1,
			      ANNOUNCED_LTK1, sizeof(ANNOUNCED_LTK1));
		return;

	case SUB_PAIR_LTK2: {
		/*
		 * THE REAL GATE. The Switch's LTK2 write is a random 16-byte
		 * challenge, and the controller must answer with it encrypted
		 * under the link key derived from the LTK1 pair:
		 *
		 *   response = AES-128-ECB-Encrypt(key = LTK(MSO-first),
		 *                                  reverse(challenge))
		 *
		 * Ciphertext sent as-is, no reversal. Verified as an exact
		 * 16-byte match against the captured pair, which also confirms
		 * derive_link_key(): the key that decrypts the link is the one
		 * that answers here.
		 *
		 * A fixed constant, which is what the reference PC repos
		 * hardcode, fails the proof of knowledge and the Switch
		 * terminates ~43 ms later with nothing on screen. This step
		 * cannot be faked, which is why third-party pads need
		 * Switch-2-specific firmware rather than generic HID.
		 */
		uint8_t plain[16], cipher[16], resp[17];
		int err;

		if (!g_have_link_key || len < 42) {
			printk("  PAIR/LTK2 cannot answer challenge "
			       "(haveKey=%d len=%u): the Switch will reject us\n",
			       g_have_link_key, len);
			return;
		}

		for (uint8_t i = 0; i < 16; i++) {
			plain[i] = d[26 + 15 - i];	/* challenge, reversed */
		}

		err = bt_encrypt_be(g_link_key_msb, plain, cipher);
		if (err) {
			printk("  PAIR/LTK2 AES failed (%d)\n", err);
			return;
		}

		resp[0] = 0x01;	/* same marker byte a real controller uses */
		memcpy(resp + 1, cipher, 16);

		printk("  PAIR/LTK2 challenge answered\n");

		send_response(conn, CMD_PAIR, SUB_PAIR_LTK2, resp, sizeof(resp));
		return;
	}

	case SUB_PAIR_FINISH:
		send_response(conn, CMD_PAIR, SUB_PAIR_FINISH,
			      ANNOUNCED_PAIR_FINISH, sizeof(ANNOUNCED_PAIR_FINISH));
		return;

	default:
		printk("  PAIR subcmd 0x%02X not handled\n", subcmd);
		return;
	}
}

/*
 * Answer a MEMORY read.
 *
 * Request payload: [length][0x7e][0x00][0x00][addr, 4 bytes little endian].
 * Reply payload:   [length][0x00][0x00][0x00][addr, 4 bytes LE][content...].
 *
 * Bytes 1-3 of the reply are all zero. Do not mirror the request framing back
 * (0x7e,0x00,0x00), and do not read them off a frame with a bad CRC, which is
 * where 0x00,0x01,0x00 came from: that 0x01 is the same single-bit corruption
 * that produced the bogus 0x20 type byte. Two clean captures and the decrypted
 * reconnect capture all show 0x00,0x00,0x00.
 */
static void handle_memory_read(struct bt_conn *conn, const uint8_t *req, uint16_t req_len)
{
	uint8_t inner[8 + 64];
	uint8_t content_len;
	uint32_t addr;

	if (req_len < 8) {
		printk("  memory read: payload too short (%u)\n", req_len);
		return;
	}

	addr = (uint32_t)req[4] | ((uint32_t)req[5] << 8) |
	       ((uint32_t)req[6] << 16) | ((uint32_t)req[7] << 24);
	content_len = req[0] > 64 ? 64 : req[0];

	printk("  memory read: addr=0x%08X len=%u\n", (unsigned)addr, req[0]);

	inner[0] = req[0];
	inner[1] = 0x00;
	inner[2] = 0x00;
	inner[3] = 0x00;
	inner[4] = addr & 0xFF;
	inner[5] = (addr >> 8) & 0xFF;
	inner[6] = (addr >> 16) & 0xFF;
	inner[7] = (addr >> 24) & 0xFF;

	switch2_memory_build(addr, inner + 8, content_len);

	send_response(conn, CMD_MEMORY, SUB_MEMORY_READ, inner, 8 + content_len);
}

/* ---- The report stream --------------------------------------------------- */

#include "motion_replay.h"

/*
 * Real 63-byte input report, captured idle. Used as a TEMPLATE rather than a
 * zero fill, so every undecoded field starts at a value real hardware actually
 * emits.
 */
static const uint8_t INPUT_REPORT_TEMPLATE[REPORT_LEN] = {
	0x06, 0x18, 0x00, 0x00, 0x07, 0x68, 0xb8, 0x7c, 0x38, 0xff, 0xff, 0x00, 0x00, 0x5f, 0x00, 0x28,
	0x69, 0xb0, 0x00, 0x0e, 0x03, 0x30, 0x03, 0x96, 0xef, 0xbf, 0x86, 0xf6, 0xd7, 0x38, 0xcc, 0x2d,
	0xa5, 0x49, 0x10, 0x60, 0xf9, 0xeb, 0xdf, 0x3e, 0x40, 0x17, 0x64, 0xc2, 0xff, 0xf3, 0xfa, 0xab,
	0xbf, 0x10, 0x81, 0xbb, 0x84, 0x26, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* [8] = (battery << 4) | mode. Low nibble 8 = full report mode (measured: it
 * flipped 0x0 -> 0x8 exactly when CMD_FEATURE enabled full reporting). High
 * nibble is the charge level. 0x88 is what showed FULL on the console icon.
 */
#define BATTERY_BYTE 0x88

static struct bt_conn *g_stream_conn;
static bool g_streaming;
/* Set on a connection to a console that has already registered this puck, and
 * so may never ask for the stream again. Cleared once streaming starts.
 */
static bool g_reconnect_stream;
static int64_t g_conn_at;
static uint8_t g_report_seq;
static uint16_t g_motion_frame;
static uint16_t g_imu_ts12;	/* synthesised 12-bit IMU sample clock */

/*
 * Buttons live in a little-endian field at report[2:4].
 * BTN_SR = byte[3] bit6, BTN_SL = byte[3] bit7.
 *
 * SL+SR is the working default from the Arduino firmware, the configuration
 * in which the console assigned a player LED, sent a rumble preset, and
 * rendered the stick on screen.
 */
#define BTN_SR 0x4000U
#define BTN_SL 0x8000U
#define BUTTON_MASK (BTN_SL | BTN_SR)

/*
 * HOME, byte[3] bit0.
 *
 * HOME is the button a person presses to wake a console, but the wake does not
 * travel in this report. It cannot: the console is asleep and there is no
 * connection to carry one. Pressing HOME wakes the CONTROLLER's radio, which
 * then advertises with the wake byte set, and the advertisement is what the
 * console acts on. See build_adv_data() in main.c.
 *
 * So this is armed only by the 'h' console key, as a way to send a HOME press
 * to a console that is already awake and connected. The wake macro does not
 * use it.
 *
 * Held for 8 TRANSMITTED reports so the console sees a real press-release edge.
 * The hold must be counted in reports actually sent, not in loop passes. The
 * old firmware burned the whole window in eight iterations of a spin loop,
 * producing an edge far too short for the console to notice.
 */
#define BTN_HOME 0x0100U
#define HOME_HOLD_REPORTS 8

static bool g_wake_armed;
static uint8_t g_home_hold;

/*
 * BUTTON POLICY. Deliberate, not continuous.
 *
 * A real Joy-Con presses nothing on a normal reconnect: every report in the
 * decrypted capture carries [2]=00 [3]=00. Two moments need presses:
 *
 *   REGISTER  Change Grip/Order asks for an SL+SR press to confirm the slot
 *             and its position. Continuous while registering.
 *   UNLOCK    after a wake the console shows a lock screen that needs one
 *             button pressed several times (wake.presses, four by default) or
 *             it goes back to sleep. Discrete press/release cycles, then stop.
 *
 * Anything else: no buttons at all.
 */
enum press_mode {
	PRESS_NONE = 0,
	PRESS_REGISTER,
	PRESS_UNLOCK,
};

static enum press_mode g_press_mode = PRESS_REGISTER;

/*
 * Unlock timing, all counted in TRANSMITTED reports at ~30 ms each. Defaults
 * in puck_settings.c, adjustable from the config page.
 *
 * Pressing as soon as the link came up registered nothing: the console had
 * barely woken and the lock screen was not up yet. So wait for it, press at a
 * human speed, and hold the link open afterwards so the console can act on it.
 */
#define UNLOCK_DELAY     (g_cfg.unlock_delay)
#define UNLOCK_PRESSES   (g_cfg.unlock_presses)
#define UNLOCK_DOWN      (g_cfg.unlock_down)
#define UNLOCK_UP        (g_cfg.unlock_up)
#define UNLOCK_SETTLE    (g_cfg.unlock_settle)

/*
 * ONE button, pressed repeatedly. The lock screen wants the same button each
 * time, not a mash: setting every bit in the field at once is many different
 * buttons simultaneously, and it does not clear.
 *
 * Only BTN_B's position is documented (byte[2] bit0). The other face buttons
 * are presumably neighbouring bits but have never been confirmed, so the
 * candidates are sweepable at runtime with `u` rather than baked in.
 *
 * Safe at 16 bits: the field is [2:4], and only a wider mask would reach the
 * constant at [4] or the packed stick at [5].
 */
static const uint16_t UNLOCK_CANDIDATES[] = {
	0x0001,	/* B    byte[2] bit0, the one position we have documented */
	0x0002,	/* presumably A */
	0x0004,	/* presumably X */
	0x0008,	/* presumably Y */
	0x0100,	/* HOME byte[3] bit0 */
};
static uint8_t g_unlock_idx;	/* which candidate .u. last selected */
#define UNLOCK_MASK (g_cfg.unlock_mask)

void switch2_gatt_cycle_unlock_button(void)
{
	g_unlock_idx = (g_unlock_idx + 1) % ARRAY_SIZE(UNLOCK_CANDIDATES);
	g_cfg.unlock_mask = UNLOCK_CANDIDATES[g_unlock_idx];
	puck_settings_save();
	printk("\n  unlock button -> 0x%04X (candidate %u of %u, saved)\n",
	       g_cfg.unlock_mask, g_unlock_idx + 1,
	       (unsigned)ARRAY_SIZE(UNLOCK_CANDIDATES));
}

static uint8_t g_unlock_left;
static uint16_t g_unlock_tick;

void switch2_gatt_begin_unlock(void)
{
	g_press_mode = PRESS_UNLOCK;
	g_unlock_left = g_cfg.unlock_enabled ? UNLOCK_PRESSES : 0;
	g_unlock_tick = 0;

	if (g_cfg.unlock_enabled) {
		printk("\n*** UNLOCK: pressing %u times to clear the lock "
		       "screen ***\n", UNLOCK_PRESSES);
	} else {
		/*
		 * Unlock disabled: wake the console but leave the lock screen
		 * up. The sequence still runs so the link is released on
		 * schedule. A puck that holds the link is a console that can
		 * never sleep again.
		 */
		printk("\n*** WAKE: unlock disabled, leaving the lock screen "
		       "up ***\n");
	}
}

void switch2_gatt_set_registering(bool on)
{
	g_press_mode = on ? PRESS_REGISTER : PRESS_NONE;
	printk("  button policy -> %s\n", on ? "REGISTER (SL+SR)" : "NONE");
}

/* Set when advertising was started AS a wake action, so the unlock sequence
 * runs once the console has actually opened the report stream.
 */
static bool g_wake_pending;

void switch2_gatt_set_wake_pending(bool on)
{
	g_wake_pending = on;
}

void switch2_gatt_arm_wake(void)
{
	g_wake_armed = true;
	printk("\n*** WAKE ARMED: HOME will be pressed on the next report ***\n");
}

/* Advances only on a TRANSMITTED report, and is reset when the stream opens, so
 * every connection presents the same button phase from the same instant.
 * Free-running it against the console's cadence made the outcome depend on
 * where the counter happened to have wandered: "rare alignment, minutes of
 * nothing, then a sudden good run".
 */
static uint32_t g_auto_tick;

/* 12-bit packed stick, two axes in three bytes. Currently unused: the puck
 * sends no synthetic stick input. This is what the RF path will feed the Steam
 * Controller's actual stick through, so it stays.
 *
 * The calibration handed to the console in MEM_00013080[40..48] declares centre
 * (2048, 2048) and 1600 of travel each way, so 448..3648 on both axes. Anything
 * outside that is a reading real hardware cannot produce.
 */
__maybe_unused static void pack_stick_xy(uint8_t *out3, uint16_t x12, uint16_t y12)
{
	uint32_t v = ((uint32_t)(y12 & 0xFFF) << 12) | (x12 & 0xFFF);

	out3[0] = v & 0xFF;
	out3[1] = (v >> 8) & 0xFF;
	out3[2] = (v >> 16) & 0xFF;
}

static void build_report(uint8_t *report, uint16_t ts_next, uint16_t ts_step)
{
	memcpy(report, INPUT_REPORT_TEMPLATE, REPORT_LEN);

	/* [0] is an 8-BIT counter. [1] is a constant 0x18, verified 0x18 in
	 * all 283 real reports, which is why this is not a 16-bit counter.
	 */
	report[0] = g_report_seq;

	/* Replay real motion over [4:56]. A frozen sample is a controller whose
	 * IMU is perfectly dead, which the console notices.
	 *
	 * Do not narrow the span to [16:56] to protect the non-IMU bytes. The
	 * real device's later frames carry exactly the values the replay
	 * produces, so those fields are dynamic on real hardware too.
	 */
	memcpy(report + MOTION_OFF, MOTION_REPLAY[g_motion_frame], MOTION_LEN);

	/* After the memcpy: the replay covers [4:56] and would otherwise
	 * overwrite this with whatever the captured frame held. The console
	 * acts on it, and reported the pad as out of battery and hung up.
	 */
	report[8] = BATTERY_BYTE;

	/*
	 * SYNTHESISE the IMU clock rather than replaying it.
	 *
	 *   ts12    = ((report[17] & 0x0F) << 8) |  report[16]
	 *   delta12 = ((report[18] & 0x0F) << 4) | (report[17] >> 4)
	 *
	 * The clock is self-checking: ts12[i] - ts12[i-1] == delta12[i] holds
	 * on all 255 pairs in the capture. But the replay is a 256-frame loop,
	 * and at the seam ts12 steps by 32 while delta12 still claims 48. One
	 * report that contradicts itself every ~7.7 s, which is exactly how
	 * often the console dropped the link once off the pairing screen.
	 *
	 * So keep the replay's motion data and generate the clock at 48 ticks
	 * per report, the modal step in the capture. It is the one field the
	 * console can validate knowing nothing about the hardware, just by
	 * reading the report against itself.
	 */
	report[16] = ts_next & 0xFF;
	report[17] = ((ts_next >> 8) & 0x0F) | ((ts_step & 0x0F) << 4);
	report[18] = (ts_step >> 4) & 0x0F;

	/*
	 * SL+SR, 13 reports down then 13 up (~390 ms each at 30 ms/report).
	 *
	 * OR'd into the existing bytes rather than assigned: the rest of the
	 * field is non-zero in the captured template and its meaning is
	 * unknown, so clobbering it would change more than the button.
	 *
	 * Keep it strictly inside [2:4]. A 32-bit OR at [2] spans [2:6], where
	 * any mask bit above 0xFFFF corrupts the constant 0x07 at [4] and the
	 * packed stick at [5].
	 */
	{
		uint16_t v = (uint16_t)report[2] | ((uint16_t)report[3] << 8);
		bool any = false;

		if (g_press_mode == PRESS_REGISTER) {
			/* Continuous while the console is registering. */
			if (((g_auto_tick / 13) % 2) == 0) {
				v |= BUTTON_MASK;
				any = true;
			}
		} else if (g_press_mode == PRESS_UNLOCK && g_unlock_left > 0 &&
			   g_unlock_tick >= UNLOCK_DELAY) {
			uint16_t t = g_unlock_tick - UNLOCK_DELAY;

			if ((t % (UNLOCK_DOWN + UNLOCK_UP)) < UNLOCK_DOWN) {
				v |= UNLOCK_MASK;
				any = true;
			}
		}

		/* Start the HOME press on the first report after arming. */
		if (g_wake_armed && g_home_hold == 0) {
			g_home_hold = HOME_HOLD_REPORTS;
			g_wake_armed = false;
			printk("  sending HOME press (%u reports)\n", HOME_HOLD_REPORTS);
		}
		if (g_home_hold > 0) {
			v |= BTN_HOME;
			any = true;
		}

		if (any) {
			report[2] = v & 0xFF;
			report[3] = (v >> 8) & 0xFF;
		}
	}

	/*
	 * NO SYNTHETIC STICK INPUT.
	 *
	 * A real Joy-Con's reconnect reports carry no input at all. The stick
	 * belongs to the Steam Controller over RF, so the replayed capture's
	 * resting value is what goes out here.
	 */
}

/*
 * Stream at ~30 ms.
 *
 * The counters advance ONLY on a successful notify. Advancing them on a skipped
 * or failed send emits a report whose timestamp has jumped by a multiple of 48
 * while delta12 still claims 48, the same self-contradiction the synthesised
 * clock exists to prevent.
 */
static void report_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (1) {
		uint8_t report[REPORT_LEN];
		const uint16_t ts_step = 48;
		uint16_t ts_next;
		int err;

		if (!g_streaming && g_reconnect_stream && g_stream_conn &&
		    (k_uptime_get() - g_conn_at) > 1500) {
			/* A registered console that reconnects does not always
			 * re-run the handshake or re-arm anything. It just
			 * waits for reports. Give it 1.5 s to speak first (a
			 * reconnect that DOES re-handshake will have started
			 * the stream through the normal trigger by then), then
			 * open the stream from this side.
			 */
			printk("\n  reconnect: console has not asked, opening the "
			       "stream ourselves\n");
			force_report_ccc(g_stream_conn);
			stream_start(g_stream_conn);
		}

		if (!g_streaming || !g_stream_conn) {
			k_sleep(K_MSEC(50));
			continue;
		}

		ts_next = (g_imu_ts12 + ts_step) & 0x0FFF;
		build_report(report, ts_next, ts_step);

		err = bt_gatt_notify(g_stream_conn,
				     &switch2_attrs[ATTR_D5A9_VALUE],
				     report, sizeof(report));
		if (err) {
			/* Do NOT advance anything on a failure. */
			static uint32_t fails;

			if ((fails++ % 32) == 0) {
				printk("report notify failed (%d) x%u\n", err, fails);
			}
			k_sleep(K_MSEC(30));
			continue;
		}

		g_imu_ts12 = ts_next;
		g_report_seq++;
		g_auto_tick++;

		/* Only on a TRANSMITTED report. See HOME_HOLD_REPORTS. */
		if (g_home_hold > 0) {
			g_home_hold--;
			if (g_home_hold == 0) {
				printk("  HOME released\n");
			}
		}

		/* Advance the unlock sequence, also per transmitted report. */
		if (g_press_mode == PRESS_UNLOCK) {
			g_unlock_tick++;

			if (g_unlock_tick == UNLOCK_DELAY) {
				printk("  lock screen should be up: pressing now\n");
			}

			if (g_unlock_left > 0 && g_unlock_tick > UNLOCK_DELAY) {
				uint16_t t = g_unlock_tick - UNLOCK_DELAY;

				if (t % (UNLOCK_DOWN + UNLOCK_UP) == 0) {
					g_unlock_left--;
					printk("  unlock press %u of %u\n",
					       UNLOCK_PRESSES - g_unlock_left,
					       UNLOCK_PRESSES);
				}
			} else if (g_unlock_left == 0 &&
				   g_unlock_tick > UNLOCK_DELAY +
					   UNLOCK_PRESSES * (UNLOCK_DOWN + UNLOCK_UP) +
					   UNLOCK_SETTLE) {
				/*
				 * Settled. Now hand back. The puck must NOT
				 * sit here holding a link, or the console can
				 * never sleep again. In the product this is
				 * where BLE gives way to RF.
				 */
				g_press_mode = PRESS_NONE;
				printk("\n*** UNLOCK COMPLETE: releasing the "
				       "console ***\n");
				switch2_wake_complete();
			}
		}
		g_motion_frame = (g_motion_frame + 1) % MOTION_FRAMES;

		if (g_motion_frame == 0) {
			printk("stream: %u reports sent\n", (unsigned)MOTION_FRAMES);
		}

		k_sleep(K_MSEC(30));
	}
}

K_THREAD_DEFINE(report_tid, 1536, report_thread, NULL, NULL, NULL, 6, 0, 0);

/*
 * Force the report CCCD to "notify enabled" for this peer.
 *
 * On a reconnect the console does not always re-arm the report CCCD. It
 * registered the puck once and behaves as though the subscription persists,
 * while this side clears every CCCD on disconnect, since nothing is bonded as
 * far as the host knows.
 * Both ends then wait for the other: a stable 60+ s connection with zero ATT
 * traffic either way, and every notification failing with INVALID_STATE.
 */
static void force_report_ccc(struct bt_conn *conn)
{
	struct _bt_gatt_ccc *ccc = switch2_attrs[ATTR_D5A9_CCC].user_data;
	struct bt_conn_info info;

	if (bt_conn_get_info(conn, &info) || info.type != BT_CONN_TYPE_LE) {
		return;
	}

	ccc->cfg[0].id = info.id;
	bt_addr_le_copy(&ccc->cfg[0].peer, info.le.dst);
	ccc->cfg[0].value = BT_GATT_CCC_NOTIFY;
	ccc->value = BT_GATT_CCC_NOTIFY;

	printk("  report CCCD force-armed for the reconnect\n");
}

static void stream_start(struct bt_conn *conn)
{
	if (g_streaming) {
		return;
	}
	g_stream_conn = conn;
	g_report_seq = 0;
	g_motion_frame = 0;
	g_imu_ts12 = 0;
	g_auto_tick = 0;	/* same button phase on every connection */
	g_reconnect_stream = false;
	g_streaming = true;
	printk("  *** REPORT STREAM STARTED (notifying from 0x%04X) ***\n",
	       switch2_gatt_report_handle());

	/* If this connection came from a wake request, the console is now
	 * showing its lock screen and is waiting for button presses. Start them
	 * here rather than on connect. Before the stream is open, no press
	 * reaches the console at all.
	 */
	if (g_wake_pending) {
		g_wake_pending = false;
		switch2_gatt_begin_unlock();
	}
}

/*
 * Drop every scrap of per-link state before the radio changes hands.
 *
 * bt_disable() does not unregister the dynamic services, so the CCC config
 * entries survive a handover still holding the previous peer's address and
 * value. The report CCC is what decides whether the console is subscribed, and
 * believing it is when it is not gives the stable-but-silent link where both
 * ends wait for the other.
 */
void switch2_gatt_radio_release(void)
{
	struct _bt_gatt_ccc *ccc = switch2_attrs[ATTR_D5A9_CCC].user_data;

	g_streaming = false;
	g_stream_conn = NULL;
	g_reconnect_stream = false;
	g_wake_pending = false;
	g_press_mode = PRESS_NONE;
	g_unlock_left = 0;

	memset(ccc->cfg, 0, sizeof(ccc->cfg));
	ccc->value = 0;

	printk("  per-link GATT state cleared for the handover\n");
}

void switch2_gatt_disconnected(void)
{
	g_streaming = false;
	g_stream_conn = NULL;
	g_reconnect_stream = false;

	/*
	 * THE LINK KEY DELIBERATELY SURVIVES. Do not clear it here. Once the
	 * console has registered this puck it may reconnect and go straight to
	 * LL_ENC_REQ with the key it stored, and a peripheral that has thrown
	 * its copy away can never answer. Nothing is persisted to flash
	 * (CONFIG_BT_SETTINGS=n), so this only survives within a boot. Across
	 * a reflash the console's registration is stale and the address must be
	 * bumped instead. See puck_addr().
	 */
}

/*
 * Re-arm the derived key on every new connection.
 *
 * The host's key store is tied to a connection's key slot and no SMP ever runs
 * here, so the entry cannot be assumed to survive on its own. Re-installing it
 * here
 * is the same trick the Arduino firmware used (pre-load the key so the
 * stack's own lookup serves it) and it has to happen well before LL_ENC_REQ,
 * which a bonded reconnect sends within a few hundred milliseconds.
 */
void switch2_gatt_connected(struct bt_conn *conn)
{
	g_conn_at = k_uptime_get();
	g_stream_conn = conn;

	if (!g_have_link_key) {
		return;
	}

	/* This console has registered the puck before, so it may reconnect and
	 * simply wait for reports rather than re-running anything.
	 */
	g_reconnect_stream = true;

	printk("  re-installing stored link key for a bonded reconnect\n");
	install_link_key(conn, g_link_key_wire);
}

/* ---- Phase 6: post-encryption bring-up ---------------------------------- */

#define CMD_LEDS             0x09
#define SUB_LEDS_SET_PLAYER  0x07
#define CMD_VIBRATION        0x0A
#define CMD_FEATURE          0x0C
#define SUB_FEATURE_INIT     0x02
#define SUB_FEATURE_ENABLE   0x04

/* All of these are MEASURED replies, not invented ones. The project's recurring
 * mistake was answering with an empty ack where a real controller sends content
 * (which stalls the console at that exact step), and later over-correcting by
 * echoing parameters where a real controller sends nothing.
 */

/* FEATURE. Measured against a real Joy-Con that complied (LED lit, motor
 * blipped): it answers with exactly four zero bytes, not an echo of the
 * requested flags.
 */
static const uint8_t RSP_FEATURE[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t RSP_11_01[4] = { 0x01, 0x00, 0x00, 0x00 };
static const uint8_t RSP_13_01[4] = { 0x01, 0x00, 0x00, 0x00 };
static const uint8_t RSP_11_03[29] = {
	0x01, 0x20, 0x03, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x3b, 0x79,
	0x7d, 0x8b, 0x3a, 0x0a, 0xe8, 0x9c, 0x42, 0x58, 0xa0, 0x0b,
	0x42, 0x0a, 0xe8, 0x9c, 0x41, 0x58, 0xa0, 0x0b, 0x41
};

/* The exact payload a real Joy-Con notifies on 0x01/0x0C right before its
 * reports start. The console ASKS for this after arming the stream, twice
 * about 10 s apart, and getting nothing both times killed every otherwise
 * successful bring-up.
 */
static const uint8_t RSP_01_0C[4] = { 0x61, 0x12, 0x50, 0x10 };

/*
 * Answer a post-encryption command. Returns true if it was handled here.
 *
 * The console issues these once the link is encrypted. Every one of them must
 * get a reply: this file's own history records that an empty ack stalls
 * bring-up "the same way it stalled the pairing ceremony", and that silence on
 * 0x01/0x0C ended runs that had otherwise gone further than any before.
 */
static bool handle_phase6(struct bt_conn *conn, uint8_t cmd, uint8_t subcmd)
{
	const uint8_t *body = NULL;
	uint8_t body_len = 0;

	if (cmd == CMD_VIBRATION) {
		/* MEASURED: a real Joy-Con answers VIBRATION with an EMPTY
		 * body, verified on hardware that actually buzzed, so this is
		 * the reply of a controller that complied. Applies to every
		 * vibration subcommand, including the 0x0A/0x08 the console
		 * sends only to a registered controller, echoing back the first
		 * bytes of the page served from 0x00013060. Blocking on that one
		 * stalls with the controller already visible on screen.
		 */
	} else if (cmd == CMD_LEDS && subcmd == SUB_LEDS_SET_PLAYER) {
		/* Also measured empty, by asking a real Joy-Con directly. */
	} else if (cmd == CMD_FEATURE &&
		   (subcmd == SUB_FEATURE_INIT || subcmd == SUB_FEATURE_ENABLE)) {
		body = RSP_FEATURE;
		body_len = sizeof(RSP_FEATURE);
	} else if (cmd == 0x11 && subcmd == 0x01) {
		body = RSP_11_01;
		body_len = sizeof(RSP_11_01);
	} else if (cmd == 0x11 && subcmd == 0x03) {
		body = RSP_11_03;
		body_len = sizeof(RSP_11_03);
	} else if (cmd == 0x13 && subcmd == 0x01) {
		body = RSP_13_01;
		body_len = sizeof(RSP_13_01);
	} else if (cmd == 0x01 && subcmd == 0x0C) {
		body = RSP_01_0C;
		body_len = sizeof(RSP_01_0C);
	} else {
		return false;
	}

	send_response(conn, cmd, subcmd, body, body_len);

	/* 0x0C/0x04 is the LAST thing the console says in phase 6, so
	 * registration is complete once it is answered. That is the safe moment
	 * to start advertising "I belong to you". See
	 * switch2_registration_complete().
	 */
	if (cmd == CMD_FEATURE && subcmd == SUB_FEATURE_ENABLE) {
		switch2_registration_complete();
	}

	return true;
}

/*
 * The handshake channel (65a7, handle 0x0016).
 *
 * Frame shape: a 17-byte zero prefix, then [cmd][0x91][0x01][subcmd][...].
 * So cmd is at [17] and subcmd at [20], offsets read off live frames rather
 * than counted, because hand-deriving them produced two different answers.
 */
static ssize_t wr_65a7(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	const uint8_t *d = buf;
	uint8_t cmd, subcmd;

	hexdump("ATT WRITE", bt_gatt_attr_get_handle(attr), buf, len);

	if (len < 25) {
		printk("  handshake frame shorter than 25 bytes: not the known shape\n");
		return len;
	}

	cmd = d[17];
	subcmd = d[20];
	printk("  handshake cmd=0x%02X sub=0x%02X\n", cmd, subcmd);

	if (cmd == CMD_PAIR) {
		handle_pair(conn, subcmd, d, len);
	} else if (cmd == CMD_MEMORY && subcmd == SUB_MEMORY_READ) {
		handle_memory_read(conn, d + 25, len - 25);
	} else if (cmd == 0x07 && subcmd == 0x01) {
		send_response(conn, 0x07, 0x01, ANNOUNCED_CMD07, sizeof(ANNOUNCED_CMD07));
	} else if (cmd == 0x10 && subcmd == 0x01) {
		send_response(conn, 0x10, 0x01, ANNOUNCED_CMD10, sizeof(ANNOUNCED_CMD10));
	} else if (cmd == 0x16 && subcmd == 0x01) {
		send_response(conn, 0x16, 0x01, ANNOUNCED_CMD16, sizeof(ANNOUNCED_CMD16));
	} else if (!handle_phase6(conn, cmd, subcmd)) {
		printk("  no reply defined for this command yet\n");
	}

	return len;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	uint16_t handle = bt_gatt_attr_get_handle(attr);

	printk("CCCD handle=0x%04X -> 0x%04x  (%s)\n", handle, value,
	       value == BT_GATT_CCC_NOTIFY ? "NOTIFY ARMED" : "off");

	/*
	 * The console unsubscribes the report channel when it is finished,
	 * writing 0x0000 to 0x000F and the other three CCCDs just before
	 * dropping the link. Stop streaming when it does.
	 *
	 * Use this rather than an inactivity timeout. A healthy link can carry
	 * no ATT traffic for 85 s, so a silence-based watchdog would fire on a
	 * perfectly good connection.
	 */
	if (handle != switch2_gatt_report_handle() + 1) {
		return;
	}

	/*
	 * The console suspends and resumes the report stream by writing this
	 * CCCD, and it does that whenever it reorganises controller slots:
	 * backing out of Change Grip/Order, another Joy-Con arriving or leaving.
	 * It is re-shuffling, not dropping the controller.
	 *
	 * So follow it. Stop notifying when unsubscribed, start again when
	 * re-armed, and never tear the link down: dropping it here to force a
	 * re-advertise puts a visible reconnect blip on every slot change.
	 */
	if (value == BT_GATT_CCC_NOTIFY) {
		if (!g_streaming && g_stream_conn) {
			/* RESUME, not restart: the IMU clock must keep
			 * advancing. Rewinding it to zero here would emit a
			 * report whose timestamp jumped backwards while
			 * delta12 still claimed 48, the one field the console
			 * can validate against itself with no knowledge of the
			 * hardware, and the reason the synthesised clock exists.
			 */
			printk("  console armed the report channel: resuming "
			       "the stream (seq=%u ts12=%u)\n",
			       g_report_seq, g_imu_ts12);
			g_reconnect_stream = false;
			g_streaming = true;
		}
	} else if (g_streaming) {
		printk("  console unsubscribed the report channel: pausing "
		       "the stream (link stays up)\n");
		g_streaming = false;
		g_reconnect_stream = false;
	}
}

/* Wrap each value so one pair of handlers serves every attribute. */
#define VAL(_buf) (&(struct switch2_val){ .data = (_buf), .len = sizeof(_buf), \
					  .cap = sizeof(_buf) })

static struct switch2_val val_d5a9 = { .data = v_d5a9, .len = sizeof(v_d5a9), .cap = sizeof(v_d5a9) };
static struct switch2_val val_fd2 = { .data = v_fd2, .len = sizeof(v_fd2), .cap = sizeof(v_fd2) };
static struct switch2_val val_fde = { .data = v_fde, .len = sizeof(v_fde), .cap = sizeof(v_fde) };
static struct switch2_val val_d5a9_679d = { .data = d_d5a9_679d, .len = 0, .cap = sizeof(d_d5a9_679d) };
static struct switch2_val val_fd2_679d = { .data = d_fd2_679d, .len = 0, .cap = sizeof(d_fd2_679d) };
static struct switch2_val val_fde_679d = { .data = d_fde_679d, .len = 0, .cap = sizeof(d_fde_679d) };
static struct switch2_val val_cmdresp_b746 = { .data = d_cmdresp_b746, .len = 0, .cap = sizeof(d_cmdresp_b746) };
static struct switch2_val val_640c_b746 = { .data = d_640c_b746, .len = 0, .cap = sizeof(d_640c_b746) };
static struct switch2_val val_d3bd_b746 = { .data = d_d3bd_b746, .len = 0, .cap = sizeof(d_d3bd_b746) };
static struct switch2_val val_bd281 = { .data = v_bd281, .len = sizeof(v_bd281), .cap = sizeof(v_bd281) };
static struct switch2_val val_bd282 = { .data = v_bd282, .len = 0, .cap = sizeof(v_bd282) };
static struct switch2_val val_bd283 = { .data = v_bd283, .len = sizeof(v_bd283), .cap = sizeof(v_bd283) };

/* Write-only channels keep no value; they exist to be logged (and, later, to
 * drive the handshake). A one-byte sink is enough to satisfy the handler.
 */
static uint8_t sink_vib[64], sink_cmdw[128], sink_65a7[64], sink_4147[20], sink_fdf[20];
static struct switch2_val val_vib = { .data = sink_vib, .len = 0, .cap = sizeof(sink_vib) };
static struct switch2_val val_cmdw = { .data = sink_cmdw, .len = 0, .cap = sizeof(sink_cmdw) };
static struct switch2_val val_65a7 = { .data = sink_65a7, .len = 0, .cap = sizeof(sink_65a7) };
static struct switch2_val val_4147 = { .data = sink_4147, .len = 0, .cap = sizeof(sink_4147) };
static struct switch2_val val_fdf = { .data = sink_fdf, .len = 0, .cap = sizeof(sink_fdf) };

/* ---- The attribute table ------------------------------------------------ */

/*
 * Declaration order is the real Joy-Con 2 Right's, with two deliberate
 * departures carried over from the Arduino firmware: d5a9e01e is FIRST (so its
 * value lands on 0x000E) and ...fd2 is LAST.
 *
 * Every write permission is open. Encryption is not required on any attribute:
 * the console writes 0x000F and 0x0010 BEFORE the link is encrypted, and
 * demanding security here would reject those writes.
 */
static struct bt_gatt_attr switch2_attrs[] = {
	/* [0] */
	BT_GATT_PRIMARY_SERVICE(UUID_SWITCH2_SERVICE),

	/* 1. d5a9e01e, THE REPORT CHANNEL. value/CCCD/679d must be
	 * 0x000E/0x000F/0x0010.
	 */
	BT_GATT_CHARACTERISTIC(UUID_UNK_D5A9,
			       BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       rd, wr, &val_d5a9),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_679D, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_d5a9_679d),

	/* 2. Vibration, 3rd characteristic on the real device, here between
	 * d5a9 and COMMAND_WRITE, not at the end as originally guessed.
	 */
	BT_GATT_CHARACTERISTIC(UUID_VIBRATION_PRO,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr, &val_vib),

	/* 3. COMMAND_WRITE */
	BT_GATT_CHARACTERISTIC(UUID_COMMAND_WRITE,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr, &val_cmdw),

	/* 4. 65a7, the handshake channel. 42-byte LTK writes land here.
	 * [9] declaration, [10] value = handle 0x0016.
	 */
	BT_GATT_CHARACTERISTIC(UUID_UNK_65A7,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr_65a7, &val_65a7),

	/* 5. 4147 */
	BT_GATT_CHARACTERISTIC(UUID_UNK_4147,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr, &val_4147),

	/* 6. COMMAND_RESPONSE, where handshake replies and the registration
	 * announcement are notified from.
	 */
	BT_GATT_CHARACTERISTIC(UUID_COMMAND_RESPONSE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_B746, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_cmdresp_b746),

	/* 7. 640c */
	BT_GATT_CHARACTERISTIC(UUID_UNK_640C, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_B746, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_640c_b746),

	/* 8. d3bd */
	BT_GATT_CHARACTERISTIC(UUID_UNK_D3BD, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_B746, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_d3bd_b746),

	/* 9. fde */
	BT_GATT_CHARACTERISTIC(UUID_UNK_FDE,
			       BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       rd, wr, &val_fde),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_679D, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_fde_679d),

	/* 10. fdf */
	BT_GATT_CHARACTERISTIC(UUID_UNK_FDF,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr, &val_fdf),

	/* 11. INPUT_REPORT (...fd2), last. See the UUID comment above. */
	BT_GATT_CHARACTERISTIC(UUID_INPUT_REPORT,
			       BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       rd, wr, &val_fd2),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(UUID_DESC_679D, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   rd, wr, &val_fd2_679d),
};

/* The bd28x service. It must land AFTER the main one so the main one keeps the
 * low handles; bd281 is read very early, before the CCCD exchange starts.
 */
static struct bt_gatt_attr switch2_attrs1[] = {
	BT_GATT_PRIMARY_SERVICE(UUID_SERVICE1),

	BT_GATT_CHARACTERISTIC(UUID_S1_CHAR_R1, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       rd, wr, &val_bd281),
	BT_GATT_CHARACTERISTIC(UUID_S1_CHAR_W,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE, NULL, wr, &val_bd282),
	BT_GATT_CHARACTERISTIC(UUID_S1_CHAR_R2, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       rd, wr, &val_bd283),
};

/*
 * Registered at runtime, in this order, DELIBERATELY.
 *
 * BT_GATT_SERVICE_DEFINE places services in a linker iterable section, so the
 * order that comes out is the linker's, not the source file's. Declared
 * statically with main first, this build's linker emitted bd28x first and put
 * the report channel on 0x0015, seven handles adrift. The console writes
 * 0x000F and 0x0010 by hardcoded handle, so handle position cannot be left to
 * link order.
 *
 * bt_gatt_service_register() assigns handles strictly in call order. Zephyr's
 * own GAP and GATT services occupy 0x0001-0x000B, so registering the main
 * service first puts its declaration on 0x000C and lands d5a9 on
 * 0x000E/0x000F/0x0010 with no spacer service at all.
 */
static struct bt_gatt_service switch2_svc = BT_GATT_SERVICE(switch2_attrs);
static struct bt_gatt_service switch2_svc1 = BT_GATT_SERVICE(switch2_attrs1);

/*
 * Restore a link key saved before the last power cut.
 *
 * With this in place a cold boot is indistinguishable, to the console, from a
 * warm reconnect: it sends LL_ENC_REQ, the host answers from bt_keys, and the
 * link encrypts without any PAIR ceremony or user interaction. Without it the
 * controller answers LL_REJECT_EXT_IND "PIN or Key Missing" and the console
 * hangs up 0.4 s after finding the puck.
 */
static void restore_link_key(void)
{
	uint8_t wire[16];

	if (!switch2_store_load_key(wire)) {
		return;
	}

	memcpy(g_link_key_wire, wire, 16);
	for (uint8_t i = 0; i < 16; i++) {
		g_link_key_msb[i] = wire[15 - i];
	}
	g_have_link_key = true;

	printk("restored link key from flash\n"
	       "  (it is installed per-connection by switch2_gatt_connected)\n");
}

int switch2_gatt_init(void)
{
	int err = bt_gatt_service_register(&switch2_svc);

	/*
	 * -EINVAL here means "already registered", which is the normal case on
	 * the second call: bt_disable() does NOT unregister dynamic services,
	 * so after an RF -> BLE handover the table is still in place. Verified
	 * after a disable/enable cycle (the report channel was still on
	 * 0x000E/0x000F/0x0010 and bd281 still on 0x0031) so this is not a
	 * failure to report, and treating it as one made a working handover
	 * look broken.
	 */
	if (err == -EINVAL) {
		printk("GATT already registered (surviving a radio handover)\n");
		restore_link_key();
		return 0;
	}
	if (err) {
		printk("main service register failed (%d)\n", err);
		return err;
	}

	err = bt_gatt_service_register(&switch2_svc1);
	if (err) {
		printk("bd28x service register failed (%d)\n", err);
		return err;
	}

	restore_link_key();
	return 0;
}

/* ---- Handle reporting --------------------------------------------------- */

uint16_t switch2_gatt_trigger_handle(void)
{
	return bt_gatt_attr_get_handle(&switch2_attrs[ATTR_D5A9_679D]);
}

uint16_t switch2_gatt_report_handle(void)
{
	return bt_gatt_attr_get_handle(&switch2_attrs[ATTR_D5A9_VALUE]);
}

static uint8_t dump_one(const struct bt_gatt_attr *attr, uint16_t handle,
			void *user_data)
{
	char uuid[BT_UUID_STR_LEN];

	ARG_UNUSED(user_data);
	bt_uuid_to_str(attr->uuid, uuid, sizeof(uuid));
	printk("  0x%04X  %s\n", handle, uuid);
	return BT_GATT_ITER_CONTINUE;
}

void switch2_gatt_dump(void)
{
	printk("\n=== ATTRIBUTE TABLE ===\n");
	bt_gatt_foreach_attr(0x0001, 0xFFFF, dump_one, NULL);
}

void switch2_gatt_check(void)
{
	uint16_t v = switch2_gatt_report_handle();
	uint16_t c = bt_gatt_attr_get_handle(&switch2_attrs[ATTR_D5A9_CCC]);
	uint16_t d = switch2_gatt_trigger_handle();
	uint16_t r = bt_gatt_attr_get_handle(&switch2_attrs[ATTR_CMDRESP_VALUE]);

	/* bd281 advertises bd282's handles to the console; fill them in from the
	 * table this build registered rather than a real device's numbers.
	 * attrs[] here: [0] service, [1] bd281 decl, [2] bd281 value,
	 * [3] bd282 decl, [4] bd282 value.
	 */
	uint16_t w_decl = bt_gatt_attr_get_handle(&switch2_svc1.attrs[3]);
	uint16_t w_val = bt_gatt_attr_get_handle(&switch2_svc1.attrs[4]);

	v_bd281[0] = w_decl & 0xFF;
	v_bd281[1] = w_decl >> 8;
	v_bd281[2] = w_val & 0xFF;
	v_bd281[3] = w_val >> 8;

	printk("\nREPORT CHANNEL d5a9: value=0x%04X cccd=0x%04X 679d-desc=0x%04X\n", v, c, d);
	printk("COMMAND_RESPONSE value=0x%04X (replies notify from here; old stack: 0x001A)\n", r);
	printk("          real device:  0x000E        0x000F        0x0010\n");
	if (v == 0x000E && c == 0x000F && d == 0x0010) {
		printk("  ALIGNED: matches a real Joy-Con 2\n");
	} else {
		printk("  MISALIGNED by %+d: add or remove a spacer service\n",
		       (int)v - 0x000E);
	}
	printk("bd281 -> bd282 decl=0x%04X val=0x%04X\n\n", w_decl, w_val);
}
