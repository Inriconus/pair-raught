/*
 * The puck AS a Nintendo Switch Pro Controller over USB.
 *
 * Ported from the Arduino build's mode_switch_pro.cpp, which is the proven
 * implementation. The console demands considerably more than an input report,
 * and every constant here was measured against real hardware there.
 *
 * THE CONSOLE WILL NOT LOOK AT INPUT UNTIL IT HAS RUN ITS HANDSHAKE. Report
 * 0x80 selects device type and baud rate, report 0x01 carries subcommands, and
 * nothing streams until subcommand 0x03 sets report mode to 0x30. It also reads
 * back SPI calibration blocks a real pad keeps in flash. Answer all of it, or
 * the pad simply never appears.
 *
 * ONE Pro Controller, not four. The Arduino build mounts a pool so several
 * bonded Steam Controllers appear as several pads; that needs dynamic USB
 * re-enumeration and is separate work. This presents bond slot 0.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/drivers/usb/usb_buf.h>
#include <string.h>
#include <errno.h>
#include <hal/nrf_ficr.h>

#include "switch_pro_usb.h"
#include "switch_pro.h"
#include "puck_input.h"
#include "switch2_store.h"
#include "puck_rumble.h"
#include "puck_settings.h"
#include "rf_link.h"
#include "puck_trace.h"

/*
 * The controller's input, or false when there is none to send: never
 * connected, or silent long enough that rf_link no longer counts it live.
 *
 * puck_input keeps the last report it was given, so reading it without this
 * check leaves a controller that powered off or left range holding its last
 * buttons at the console until it comes back.
 */
static bool live_input(struct puck_input *in)
{
	return rf_link_slot_live(0) && puck_input_get(0, in);
}

/*
 * Report descriptor, copied byte-for-byte from the Arduino build. Declares the
 * 0x30 input report, the 0x21 subcommand reply, the 0x81 handshake reply, and
 * the 0x01/0x10/0x80/0x82 output reports the console writes.
 */
static const uint8_t SWPRO_HID_DESC[] = {
	0x05, 0x01, 0x15, 0x00, 0x09, 0x04, 0xA1, 0x01, 0x85, 0x30, 0x05, 0x01,
	0x05, 0x09, 0x19, 0x01, 0x29, 0x0A, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
	0x95, 0x0A, 0x55, 0x00, 0x65, 0x00, 0x81, 0x02, 0x05, 0x09, 0x19, 0x0B,
	0x29, 0x0E, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81, 0x02,
	0x75, 0x01, 0x95, 0x02, 0x81, 0x03, 0x0B, 0x01, 0x00, 0x01, 0x00, 0xA1,
	0x00, 0x0B, 0x30, 0x00, 0x01, 0x00, 0x0B, 0x31, 0x00, 0x01, 0x00, 0x0B,
	0x32, 0x00, 0x01, 0x00, 0x0B, 0x35, 0x00, 0x01, 0x00, 0x15, 0x00, 0x27,
	0xFF, 0xFF, 0x00, 0x00, 0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0xC0, 0x0B,
	0x39, 0x00, 0x01, 0x00, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3B,
	0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x05, 0x09, 0x19,
	0x0F, 0x29, 0x12, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x04, 0x81,
	0x02, 0x75, 0x08, 0x95, 0x34, 0x81, 0x03, 0x06, 0x00, 0xFF, 0x85, 0x21,
	0x09, 0x01, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03, 0x85, 0x81, 0x09, 0x02,
	0x75, 0x08, 0x95, 0x3F, 0x81, 0x03, 0x85, 0x01, 0x09, 0x03, 0x75, 0x08,
	0x95, 0x3F, 0x91, 0x83, 0x85, 0x10, 0x09, 0x04, 0x75, 0x08, 0x95, 0x3F,
	0x91, 0x83, 0x85, 0x80, 0x09, 0x05, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
	0x85, 0x82, 0x09, 0x06, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83, 0xC0
};

/*
 * Canonical factory SPI blocks the console reads for calibration. Neutral IMU
 * and centred sticks, so a fresh "pad" calibrates sane. The user-cal region
 * (0x80xx) reads 0xFF when blank, which is how the console knows to fall back
 * to these factory values.
 */
static const uint8_t SPI_IMU_CAL[24] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34
};
static const uint8_t SPI_PARAMS1[24] = {
	0x50, 0xFD, 0x00, 0x00, 0xC6, 0x0F, 0x0F, 0x30, 0x61, 0x96, 0x30, 0xF3,
	0xD4, 0x14, 0x54, 0x41, 0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63
};
static const uint8_t SPI_PARAMS2[18] = {
	0x0F, 0x30, 0x61, 0x96, 0x30, 0xF3, 0xD4, 0x14, 0x54,
	0x41, 0x15, 0x54, 0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63
};

/*
 * Manual-pairing (subcommand 0x01) reply bodies. A real Switch runs this
 * 3-stage key exchange over USB so it can register the pad. It validates the
 * SHAPE, not the key contents.
 */
static const uint8_t BT_PAIR_2[31] = {
	0x02, 0xE5, 0xC8, 0xE4, 0x92, 0x05, 0xFF, 0xC9, 0x8A, 0x7D, 0xEA,
	0x15, 0xF6, 0x19, 0xBA, 0x82, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t BT_PAIR_3[31] = { 0x03 };

/* "Pro Controller" */
static const uint8_t PRO_NAME[14] = { 0x50, 0x72, 0x6F, 0x20, 0x43, 0x6F, 0x6E,
				      0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65, 0x72 };

#define JC_REPLEN 63

/* The controller accelerometer is +/-2g (16384/g) and the factory calibration
 * presented here declares a +/-8g Pro Controller (4096/g), so divide by four.
 */
#define SW_ACCEL_DIV 4
#define JCQ_N     8

/*
 * A FLOOR, not the rate. Reports actually go out every 10 ms, about 100 Hz.
 *
 * switch_pro_usb_task() is called only from the RF thread, which sleeps 10 ms a
 * pass, so this gate has always expired by the time it is tested and any value
 * of 10 or below gives the same 100 Hz. Changing the real rate means changing
 * that sleep, and faster cadences there wedged the CDC console hard enough that
 * the host could not open the port.
 *
 * 8 stays as the intent, since that is what a real Pro Controller polls at. The
 * console integrates the three IMU samples per report by count, so the cadence
 * is protocol rather than taste. 100 Hz measures fine on hardware.
 */
#define STREAM_MS 8

/* Factory stick calibration, packed 12-bit, built at boot. */
static uint8_t g_stick_cal[18];

/*
 * The user-cal mirror the console writes during "Calibrate Motion Controls"
 * and reads back afterwards. That is how a resting IMU offset gets cancelled.
 * Blank is 0xFF so the factory blocks win.
 */
/*
 * Mirror of the console's user calibration page. Blank flash reads 0xFF, so
 * the unwritten state has to BE 0xFF rather than the zeroes BSS would give it.
 * Filled by switch_pro_usb_load_cal(), which must run in EVERY USB mode. Doing
 * it from switch_pro_usb_register() only covers Pro mode, and a dongle-mode
 * puck then reports all 256 bytes written when none were.
 */
static uint8_t g_user_cal[0x100];
static bool g_user_cal_dirty;
static uint32_t g_user_cal_dirty_ms;

/* The pretend Bluetooth address, derived from the chip so two pucks differ. */
static uint8_t g_mac[6];

static const struct device *pro_dev;
static uint8_t g_report_mode;	/* 0 until subcommand 0x03 selects 0x30 */
static uint8_t g_timer;
static uint32_t g_last_stream_ms;
static uint32_t g_rx_reports;
static uint32_t g_tx_reports;

/*
 * Reply FIFO. The console's handshake reports arrive in a USB callback; the
 * canonical answer is enqueued there and sent from the task, one per pass, so a
 * bursty init cannot starve the 0x30 stream.
 *
 * Every reply goes out padded to the full 63 bytes: a reply shorter than the
 * descriptor declares is silently dropped by some hosts, which in the Arduino
 * build meant short 0x81/0x21 answers never arrived at all.
 */
struct jc_rep {
	uint8_t rid;
	uint8_t data[JC_REPLEN];
};

static struct jc_rep g_q[JCQ_N];
static uint8_t g_qh, g_qt;
static struct k_spinlock g_q_lock;

/*
 * One in-flight buffer, UDC-aligned: the HID class wraps what it is given by
 * reference rather than copying it. See the long note in puck_hid.c, and note
 * input_report_done is registered below, without which submit blocks the caller
 * forever on a host that is not reading.
 */
UDC_STATIC_BUF_DEFINE(g_pro_tx, UDC_ROUND_UP(64));
static volatile bool g_tx_busy;

static void pack12(uint8_t *o9, const uint16_t v[6])
{
	o9[0] = v[0] & 0xFF;
	o9[1] = ((v[1] & 0x0F) << 4) | ((v[0] >> 8) & 0x0F);
	o9[2] = (v[1] >> 4) & 0xFF;
	o9[3] = v[2] & 0xFF;
	o9[4] = ((v[3] & 0x0F) << 4) | ((v[2] >> 8) & 0x0F);
	o9[5] = (v[3] >> 4) & 0xFF;
	o9[6] = v[4] & 0xFF;
	o9[7] = ((v[5] & 0x0F) << 4) | ((v[4] >> 8) & 0x0F);
	o9[8] = (v[5] >> 4) & 0xFF;
}

static void build_stick_cal(void)
{
	const uint16_t C = 2048, R = 1800;	/* centre, +/- range per axis */
	uint16_t L[6] = { R, R, C, C, R, R };	/* max(x,y) centre(x,y) min(x,y) */
	uint16_t Rr[6] = { C, C, R, R, R, R };	/* centre(x,y) min(x,y) max(x,y) */

	pack12(&g_stick_cal[0], L);
	pack12(&g_stick_cal[9], Rr);
}

static void build_mac(void)
{
	uint32_t a = NRF_FICR->DEVICEID[0];
	uint32_t b = NRF_FICR->DEVICEID[1];

	/* Locally administered and unicast, deliberately not a Nintendo OUI.
	 * The console only needs a stable address, not a real one.
	 */
	g_mac[0] = 0x02;
	g_mac[1] = (uint8_t)(a >> 24);
	g_mac[2] = (uint8_t)(a >> 16);
	g_mac[3] = (uint8_t)(a >> 8);
	g_mac[4] = (uint8_t)(b >> 8);
	g_mac[5] = (uint8_t)b;
}

static void spi_read(uint32_t addr, uint8_t len, uint8_t *dst)
{
	for (uint8_t i = 0; i < len; i++) {
		uint32_t a = addr + i;
		uint8_t v = 0xFF;

		if (a >= 0x6020 && a < 0x6020 + 24) {
			v = SPI_IMU_CAL[a - 0x6020];
		} else if (a >= 0x603D && a < 0x603D + 18) {
			v = g_stick_cal[a - 0x603D];
		} else if (a >= 0x6050 && a < 0x6050 + 13) {
			/*
			 * Colours come from settings, so the pad can be any
			 * colour the console shows. Four RGB triples (body,
			 * buttons, left grip, right grip) then a terminator.
			 */
			unsigned k = a - 0x6050;

			if (k < 3) {
				v = g_cfg.col_body[k];
			} else if (k < 6) {
				v = g_cfg.col_buttons[k - 3];
			} else if (k < 9) {
				v = g_cfg.col_grip_l[k - 6];
			} else if (k < 12) {
				v = g_cfg.col_grip_r[k - 9];
			} else {
				v = 0xFF;
			}
		} else if (a >= 0x6080 && a < 0x6080 + 24) {
			v = SPI_PARAMS1[a - 0x6080];
		} else if (a >= 0x6098 && a < 0x6098 + 18) {
			v = SPI_PARAMS2[a - 0x6098];
		} else if (a >= 0x8000 && a < 0x8100) {
			v = g_user_cal[a - 0x8000];
		}
		dst[i] = v;
	}
}

static void spi_write(uint32_t addr, uint8_t len, const uint8_t *data,
		      uint16_t avail)
{
	if (len > avail) {
		len = (uint8_t)avail;
	}
	for (uint8_t i = 0; i < len; i++) {
		uint32_t a = addr + i;

		if (a >= 0x8000 && a < 0x8100 &&
		    g_user_cal[a - 0x8000] != data[i]) {
			g_user_cal[a - 0x8000] = data[i];
			g_user_cal_dirty = true;
			g_user_cal_dirty_ms = k_uptime_get_32();
		}
	}
}

static void jc_enq(uint8_t rid, const uint8_t *d, uint8_t len)
{
	k_spinlock_key_t key = k_spin_lock(&g_q_lock);
	uint8_t nt = (uint8_t)((g_qt + 1) % JCQ_N);

	if (nt == g_qh) {
		/* Full -> drop. The console re-requests on timeout, which is a
		 * better failure than answering out of order.
		 */
		k_spin_unlock(&g_q_lock, key);
		return;
	}
	if (len > JC_REPLEN) {
		len = JC_REPLEN;
	}
	g_q[g_qt].rid = rid;
	memset(g_q[g_qt].data, 0, JC_REPLEN);
	memcpy(g_q[g_qt].data, d, len);
	g_qt = nt;
	k_spin_unlock(&g_q_lock, key);
}

/*
 * The standard input prefix [0..11]: timer, battery/connection, three button
 * bytes, both packed sticks, vibrator echo. Shared by the streamed 0x30 report
 * and by every 0x21 subcommand reply.
 *
 * Buttons and axes come from switch_pro.c rather than being re-derived here, so
 * there is ONE mapping with one place to fix. (The Arduino equivalent inlines
 * its own copy, including the A/B swap and back-paddle remap codes this port
 * deliberately leaves to the config screen.)
 */
static void input_prefix(uint8_t *out)
{
	struct puck_input in;
	uint32_t jc = 0;
	uint16_t lx = 2048, ly = 2048, rx = 2048, ry = 2048;

	if (live_input(&in)) {
		jc = switch_pro_buttons(&in);
		lx = switch_pro_axis(in.lx);
		ly = switch_pro_axis(in.ly);
		rx = switch_pro_axis(in.rx);
		ry = switch_pro_axis(in.ry);
	}

	out[0] = g_timer++;

	/*
	 * bat_con: [7:5] capacity, bit4 charging, bit0 host-powered. Only EVEN
	 * high nibbles are legal. Packing an odd value sets the charging bit
	 * and the console shows the pad as charging. Reported as a full,
	 * discharging, battery-powered pad: claiming host-powered makes the
	 * console treat the pad as the wired primary and show a charging bolt.
	 */
	out[1] = (uint8_t)((4u << 1) << 4);

	out[2] = (uint8_t)(jc);
	out[3] = (uint8_t)(jc >> 8);
	out[4] = (uint8_t)(jc >> 16);

	/* Two 12-bit axes packed into three bytes, per stick. */
	out[5] = (uint8_t)(lx & 0xFF);
	out[6] = (uint8_t)(((ly & 0x0F) << 4) | ((lx >> 8) & 0x0F));
	out[7] = (uint8_t)((ly >> 4) & 0xFF);
	out[8] = (uint8_t)(rx & 0xFF);
	out[9] = (uint8_t)(((ry & 0x0F) << 4) | ((rx >> 8) & 0x0F));
	out[10] = (uint8_t)((ry >> 4) & 0xFF);

	/* rumble_input_report echo: a genuine pad emits 0x09..0x0C here, and
	 * some console firmware expects it nonzero.
	 */
	out[11] = 0x09;
}

/*
 * The streamed 0x30 report: the prefix plus three IMU samples.
 *
 * AXIS MAPPING IS A ROTATION, AND BOTH SENSORS MUST SHARE IT. Gyro slots follow
 * hid-nintendo: +6 roll, +8 pitch, +10 yaw, sourced as roll<-+gy, pitch<--gx,
 * yaw<-+gz. The accelerometer takes the same signed permutation, X<-+ay,
 * Y<--ax, Z<-+az, because the console fuses accel with gyro to anchor absolute
 * orientation. Opposite handedness between the two frames makes the gravity
 * correction push the wrong way, which shows up as intermittent ~45-degree axis
 * mixing and a roll bias that only a replug clears.
 *
 * SCALE: the controller's accelerometer is +/-2g (~16384 counts/g) and the
 * factory calibration presented here declares a +/-8g Pro Controller (4096/g),
 * so accel is divided by four. Without that the console reads gravity as ~4g
 * and rejects the accel for drift correction, since it has to look like gravity
 * rather than linear acceleration, and gyro roll error drifts into a slow lean.
 * Gyro goes out at native scale.
 *
 * The sensitivity trim matches a genuine Pro Controller and its own Steam mode:
 * roll at 80%, pitch and yaw at 90%. In int32 so the multiply cannot overflow.
 *
 * All three samples in a report carry the same values. A real pad sends three
 * genuinely spaced ones; there is only one per RF poll here, and the console
 * integrates by sample count at a fixed rate, so repeating keeps its clock
 * right.
 */
static void build_0x30(uint8_t out[JC_REPLEN])
{
	struct puck_input in;
	int16_t aX = 0, aY = 0, aZ = 0;
	int16_t groll = 0, gpitch = 0, gyaw = 0;

	memset(out, 0, JC_REPLEN);
	input_prefix(out);

	if (live_input(&in)) {
		aX = (int16_t)(in.ay / SW_ACCEL_DIV);
		aY = (int16_t)((-(int16_t)in.ax) / SW_ACCEL_DIV);
		aZ = (int16_t)(in.az / SW_ACCEL_DIV);

		groll = (int16_t)(((int32_t)in.gy * 4) / 5);
		gpitch = (int16_t)(((int32_t)(-(int16_t)in.gx) * 9) / 10);
		gyaw = (int16_t)(((int32_t)in.gz * 9) / 10);
	}

	for (int k = 0; k < 3; k++) {
		int o = 12 + k * 12;

		out[o + 0] = (uint8_t)(aX & 0xFF);
		out[o + 1] = (uint8_t)((aX >> 8) & 0xFF);
		out[o + 2] = (uint8_t)(aY & 0xFF);
		out[o + 3] = (uint8_t)((aY >> 8) & 0xFF);
		out[o + 4] = (uint8_t)(aZ & 0xFF);
		out[o + 5] = (uint8_t)((aZ >> 8) & 0xFF);
		out[o + 6] = (uint8_t)(groll & 0xFF);
		out[o + 7] = (uint8_t)((groll >> 8) & 0xFF);
		out[o + 8] = (uint8_t)(gpitch & 0xFF);
		out[o + 9] = (uint8_t)((gpitch >> 8) & 0xFF);
		out[o + 10] = (uint8_t)(gyaw & 0xFF);
		out[o + 11] = (uint8_t)((gyaw >> 8) & 0xFF);
	}
}

static void jc_subcmd(uint8_t sub, const uint8_t *args, uint16_t alen)
{
	uint8_t p[JC_REPLEN];

	memset(p, 0, sizeof p);
	input_prefix(p);
	p[13] = sub;

	switch (sub) {
	case 0x01: {	/* manual BT pairing, three stages */
		uint8_t t = (alen >= 1) ? args[0] : 3;

		p[12] = 0x81;
		if (t == 1) {
			p[14] = 0x01;
			memcpy(&p[15], g_mac, 6);
			p[22] = 0x25;
			p[23] = 0x08;
			memcpy(&p[24], PRO_NAME, sizeof PRO_NAME);
			p[43] = 0x68;
		} else if (t == 2) {
			memcpy(&p[14], BT_PAIR_2, sizeof BT_PAIR_2);
		} else {
			memcpy(&p[14], BT_PAIR_3, sizeof BT_PAIR_3);
		}
		break;
	}
	case 0x02:	/* device info */
		p[12] = 0x82;
		p[14] = 0x03;
		p[15] = 0x48;	/* firmware 3.72, a genuine Pro value */
		p[16] = 0x03;	/* controller type: Pro Controller */
		p[17] = 0x02;
		memcpy(&p[18], g_mac, 6);
		p[24] = 0x01;	/* colours live in SPI at 0x6050 */
		p[25] = 0x01;
		break;
	case 0x03:	/* set input report mode, 0x30 starts the stream */
		if (alen >= 1 && args[0] == 0x30) {
			if (g_report_mode != 0x30) {
				puck_trace_stage(TR_PRO_STREAM);
				printk("pro: console selected report mode 0x30"
				       ": streaming input\n");
			}
			g_report_mode = 0x30;
		}
		p[12] = 0x80;
		break;
	case 0x04:	/* trigger elapsed time, canned, as a real pad replies */
		p[12] = 0x83;
		p[15] = 0xCC;
		p[17] = 0xEE;
		p[19] = 0xFF;
		break;
	case 0x10: {	/* SPI read -> echo [addr][len], then the data */
		uint32_t a;
		uint8_t rl;

		if (alen < 5) {
			p[12] = 0x80;
			break;
		}
		a = (uint32_t)args[0] | ((uint32_t)args[1] << 8) |
		    ((uint32_t)args[2] << 16) | ((uint32_t)args[3] << 24);
		rl = args[4];
		if (rl > 0x1D) {
			rl = 0x1D;
		}
		p[12] = 0x90;
		p[14] = args[0];
		p[15] = args[1];
		p[16] = args[2];
		p[17] = args[3];
		p[18] = rl;
		spi_read(a, rl, &p[19]);
		break;
	}
	case 0x11:	/* SPI write -> persist the console's motion calibration */
		if (alen >= 5) {
			uint32_t a = (uint32_t)args[0] |
				     ((uint32_t)args[1] << 8) |
				     ((uint32_t)args[2] << 16) |
				     ((uint32_t)args[3] << 24);

			spi_write(a, args[4], &args[5],
				  (alen > 5) ? (uint16_t)(alen - 5) : 0);
		}
		p[12] = 0x80;
		break;
	case 0x21:	/* set NFC/IR config */
		p[12] = 0xA0;
		break;
	default:
		/* Generic positive ACK for 0x06/0x08/0x30/0x38/0x40/0x41/0x48
		 * and friends. Answering EVERYTHING matters: an unanswered
		 * subcommand stalls the console's init rather than being
		 * skipped over.
		 */
		p[12] = 0x80;
		break;
	}

	jc_enq(0x21, p, JC_REPLEN);
}

/*
 * The console's rumble stream: [timer][left x4][right x4], carried both in
 * report 0x10 (rumble alone) and ahead of every 0x01 subcommand.
 *
 * DECODE EVERY FRAME, RELAY ONLY ON CHANGE. The decode is stateful. The
 * packed commands step a running amplitude, so skipping frames corrupts the
 * state that later frames depend on. But the console sends rumble every frame,
 * and re-sending an unchanged level would flood the RF relay, which shares the
 * poll with the controller's own input.
 */
static uint16_t g_last_lo, g_last_hi;

/* Where the rumble chain stops, when the console is the only thing that can
 * exercise it and there is no console port to watch. Shown by key .s..
 */
/* Persisted so the CONSOLE's behaviour can be read back on a PC afterwards.
 * The puck power-cycles moving between them, which lost this twice.
 */
static uint8_t g_rumble_flags;
static uint8_t g_rumble_flags_prev;

#define RF_SEEN     0x01
#define RF_AUDIBLE  0x02
#define RF_RELAYED  0x04

static void rumble_flag(uint8_t bit)
{
	if (g_rumble_flags & bit) {
		return;			/* already recorded; no flash write */
	}
	g_rumble_flags |= bit;
	switch2_store_save_rumble(g_rumble_flags);
}

static uint32_t g_rumble_frames;	/* rumble bytes arrived at all */
static uint32_t g_rumble_nonzero;	/* decoded to something audible */
static uint32_t g_rumble_relayed;	/* handed to the RF relay */

static void pro_rumble(const uint8_t *p, uint16_t pn)
{
	uint16_t lo, hi;

	if (pn < 9) {
		return;
	}

	g_rumble_frames++;
	rumble_flag(RF_SEEN);

	lo = puck_rumble_decode(0, 0, p + 1);
	hi = puck_rumble_decode(0, 1, p + 5);
	if (lo || hi) {
		g_rumble_nonzero++;
		rumble_flag(RF_AUDIBLE);
	}

	if (lo == g_last_lo && hi == g_last_hi) {
		return;
	}
	g_last_lo = lo;
	g_last_hi = hi;

	g_rumble_relayed++;
	rumble_flag(RF_RELAYED);
	rf_link_rumble(0, lo, hi);
}


/*
 * Everything the console writes, from either pipe. Report 0x80 is the USB
 * handshake, 0x01 carries a subcommand behind eight rumble bytes, 0x10 is
 * rumble alone.
 *
 * `buf` always starts with the report id here: Zephyr hands the interrupt-OUT
 * payload over whole, and the control-pipe path below re-attaches the id it was
 * given so both arrive in the same shape.
 */
static void pro_out(const uint8_t *buf, uint16_t n)
{
	uint8_t id;
	const uint8_t *p;
	uint16_t pn;

	if (n < 1) {
		return;
	}
	g_rx_reports++;

	id = buf[0];
	p = buf + 1;
	pn = (uint16_t)(n - 1);

	if (id == 0x80) {
		if (pn < 1) {
			return;
		}
		if (p[0] == 0x01) {	/* device type + MAC */
			uint8_t d[9] = { 0x01,	   0x00,     0x03,
					 g_mac[0], g_mac[1], g_mac[2],
					 g_mac[3], g_mac[4], g_mac[5] };

			puck_trace_stage(TR_PRO_HANDSHAKE);
			printk("pro: handshake 0x80/01 (device type + MAC)\n");
			jc_enq(0x81, d, sizeof d);
		} else if (p[0] == 0x02) {
			uint8_t d[1] = { 0x02 };

			printk("pro: handshake 0x80/02 (baud/handshake)\n");
			jc_enq(0x81, d, sizeof d);
		} else if (p[0] == 0x03) {
			uint8_t d[1] = { 0x03 };

			jc_enq(0x81, d, sizeof d);
		}
		/* 0x04 force-USB, 0x05 enable-timeout, 0x06 reset: no reply. */
		return;
	}

	if (id == 0x01) {	/* [timer][rumble x8][subcmd][args...] */
		if (pn < 10) {
			return;
		}
		pro_rumble(p, pn);
		jc_subcmd(p[9], p + 10, (pn > 10) ? (uint16_t)(pn - 10) : 0);
		return;
	}
	/* 0x10 is rumble alone, with the same 9-byte prefix. */
	if (id == 0x10) {
		pro_rumble(p, pn);
	}
}

/*
 * INCOMING REPORTS ARE COPIED HERE AND PROCESSED LATER, NEVER IN THE CALLBACK.
 *
 * Zephyr delivers output reports on the USB stack's own thread, which preempts
 * the RF thread, and rf_link_task() is a microsecond-precise busy-wait. Real
 * work in the callback, building a 63-byte reply or reading the SPI mirrors,
 * jitters the RF RX window: replies are missed and the controller drops the
 * session, LED solid for half a second and then back to searching.
 *
 * A SOF callback, a dedicated task and report_complete were all tried in the
 * Arduino build and all three fight the RF poll the same way, which is why its
 * whole architecture is one cooperative loop.
 *
 * So the callback does one bounded memcpy and nothing else, and every reply is
 * built and sent from the RF loop, in step with the poll.
 */
#define OUTQ_N 8

struct out_ev {
	uint16_t len;
	uint8_t data[64];
};

static struct out_ev g_outq[OUTQ_N];
static uint8_t g_outq_head, g_outq_tail;
static uint32_t g_outq_dropped;
static struct k_spinlock g_outq_lock;

/* Called from the USB thread. Copy and leave, nothing else. */
static void out_enqueue(const uint8_t *buf, uint16_t len)
{
	k_spinlock_key_t key;
	uint8_t nh;

	if (len == 0) {
		return;
	}
	if (len > sizeof g_outq[0].data) {
		len = sizeof g_outq[0].data;
	}

	key = k_spin_lock(&g_outq_lock);
	nh = (uint8_t)((g_outq_head + 1) % OUTQ_N);
	if (nh == g_outq_tail) {
		/* Full: drop the NEWEST here, unlike the outgoing ring. A
		 * handshake is a conversation. Dropping the oldest would
		 * answer the console's questions out of order, and it re-sends
		 * on timeout anyway.
		 */
		g_outq_dropped++;
		k_spin_unlock(&g_outq_lock, key);
		return;
	}
	memcpy(g_outq[g_outq_head].data, buf, len);
	g_outq[g_outq_head].len = len;
	g_outq_head = nh;
	k_spin_unlock(&g_outq_lock, key);
}

/* Called from the RF loop: take one queued report, if any. */
static bool out_dequeue(uint8_t *buf, uint16_t *len)
{
	k_spinlock_key_t key = k_spin_lock(&g_outq_lock);
	bool have = false;

	if (g_outq_head != g_outq_tail) {
		*len = g_outq[g_outq_tail].len;
		memcpy(buf, g_outq[g_outq_tail].data, *len);
		g_outq_tail = (uint8_t)((g_outq_tail + 1) % OUTQ_N);
		have = true;
	}
	k_spin_unlock(&g_outq_lock, key);
	return have;
}

static void pro_output_report(const struct device *dev, const uint16_t len,
			      const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	/* USB thread. Copy only. See the note on the queue above. */
	out_enqueue(buf, len);
}

static int pro_set_report(const struct device *dev, const uint8_t type,
			  const uint8_t id, const uint16_t len,
			  const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(id);
	/* Control pipe, same rule: buf[0] already echoes the report id. */
	out_enqueue(buf, len);
	return 0;
}

static int pro_get_report(const struct device *dev, const uint8_t type,
			  const uint8_t id, const uint16_t len,
			  uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);

	/*
	 * Must return a POSITIVE byte count and write the id itself: Zephyr
	 * prepends nothing and stalls the control transfer on any non-positive
	 * return.
	 */
	if (len < 2) {
		return -ENOTSUP;
	}
	buf[0] = id;
	memset(buf + 1, 0, len - 1);
	return (int)len;
}

static void pro_input_done(const struct device *dev,
			   const uint8_t *const report)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(report);
	g_tx_busy = false;
}

static const struct hid_device_ops pro_ops = {
	.get_report = pro_get_report,
	.set_report = pro_set_report,
	.output_report = pro_output_report,
	.input_report_done = pro_input_done,
};

/*
 * A plain boot-protocol mouse, so the trackpads can drive a Switch 2 cursor.
 *
 * RISK TEST ONLY AT THIS STAGE: the descriptor is registered and the interface
 * enumerates, but nothing is ever sent. The question this answers is whether a
 * console still binds the Pro Controller with a mouse beside it. Adding an
 * interface to a working device is what COMBO did, and that went badly on the
 * PC. Decoding the trackpads is the larger job and is wasted if the answer is
 * no.
 */
static const uint8_t MOUSE_HID_DESC[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop) */
	0x09, 0x02,		/* Usage (Mouse) */
	0xA1, 0x01,		/* Collection (Application) */
	0x09, 0x01,		/*   Usage (Pointer) */
	0xA1, 0x00,		/*   Collection (Physical) */
	0x05, 0x09,		/*     Usage Page (Buttons) */
	0x19, 0x01,		/*     Usage Minimum (1) */
	0x29, 0x03,		/*     Usage Maximum (3) */
	0x15, 0x00,		/*     Logical Minimum (0) */
	0x25, 0x01,		/*     Logical Maximum (1) */
	0x95, 0x03,		/*     Report Count (3) */
	0x75, 0x01,		/*     Report Size (1) */
	0x81, 0x02,		/*     Input (Data, Variable, Absolute) */
	0x95, 0x01,		/*     Report Count (1) */
	0x75, 0x05,		/*     Report Size (5), padding */
	0x81, 0x03,		/*     Input (Constant) */
	0x05, 0x01,		/*     Usage Page (Generic Desktop) */
	0x09, 0x30,		/*     Usage (X) */
	0x09, 0x31,		/*     Usage (Y) */
	0x09, 0x38,		/*     Usage (Wheel) */
	0x15, 0x81,		/*     Logical Minimum (-127) */
	0x25, 0x7F,		/*     Logical Maximum (127) */
	0x75, 0x08,		/*     Report Size (8) */
	0x95, 0x03,		/*     Report Count (3) */
	0x81, 0x06,		/*     Input (Data, Variable, RELATIVE) */
	0xC0,			/*   End Collection */
	0xC0			/* End Collection */
};

static const struct device *mouse_dev;

/* One in-flight mouse report, UDC-aligned for the same reason as the pad: the
 * HID class wraps this buffer by reference rather than copying it.
 */
UDC_STATIC_BUF_DEFINE(g_mouse_tx, UDC_ROUND_UP(4));
static volatile bool g_mouse_busy;


static int mouse_get_report(const struct device *dev, const uint8_t type,
			    const uint8_t id, const uint16_t len,
			    uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(id);

	/* REQUIRED by the HID class for every device. Registration fails with
	 * -EINVAL without it, the interface then contributes no descriptor, and
	 * usbd_init() rejects the WHOLE device: "Unknown USB Device (Port Reset
	 * Failed)", console and all. Nothing ever polls a mouse this way, so a
	 * neutral report is the honest answer.
	 */
	if (len < 1) {
		return -ENOTSUP;
	}
	memset(buf, 0, len);
	return (int)len;
}

/*
 * WHAT THE HOST ACTUALLY DID WITH THE MOUSE, ACROSS A POWER CYCLE.
 *
 * The console is the only place this behaviour can be seen and the one place
 * with no console port to watch it on, so the answer has to survive the trip
 * back. Each bit is written once, so flash sees a handful of writes per boot.
 *
 * MEV_READ is the decisive one. input_report_done only fires when the host has
 * actually taken a report off the endpoint, so if it never sets, the console is
 * not polling the mouse interface at all. That is a different problem from a
 * cursor that does not appear.
 */
#define MEV_PROTO_BOOT	 0x01	/* host selected boot protocol */
#define MEV_PROTO_REPORT 0x02	/* host selected report protocol */
#define MEV_SENT	 0x04	/* we submitted a report */
#define MEV_READ	 0x08	/* the host consumed one */
#define MEV_SUBMIT_FAIL	 0x10	/* the endpoint refused one */

static uint8_t g_mev;
static uint8_t g_mev_prev;

static void mouse_evidence(uint8_t bit)
{
	if (g_mev & bit) {
		return;			/* already recorded; no flash write */
	}
	g_mev |= bit;
	switch2_store_save_mouse_ev(g_mev);
}

/*
 * Boot protocol vs report protocol.
 *
 * This interface declares the BOOT subclass, so a host may switch it to boot
 * protocol, and consoles commonly do. In boot protocol a mouse report is
 * EXACTLY three bytes, [buttons][dx][dy], with no wheel: the format is fixed by
 * the HID spec, not by the report descriptor. Sending four bytes to a host that
 * asked for boot is malformed, and it may ignore every one.
 */
static volatile bool g_mouse_boot;

static void mouse_set_protocol(const struct device *dev, const uint8_t proto)
{
	ARG_UNUSED(dev);

	/* 0 = boot, 1 = report (HID 1.11, 7.2.6). */
	g_mouse_boot = (proto == 0);
	mouse_evidence(g_mouse_boot ? MEV_PROTO_BOOT : MEV_PROTO_REPORT);
	printk("mouse: host selected %s protocol\n",
	       g_mouse_boot ? "BOOT" : "report");
}

static void mouse_input_done(const struct device *dev,
			     const uint8_t *const report)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(report);
	mouse_evidence(MEV_READ);
	g_mouse_busy = false;
}

static const struct hid_device_ops mouse_ops = {
	.get_report = mouse_get_report,
	.set_protocol = mouse_set_protocol,
	.input_report_done = mouse_input_done,
};

int switch_pro_mouse_register(void)
{
	mouse_dev = DEVICE_DT_GET(DT_NODELABEL(hid_mouse));

	if (!device_is_ready(mouse_dev)) {
		printk("mouse: HID device not ready\n");
		return -ENODEV;
	}

	int err = hid_device_register(mouse_dev, MOUSE_HID_DESC,
				      sizeof MOUSE_HID_DESC, &mouse_ops);

	if (err) {
		/* Do NOT step over this. An unregistered HID interface adds no
		 * descriptor and usbd_init() then rejects the entire device.
		 */
		printk("mouse: hid_device_register FAILED (%d): dropping the "
		       "mouse so the device still assembles\n", err);
		mouse_dev = NULL;
		return err;
	}
	printk("mouse: interface registered (%u-byte descriptor, sends nothing "
	       "yet)\n", (unsigned)sizeof MOUSE_HID_DESC);
	return 0;
}


int switch_pro_usb_register(void)
{
	pro_dev = DEVICE_DT_GET(DT_NODELABEL(hid_pro));

	build_stick_cal();
	puck_rumble_init();

	/* Read what the LAST session managed before this one overwrites it.
	 * The console trip is the interesting one and it is always a power
	 * cycle away from being readable.
	 */
	g_mev_prev = switch2_store_load_mouse_ev();
	g_mev = 0;
	switch2_store_save_mouse_ev(0);

	g_rumble_flags_prev = switch2_store_load_rumble();
	g_rumble_flags = 0;
	switch2_store_save_rumble(0);
	build_mac();

	if (!device_is_ready(pro_dev)) {
		printk("pro: HID device not ready\n");
		return -ENODEV;
	}

	hid_device_register(pro_dev, SWPRO_HID_DESC, sizeof SWPRO_HID_DESC,
			    &pro_ops);
	puck_trace_stage(TR_PRO_REG);
	printk("pro: interface registered (%u-byte descriptor, "
	       "MAC %02X:%02X:%02X:%02X:%02X:%02X)\n",
	       (unsigned)sizeof SWPRO_HID_DESC, g_mac[0], g_mac[1], g_mac[2],
	       g_mac[3], g_mac[4], g_mac[5]);
	return 0;
}

static void pro_send(uint8_t rid, const uint8_t *body)
{
	if (g_tx_busy) {
		return;		/* the host has not taken the last one yet */
	}
	g_pro_tx[0] = rid;
	memcpy(g_pro_tx + 1, body, JC_REPLEN);
	g_tx_busy = true;

	if (hid_device_submit_report(pro_dev, JC_REPLEN + 1, g_pro_tx)) {
		/* Not enabled yet, or the endpoint refused it. Release the flag
		 * or nothing is ever sent again.
		 */
		g_tx_busy = false;
		return;
	}
	g_tx_reports++;
}

void switch_pro_usb_task(void)
{
	uint32_t now = k_uptime_get_32();
	k_spinlock_key_t key;
	uint8_t rid = 0;
	uint8_t body[JC_REPLEN];
	bool have = false;
	struct puck_input in;

	if (!pro_dev) {
		return;
	}

	/*
	 * Once per pass, and only here. The Steam button's HOME decision runs on
	 * time, so it must not advance again for every reply that reads buttons.
	 */
	switch_pro_home_update(live_input(&in) ? &in : NULL);

	/*
	 * Persist the console's motion calibration, debounced so a calibration
	 * burst becomes one flash write rather than dozens.
	 *
	 * A dock cuts USB power whenever the console sleeps, so without this
	 * every sleep silently throws away the calibration the user just
	 * performed.
	 */
	if (g_user_cal_dirty && (now - g_user_cal_dirty_ms) > 250u) {
		g_user_cal_dirty = false;
		switch2_store_save_user_cal(g_user_cal);
		printk("pro: user calibration saved\n");
	}

	/*
	 * Process anything the console sent FIRST, here in the RF loop.
	 * The callback only copied it. One per pass, so a bursty init cannot
	 * monopolise the loop the RF poll shares.
	 */
	{
		uint8_t inbuf[64];
		uint16_t inlen;

		if (out_dequeue(inbuf, &inlen)) {
			pro_out(inbuf, inlen);
		}
	}

	/* Handshake replies first, in order, one per pass. */
	key = k_spin_lock(&g_q_lock);
	if (g_qh != g_qt) {
		rid = g_q[g_qh].rid;
		memcpy(body, g_q[g_qh].data, JC_REPLEN);
		g_qh = (uint8_t)((g_qh + 1) % JCQ_N);
		have = true;
	}
	k_spin_unlock(&g_q_lock, key);

	if (have) {
		pro_send(rid, body);
		return;
	}

	if (g_report_mode != 0x30) {
		return;		/* the console has not asked for input yet */
	}
	if ((now - g_last_stream_ms) < STREAM_MS) {
		return;
	}
	g_last_stream_ms = now;

	build_0x30(body);
	pro_send(0x30, body);
}

void switch_pro_usb_dump(void)
{
	printk("pro: mode=0x%02X %s  rx=%u tx=%u  ep=%s  txq=%u inq=%u indrop=%u\n",
	       g_report_mode,
	       g_report_mode == 0x30 ? "STREAMING" : "(awaiting handshake)",
	       g_rx_reports, g_tx_reports, g_tx_busy ? "busy" : "idle",
	       (unsigned)((JCQ_N + g_qt - g_qh) % JCQ_N),
	       (unsigned)((OUTQ_N + g_outq_head - g_outq_tail) % OUTQ_N),
	       g_outq_dropped);
	printk("pro: rumble frames=%u nonzero=%u relayed=%u  (this boot)\n",
	       g_rumble_frames, g_rumble_nonzero, g_rumble_relayed);
	printk("pro: rumble seen-since-last-erase: frames=%c audible=%c relayed=%c\n",
	       (g_rumble_flags_prev & RF_SEEN) ? 'y' : 'n',
	       (g_rumble_flags_prev & RF_AUDIBLE) ? 'y' : 'n',
	       (g_rumble_flags_prev & RF_RELAYED) ? 'y' : 'n');
	printk("mouse: this boot proto=%s sent=%c read=%c fail=%c\n",
	       (g_mev & MEV_PROTO_BOOT) ? "boot" :
	       (g_mev & MEV_PROTO_REPORT) ? "report" : "(never set)",
	       (g_mev & MEV_SENT) ? 'y' : 'n',
	       (g_mev & MEV_READ) ? 'y' : 'n',
	       (g_mev & MEV_SUBMIT_FAIL) ? 'y' : 'n');
	printk("mouse: PREVIOUS boot proto=%s sent=%c read=%c fail=%c\n",
	       (g_mev_prev & MEV_PROTO_BOOT) ? "boot" :
	       (g_mev_prev & MEV_PROTO_REPORT) ? "report" : "(never set)",
	       (g_mev_prev & MEV_SENT) ? 'y' : 'n',
	       (g_mev_prev & MEV_READ) ? 'y' : 'n',
	       (g_mev_prev & MEV_SUBMIT_FAIL) ? 'y' : 'n');
	{
		/*
		 * Is there a stored motion calibration, and did it come back?
		 *
		 * Blank is 0xFF (that is how the console knows to fall back to
		 * the factory blocks), so counting non-blank bytes distinguishes
		 * "never calibrated" from "calibrated and persisted". The
		 * console never prompts for calibration, so its behaviour gives
		 * no signal either way. This is the only way to see whether the
		 * SPI write reached flash and was reloaded.
		 */
		unsigned used = 0;

		for (unsigned i = 0; i < sizeof g_user_cal; i++) {
			if (g_user_cal[i] != 0xFF) {
				used++;
			}
		}
		printk("pro: user cal %u/%u bytes written%s\n", used,
		       (unsigned)sizeof g_user_cal,
		       used ? "" : "  (never calibrated, or not persisted)");
	}
}

/*
 * THE RIGHT TRACKPAD AS A MOUSE.
 *
 * The pad reports an ABSOLUTE position and a mouse wants a RELATIVE delta, so
 * movement is the difference between successive touched samples.
 *
 * WHILE A FINGER IS DOWN THE CURSOR TRACKS IT ONE-TO-ONE. Stop moving and the
 * cursor stops immediately. Accumulating velocity and decaying it every pass,
 * which is what the Arduino build did, leaves a finger held still on the pad
 * with velocity still bleeding off, and the cursor feels slippery.
 *
 * GLIDE BELONGS TO THE RELEASE, not to the touch. Flick and lift, and the
 * cursor coasts on the velocity the flick had, decaying away. That is the only
 * time inertia should exist.
 *
 * Two details that make it behave:
 *   - Gate on the touch bit. The coordinates hold their last value after a
 *     finger lifts, so a lift-and-land elsewhere would read as one huge jump.
 *   - Carry the fraction. Dividing pad units down to pixels discards most of a
 *     slow movement; keeping the remainder is what makes it track smoothly
 *     instead of stepping.
 */

/*
 * Speed and glide are live from settings (mouse.speed, mouse.glide); the
 * defaults are in puck_settings.c.
 *
 * mouse.speed is pad units per pixel times ten, and it defaults to 4 where the
 * Arduino build used 64. That build accumulated deltas into a velocity decaying
 * 6% per pass, so at steady state it amplified the finger by about
 * 1/(1-0.94) = 16x before dividing by 640. Tracking one-to-one drops the
 * amplification along with the slide, so the divisor comes down by the same
 * factor to keep the speed: 640/16 = 40.
 */
static float g_vx, g_vy;	/* release glide only, pad units per pass */
static float g_carry_x, g_carry_y;
static int16_t g_prev_x, g_prev_y;
static bool g_was_touching;
static uint8_t g_prev_btns;

/*
 * The floats here are SOFTWARE emulated. CONFIG_FPU is not set, so the
 * nRF52840's hardware FPU is idle and every multiply runs on the order of a
 * hundred cycles. At this call rate that is about 0.3% of the core, and the
 * smoothing reads far better as floats than as fixed point.
 *
 * It would not be fine in the RF poll, which runs far more often and has the
 * radio waiting on it. Keep float out of that path.
 */
void switch_pro_mouse_task(void)
{
	struct puck_input in;
	bool touching;
	float mx = 0.0f, my = 0.0f;
	float fx, fy;
	int dx, dy;
	uint8_t btns;

	if (!mouse_dev) {
		return;
	}
	if (!live_input(&in)) {
		/*
		 * No controller. Stop any glide and let go of the buttons rather
		 * than leave the last click held; with nothing held, nothing is
		 * sent.
		 */
		memset(&in, 0, sizeof(in));
		g_vx = 0.0f;
		g_vy = 0.0f;
		g_was_touching = false;
	}

	touching = (in.buttons & TB_RPADT) != 0;

	if (touching) {
		if (g_was_touching) {
			float ddx = (float)(in.rpx - g_prev_x);
			float ddy = (float)(in.rpy - g_prev_y);

			/* One-to-one with the finger. */
			mx = ddx;
			my = ddy;

			/*
			 * Track a smoothed recent velocity purely so a flick
			 * has something to coast on after release. Smoothed
			 * rather than "the last delta", which is noisy enough
			 * to fling the cursor off on a clean lift.
			 */
			g_vx = g_vx * 0.6f + ddx * 0.4f;
			g_vy = g_vy * 0.6f + ddy * 0.4f;
		} else {
			/* First sample of a touch: no delta yet, and any
			 * leftover glide from the last flick ends here. A
			 * finger landing on the pad stops the cursor.
			 */
			g_vx = 0.0f;
			g_vy = 0.0f;
		}
		g_prev_x = in.rpx;
		g_prev_y = in.rpy;
	} else {
		/* Released: coast on the velocity the flick had. */
		mx = g_vx;
		my = g_vy;
		g_vx *= (float)g_cfg.mouse_fric / 100.0f;
		g_vy *= (float)g_cfg.mouse_fric / 100.0f;
		if (g_vx > -1.0f && g_vx < 1.0f) {
			g_vx = 0.0f;
		}
		if (g_vy > -1.0f && g_vy < 1.0f) {
			g_vy = 0.0f;
		}
	}
	g_was_touching = touching;

	/* Y is inverted: the pad counts upward, screens count downward. */
	fx = mx / (float)(g_cfg.mouse_div * 10) + g_carry_x;
	fy = -(my / (float)(g_cfg.mouse_div * 10)) + g_carry_y;
	dx = (int)fx;
	dy = (int)fy;
	g_carry_x = fx - (float)dx;
	g_carry_y = fy - (float)dy;

	dx = CLAMP(dx, -127, 127);
	dy = CLAMP(dy, -127, 127);

	/* Right pad click = left button, left pad click = right button. */
	btns = ((in.buttons & TB_RPADC) ? 0x01 : 0) |
	       ((in.buttons & TB_LPADC) ? 0x02 : 0);

	if (dx == 0 && dy == 0 && btns == g_prev_btns) {
		return;		/* nothing to say */
	}
	g_prev_btns = btns;

	if (g_mouse_busy) {
		return;		/* host has not taken the last one */
	}
	mouse_evidence(MEV_SENT);
	g_mouse_tx[0] = btns;
	g_mouse_tx[1] = (uint8_t)(int8_t)dx;
	g_mouse_tx[2] = (uint8_t)(int8_t)dy;
	g_mouse_tx[3] = 0;	/* wheel, report protocol only */
	g_mouse_busy = true;

	/* Three bytes in boot protocol, four in report protocol. A four-byte
	 * report to a host that asked for boot is malformed.
	 */
	if (hid_device_submit_report(mouse_dev, g_mouse_boot ? 3 : 4,
				     g_mouse_tx)) {
		g_mouse_busy = false;
		mouse_evidence(MEV_SUBMIT_FAIL);
	}
}

/* How many bytes of the user-calibration mirror are written. Blank is 0xFF,
 * so a non-zero count means the console's calibration reached flash.
 */
void switch_pro_usb_load_cal(void)
{
	memset(g_user_cal, 0xFF, sizeof g_user_cal);
	switch2_store_load_user_cal(g_user_cal);
}

unsigned switch_pro_usb_cal_bytes(void)
{
	unsigned used = 0;

	for (unsigned i = 0; i < sizeof g_user_cal; i++) {
		if (g_user_cal[i] != 0xFF) {
			used++;
		}
	}
	return used;
}
