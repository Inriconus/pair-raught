/*
 * Bare-metal nRF52 RADIO layer for the Steam Controller link.
 *
 * The register programming is carried across from the Arduino build unchanged.
 * It was decoded from captures of a real Valve puck and is CRC-validated, so it
 * is ground truth rather than something to re-derive. The port only replaces
 * the three things Arduino was providing underneath it:
 *
 *   micros()     -> a 1 MHz TIMER2 (see below)
 *   HFCLK        -> requested explicitly; the SoftDevice used to keep it on
 *   Serial       -> printk, but NEVER inside a radio window (see the note on
 *                   logging at the bottom)
 *
 * RADIO OWNERSHIP. This code and the BLE controller both want the RADIO
 * peripheral and can never run at once, which suits the product: RF is the
 * normal state and BLE is a brief excursion to wake the console. main.c owns
 * the handover (`r` / `e`). bt_disable() does release the radio, and RTC0
 * stopping is the proof the controller really tore down.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>
#include <string.h>

#include "rf_radio.h"

/* ---- microsecond clock -----------------------------------------------------
 *
 * Zephyr's k_cycle_get_32() on this SoC is backed by the 32.768 kHz RTC, giving
 * ~30 us of resolution. The radio code needs to time an 800 us receive window
 * and a 3 ms disable timeout, so that is far too coarse, rounding alone would
 * swallow a whole RX window.
 *
 * TIMER2 at 1 MHz is exact and free-running. TIMER0 belongs to
 * the BLE controller, and although RF only runs while BLE is down, staying off
 * its timer avoids a subtle coupling between the two modes.
 */
void rf_time_init(void)
{
	NRF_TIMER2->TASKS_STOP = 1;
	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER2->PRESCALER = 4;	/* 16 MHz >> 4 = 1 MHz -> 1 tick = 1 us */
	NRF_TIMER2->TASKS_CLEAR = 1;
	NRF_TIMER2->TASKS_START = 1;
}

uint32_t rf_micros(void)
{
	NRF_TIMER2->TASKS_CAPTURE[0] = 1;
	return NRF_TIMER2->CC[0];
}

/* ---- HFCLK -----------------------------------------------------------------
 *
 * The RADIO cannot transmit without the 16 MHz crystal running. The SoftDevice
 * held HFCLK on for its own radio work, so the Arduino build never asked. With
 * BLE disabled under Zephyr nothing requests it, and the symptom is a radio
 * that appears to run while transmitting NOTHING.
 * The same silent-failure shape as the LFCLK problem at the start of this port.
 *
 * So it is requested explicitly, and the result is CHECKED.
 */
/*
 * nRF's HF clock is managed by an onoff manager, not by clock_control_on().
 * That returns -EPERM here. Measured: the request "failed" with -1 while
 * NRF_CLOCK still reported HFCLK running from the crystal, because USB was
 * holding it up for the CDC console.
 *
 * Relying on that would be a trap. USB suspending, or the console interface
 * going away in a headless build, would drop HFCLK and the radio would go
 * silent with nothing to point at. So take an explicit reference and hold it
 * for as long as this layer owns the radio.
 */
static struct onoff_client rf_hf_cli;
static bool rf_hf_held;

int rf_hfclk_on(void)
{
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	int res = 0;
	int err;

	if (rf_hf_held) {
		return 0;
	}
	if (!mgr) {
		printk("rf: no HFCLK onoff manager\n");
		return -ENODEV;
	}

	sys_notify_init_spinwait(&rf_hf_cli.notify);

	err = onoff_request(mgr, &rf_hf_cli);
	if (err < 0) {
		printk("rf: HFCLK request failed (%d): the radio will be silent\n", err);
		return err;
	}

	while (sys_notify_fetch_result(&rf_hf_cli.notify, &res) == -EAGAIN) {
	}
	if (res < 0) {
		printk("rf: HFCLK did not start (%d)\n", res);
		return res;
	}

	rf_hf_held = true;
	return 0;
}

void rf_hfclk_off(void)
{
	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

	if (rf_hf_held && mgr) {
		onoff_release(mgr);
		rf_hf_held = false;
	}
}

/* ---- radio configuration ---------------------------------------------------
 *
 * Register-confirmed against a real puck capture: PHY Ble_2Mbit, ENDIAN=Big
 * (MSB first), whitening off, address "ibex", CRC16 poly 0x11021 init 0xFFFF
 * with the address included. The on-air address is the bit-reverse of the
 * stored bytes, which is ESB's address convention.
 */
uint8_t rf_bitrev8(uint8_t x)
{
	uint8_t y = 0;

	for (int i = 0; i < 8; i++) {
		y = (y << 1) | (x & 1);
		x >>= 1;
	}
	return y;
}

uint8_t rf_rx[RF_BUF_LEN];
uint8_t rf_tx[RF_BUF_LEN];

void rf_set_addr(const uint8_t b4[4], uint8_t prefix)
{
	uint8_t b[4];

	for (int i = 0; i < 4; i++) {
		b[i] = rf_bitrev8(b4[i]);
	}

	NRF_RADIO->BASE0 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
			   ((uint32_t)b[2] << 8) | b[3];
	NRF_RADIO->PREFIX0 = rf_bitrev8(prefix);
	NRF_RADIO->TXADDRESS = 0;
	NRF_RADIO->RXADDRESSES = 1u << 0;
}

/* Bounded wait for the radio to reach DISABLED. NEVER spin forever: a wedged
 * RADIO would hang the caller, and on the Arduino build that meant USB stopped
 * being serviced and the device "died" until replug. A timeout bails out and
 * the next rf_config() re-initialises the radio.
 */
void rf_wait_disabled(void)
{
	uint32_t t0 = rf_micros();

	while (!NRF_RADIO->EVENTS_DISABLED && (rf_micros() - t0) < 3000u) {
	}
}

void rf_config(uint8_t ch)
{
	NRF_RADIO->TASKS_DISABLE = 1;
	rf_wait_disabled();
	NRF_RADIO->EVENTS_DISABLED = 0;

	NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos);
	NRF_RADIO->FREQUENCY = ch;

	/* +8 dBm, the nRF52840's maximum. The Pro Micro's PCB-trace antenna reads
	 * ~20 dB below a real puck and 2.4 GHz is crowded, so a poll the
	 * controller cannot hear produces no reply, which looks exactly like
	 * an intermittent disconnect. The extra current comes off USB.
	 */
	NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
	NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
#endif

	NRF_RADIO->PCNF0 = RF_PCNF0;	/* S0LEN0, LFLEN8, S1LEN3, ESB DPL */
	NRF_RADIO->PCNF1 = RF_PCNF1;	/* ENDIAN=Big, BALEN4, MAXLEN 0x60 */
	NRF_RADIO->CRCCNF = RF_CRCCNF;	/* CRC16, address included */
	NRF_RADIO->CRCPOLY = RF_CRCPOLY;
	NRF_RADIO->CRCINIT = RF_CRCINIT;
	NRF_RADIO->DATAWHITEIV = RF_WHITEIV;
	NRF_RADIO->PACKETPTR = (uint32_t)rf_rx;
}

/*
 * LOGGING INSIDE RADIO WINDOWS, don't.
 *
 * A CDC write with no timeout stalls when the host is slow, and this build sets
 * CONFIG_LOG_PRINTK=n, so printk goes straight to the CDC console and blocks. A
 * print between TXEN and the RX window blows the timing apart.
 *
 * So capture what happened into variables and print after the radio is
 * disabled. Never between.
 */
