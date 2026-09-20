/*
 * Bare-metal nRF52 RADIO layer for the Steam Controller link.
 *
 * Ported from the Arduino build's radio.cpp. Every constant is register-confirmed
 * against a capture of a real Valve puck and is CRC-validated, treat it as
 * ground truth, not as something to tune.
 */

#ifndef RF_RADIO_H
#define RF_RADIO_H

#include <stdint.h>

/* RADIO DMA buffers. Must exceed MAXLEN+2: the controller's 0x43-augmented F1
 * reply is ~66 bytes.
 */
#define RF_BUF_LEN 100
extern uint8_t rf_rx[RF_BUF_LEN];
extern uint8_t rf_tx[RF_BUF_LEN];

/* Packet/CRC configuration, decoded from a real puck and CRC-validated. */
#define RF_PCNF0   0x00030008UL	/* S0LEN0, LFLEN8, S1LEN3 (ESB DPL) */
#define RF_PCNF1   0x01040060UL	/* ENDIAN=Big, BALEN4, MAXLEN 0x60 */
#define RF_CRCCNF  0x2		/* CRC16, address included */
#define RF_CRCPOLY 0x11021UL
#define RF_CRCINIT 0xFFFFUL
#define RF_WHITEIV 37

/*
 * Discovery / reconnect rendezvous, firmware-derived and definitive:
 * base = "ibex" (69 62 65 78), prefix = 0x10, channel = 2.
 *
 * Discovery stays on this SHARED address so any controller can find a puck; each
 * bonded controller is then handed its own unique session address, so two
 * controllers never hear each other's polls.
 */
#define RF_PAIR_PREFIX 0x10
#define RF_PAIR_CH     2

/* 1 MHz microsecond clock on TIMER2. Zephyr's k_cycle_get_32() is RTC-backed
 * here and far too coarse for the radio's 800 us windows.
 */
void rf_time_init(void);
uint32_t rf_micros(void);

/* The RADIO cannot transmit without the 16 MHz crystal. With BLE disabled
 * nothing else requests it, so this must be called before any radio work,
 * otherwise the radio looks alive and transmits nothing.
 */
int rf_hfclk_on(void);

/*
 * Deliberately never called. The clock is requested once and held for the
 * firmware's lifetime, because BLE needs it too and handing it back on every
 * radio handover only reintroduces the silent-radio failure above. Holding it
 * draws about a milliamp, off USB. Kept so the pair is complete, and so a
 * battery-powered variant has somewhere to start.
 */
void rf_hfclk_off(void);

uint8_t rf_bitrev8(uint8_t x);
void rf_set_addr(const uint8_t b4[4], uint8_t prefix);
void rf_config(uint8_t ch);
void rf_wait_disabled(void);

#endif /* RF_RADIO_H */
