/*
 * The puck AS a Nintendo Switch Pro Controller over USB.
 *
 * This is the puck's job in normal use: plugged into the console's USB port,
 * presenting as a Pro Controller, driven by the Steam Controller over RF.
 * switch_pro.c decides WHAT the buttons are; this file is how the console is
 * told, and it is a protocol, not just a report. Nothing streams until the
 * console has run its handshake and selected report mode 0x30.
 *
 * Used INSTEAD of the four Valve dongle interfaces (puck_hid.c): one USB
 * device means one identity, chosen at boot from the stored mode.
 */

#ifndef SWITCH_PRO_USB_H
#define SWITCH_PRO_USB_H

#include <stdint.h>

/* Nintendo Co., Ltd. / Pro Controller. The console checks this. */
#define SWPRO_VID 0x057E
#define SWPRO_PID 0x2009

/* Register the report descriptor and callbacks. MUST run before usbd_init(),
 * for the same reason puck_hid_register_all() must: Zephyr's HID class
 * contributes no interface descriptor until the report descriptor is
 * registered, and device assembly then fails with -EINVAL.
 */
int switch_pro_usb_register(void);

/*
 * Stream one 0x30 input report if the console is ready for one. Call often;
 * it paces itself and does nothing until the handshake has selected 0x30.
 */
void switch_pro_usb_task(void);

/* Register the mouse interface. Same timing rule as the Pro Controller: MUST
 * run before usbd_init().
 */
int switch_pro_mouse_register(void);

/*
 * Turn right-trackpad movement into mouse reports. Call from the RF loop, for
 * the same reason the pad stream is driven there rather than from a thread of
 * its own.
 */
void switch_pro_mouse_task(void);

/* Per-slot handshake and stream state. Bound to console key 's'. */
void switch_pro_usb_dump(void);

/* Bytes written in the motion-calibration mirror; 0 means none stored. */
/* Load the stored motion calibration. Call once, in any USB mode. */
void switch_pro_usb_load_cal(void);

unsigned switch_pro_usb_cal_bytes(void);

/*
 * Service the handshake queue and the input stream. Runs on its own thread; the
 * declaration is here so the console can call it directly if ever needed.
 */
void switch_pro_usb_task(void);

#endif /* SWITCH_PRO_USB_H */
