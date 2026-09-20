/*
 * The puck's USB identity: a Valve Steam Controller dongle (28DE:1304) with one
 * HID interface per bond slot, plus the CDC console.
 */

#ifndef PUCK_USB_H
#define PUCK_USB_H

/* One HID interface per bond slot. A real dongle exposes four and Steam treats
 * each as an independent controller, interface N owning slot N.
 */
#define PUCK_HID_SLOTS 4

/*
 * Build and start the USB device. Must run before anything tries to use the
 * console, and replaces Zephyr's cdc_acm_serial auto-init (which is disabled in
 * prj.conf because it hardcodes its own VID/PID and registers only CDC).
 */
int puck_usb_init(void);

/* True if THIS boot came up as a Pro Controller. Ask this rather than re-reading
 * NVS: the stored request is consumed at init while the mode is one-shot.
 */
bool puck_usb_is_pro(void);

/* True if this boot brought up the four Valve dongle interfaces (DONGLE and
 * COMBO). Gate anything that sends to Steam on this.
 */
bool puck_usb_has_dongle(void);

/* True if this boot must hand the radio to RF, meaning anything that forwards the
 * controller onward, which is every mode except the dongle.
 */
bool puck_usb_needs_rf(void);

/* Print this boot's USB identity and the previous one. Bound to console key 's'. */
void puck_usb_dump_mode(void);

#endif /* PUCK_USB_H */
