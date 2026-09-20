/*
 * Breadcrumbs for when there is no console.
 *
 * The puck's whole debugging story runs through the CDC console, and the
 * moment it is plugged into the console's own USB port, that channel is gone.
 * So does the log: a firmware that dies in the Switch's port looks exactly like
 * one that is working silently.
 *
 * Worse, that silence is ambiguous. A board that faults keeps its stale USB
 * enumeration for a while, so the host still lists the device and the CDC port
 * still opens. It just never says anything, and a null dereference on the first
 * RF poll is indistinguishable from a USB wedge.
 *
 * So: record how far the boot got, in RAM that survives a warm reboot and in
 * NVS that survives a power cut, and print it on the NEXT boot when the console
 * is available again. Plug the puck back into a PC and it tells you where it
 * died.
 */

#ifndef PUCK_TRACE_H
#define PUCK_TRACE_H

#include <stdint.h>

/* How far the boot got. Ordered: the highest one reached is what gets kept. */
/*
 * ORDER IS THE CONTRACT. Only a HIGHER stage is recorded, so these have to be
 * numbered in the order the calls actually happen or the later ones are thrown
 * away silently. TR_PRO_REG is the trap: it is recorded from inside
 * puck_usb_init(), so it must sit BELOW TR_USB_UP, which main.c records on the
 * line after that call returns. Numbered above it, a Pro-mode boot that died in
 * BT or GATT bring-up reported PRO_REG and hid where it really stopped.
 *
 * Stages 1-8 are the boot sequence and are strictly ordered. 9-12 are
 * post-boot milestones that depend on when a console or a controller shows up,
 * so whichever of those arrives second may be dropped. They all mean "boot
 * finished", which is the question this answers.
 */
#define TR_START	 1
#define TR_STORE	 2	/* NVS mounted */
#define TR_USB_INIT	 3	/* USB identity chosen */
#define TR_PRO_REG	 4	/* Pro Controller interface registered */
#define TR_USB_UP	 5	/* usbd_enable() returned */
#define TR_BT_UP	 6	/* bt_enable() returned */
#define TR_GATT		 7	/* GATT table registered */
#define TR_BOOTMODE	 8	/* radio mode decided */
#define TR_RF_UP	 9	/* radio handed to RF */
#define TR_PRO_HANDSHAKE 10	/* console sent its first 0x80 */
#define TR_PRO_STREAM	11	/* console selected report mode 0x30 */
#define TR_RF_LIVE	12	/* a controller answered a poll */

/* Note a stage. RAM always, flash only when the stage is new. */
void puck_trace_stage(uint8_t stage);

/*
 * Print what the PREVIOUS boot managed, and persist a fault carried over in
 * retained RAM. Call once, as soon as the console can be written to.
 */
void puck_trace_boot_report(void);

/*
 * Print the previous boot outcome, once, from a context where the console
 * exists. MUST NOT be called from early boot: printk before USB has enumerated
 * goes nowhere, taking the record with it.
 */
void puck_trace_report(void);

/* Print the current and stored trace. Bound to console key 's'. */
void puck_trace_dump(void);

#endif /* PUCK_TRACE_H */
