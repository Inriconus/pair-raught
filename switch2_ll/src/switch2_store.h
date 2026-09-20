/*
 * Persistent storage for the link key (Zephyr NVS on the board's
 * 32 KB storage_partition).
 *
 * The Switch 2 drops dock power when it sleeps, so a wake attempt always starts
 * from a cold boot, and the console expects to encrypt with the bond it stored
 * rather than re-pairing. Without these sixteen bytes surviving the power cut,
 * the autonomous wake path dies at LL_ENC_REQ with "PIN or Key Missing".
 */

#ifndef SWITCH2_STORE_H
#define SWITCH2_STORE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Mount the NVS filesystem. Call once at boot, before bt_enable(). */
int switch2_store_init(void);

/* True once NVS is mounted. Reported periodically, because a failed mount is
 * otherwise completely silent. Nothing is persisted and nothing complains,
 * and the boot-time error is emitted before USB has enumerated.
 */
bool switch2_store_ready(void);

/*
 * Erase everything the puck remembers and leave the store unmounted. Reboot
 * afterwards. There is no way to do this from the bootloader, which will not
 * write to this part of flash.
 */
int switch2_store_erase_all(void);

/* Persist the derived link key, in SMP WIRE ORDER (least significant octet
 * first), the same form bt_keys wants and install_link_key() uses.
 */
int switch2_store_save_key(const uint8_t key[16]);

/* Load the stored key. Returns false if none has been saved. */
bool switch2_store_load_key(uint8_t key[16]);

/* Forget the stored key, for when the console has genuinely dropped the puck and a
 * fresh registration is needed.
 */
int switch2_store_clear_key(void);

/*
 * The bonded console's address, wire order. LEARNED at registration, never
 * compiled in: a hardcoded address is one particular Switch, and a puck built
 * for someone else would advertise inviting a console it has never met.
 *
 * Its presence is also what distinguishes "registered, so advertise in WAKE
 * mode" from "never registered, so advertise for discovery".
 */
int switch2_store_save_host(const uint8_t mac[6]);
bool switch2_store_load_host(uint8_t mac[6]);
int switch2_store_clear_host(void);

/*
 * RE-PAIR REQUEST. A sticky "ignore the stored console and pair again" flag.
 *
 * Normally a puck that already knows a console boots straight into RF, because
 * that is the mode it spends its life in. This flag is the override: set it and
 * the next boot comes up in BLE pairing mode instead, even though a host is
 * stored. It is what the configuration screen's "re-pair" control will set.
 *
 * Cleared by switch2_store_take_repair() so a request survives exactly one boot
 * long enough to register with a console, not long enough to strand the puck
 * off RF if pairing never happens.
 */
int switch2_store_request_repair(void);
bool switch2_store_take_repair(void);


/*
 * A Steam bond slot: the 24-byte record Steam writes over the HID control
 * channel (0xA2). Steam pairs the controller itself, over the CONTROLLER's
 * own USB, and hands both sides the same record; the puck never discovers
 * anything over the air. So this record IS the pairing, and losing it on a
 * power cut means the controller is silently unpaired with nothing to show for
 * it. Slots are numbered by HID interface, interface N owning slot N.
 */
int switch2_store_save_bond(int slot, const uint8_t rec[24]);
bool switch2_store_load_bond(int slot, uint8_t rec[24]);
int switch2_store_clear_bond(int slot);

/*
 * Which USB device the puck presents as. One device, one identity, chosen
 * before enumeration, so it has to persist and is applied at boot.
 *
 * With no explicit choice stored the puck INFERS one: a fresh puck with no
 * controller bonded comes up as the DONGLE, because the only way to bond one is
 * Steam over those interfaces and nothing else could rescue it. Once a
 * controller is bonded it comes up as a PRO CONTROLLER, which is what it is for.
 *
 * A dongle request is one-shot. See switch2_store_take_usb_mode().
 */
#define PUCK_USB_MODE_DONGLE 0
#define PUCK_USB_MODE_PRO    1
#define PUCK_USB_MODE_MOUSE  2	/* mouse ONLY. A test, see puck_usb.c */

int switch2_store_save_usb_mode(uint8_t mode);
uint8_t switch2_store_load_usb_mode(void);

/*
 * Read the USB mode for this boot, consuming a dongle or mouse request so the
 * next boot is a Pro Controller again. A Switch dock drops USB power when the
 * console sleeps, so the puck reboots on every sleep cycle and a temporary
 * mode must not survive that.
 */
uint8_t switch2_store_take_usb_mode(void);

/*
 * The Pro Controller user-calibration mirror (SPI 0x8000-0x80FF). Blank is
 * 0xFF, which is how the console knows to fall back to the factory blocks,
 * load() leaving the buffer untouched on a miss is therefore correct.
 */
int switch2_store_save_user_cal(const uint8_t cal[256]);
bool switch2_store_load_user_cal(uint8_t cal[256]);

/*
 * The boot breadcrumb: the highest stage a boot reached, and a fault code if it
 * died there. Written only on a new high-water mark. See puck_trace.h.
 */
int switch2_store_save_trace(uint8_t stage, uint8_t fault);
bool switch2_store_load_trace(uint8_t *stage, uint8_t *fault);



/*
 * Rumble evidence across a power cycle: bit0 frames arrived, bit1 decoded
 * audible, bit2 relayed. Written once per bit, so the console's behaviour can
 * be read back on a PC afterwards.
 */
int switch2_store_save_rumble(uint8_t flags);
uint8_t switch2_store_load_rumble(void);

/*
 * Which identity the LAST boot came up as, distinct from what is stored,
 * since dongle and mouse-only requests are consumed at boot. Without it there
 * is no way to know a console trip ran in the mode it was meant to.
 */
int switch2_store_save_last_mode(uint8_t mode);
bool switch2_store_load_last_mode(uint8_t *mode);

/* What the host did with the mouse interface, across a power cycle. */
int switch2_store_save_mouse_ev(uint8_t flags);
uint8_t switch2_store_load_mouse_ev(void);

/*
 * The settings record, one NVS id for the whole struct. The length is checked
 * on read so a record from a build with a different struct size is rejected
 * rather than read as garbage.
 */
int switch2_store_save_settings(const void *cfg, size_t len);
bool switch2_store_load_settings(void *cfg, size_t len);

/*
 * A chime's notes, one NVS id per chime (id is a puck_chime_id). Kept out of the
 * settings record, which has to keep its size for older records to load, and
 * length-checked on read the same way.
 */
int switch2_store_save_chime(int id, const void *chime, size_t len);
bool switch2_store_load_chime(int id, void *chime, size_t len);

#endif /* SWITCH2_STORE_H */
