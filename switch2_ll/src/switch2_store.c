/*
 * Persistent storage for the link key.
 *
 * WHY THIS IS NOT OPTIONAL.
 *
 * The Switch 2 cuts power to the dock briefly when it goes to sleep, and the
 * puck is USB-powered from that port. So every wake attempt begins with a cold
 * boot, which makes the cold path the main path here rather than an edge case.
 *
 * Measured on air, booting in wake mode with no stored key:
 *
 *     CONNECT_IND interval=4 (5 ms)       the console finds the puck on its own
 *     0x08/0x09 feature, 0x14/0x15 length, 0x16/0x17/0x18 PHY
 *     0x03 LL_ENC_REQ                     it asks to encrypt with the bond it saved
 *     0x04 LL_ENC_RSP
 *     0x11 LL_REJECT_EXT_IND error 0x06   BT_HCI_ERR_PIN_OR_KEY_MISSING
 *     0x02 LL_TERMINATE_IND reason 0x13   the console gives up
 *
 * The console does NOT re-run the PAIR ceremony on a reconnect; it expects the
 * key it stored at registration. So the autonomous wake path fails on exactly
 * one missing thing: sixteen bytes that did not survive the power cut.
 *
 * The LTK and the console's address are both stored. Everything else is either
 * re-derived or re-sent by the console on each connection.
 */

#include <zephyr/kernel.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <string.h>

#include "switch2_store.h"

/* Any stable non-zero ids; NVS keys are per-filesystem.
 *
 * Everything persistent lives here rather than in a filesystem: the whole set
 * is tiny (a 16-byte LTK, a 6-byte console address, and later 4x24 bytes of
 * Steam Controller bonds), and NVS gives wear levelling and atomic writes with
 * none of a filesystem's mount/format failure modes.
 */
#define NVS_ID_LINK_KEY    1
#define NVS_ID_BONDED_HOST 2

static struct nvs_fs fs;
static bool g_mounted;

int switch2_store_init(void)
{
	struct flash_pages_info info;
	int err;

	fs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(fs.flash_device)) {
		printk("store: flash device not ready\n");
		return -ENODEV;
	}

	fs.offset = FIXED_PARTITION_OFFSET(storage_partition);

	/* The sector size must be a whole number of flash pages, so ask the
	 * driver rather than assuming 4096.
	 */
	err = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (err) {
		printk("store: flash_get_page_info_by_offs failed (%d)\n", err);
		return err;
	}

	fs.sector_size = info.size;
	fs.sector_count = 3U;	/* NVS needs at least 2; 3 gives wear headroom */

	err = nvs_mount(&fs);
	if (err == -EDEADLK) {
		/*
		 * The area holds something that is not an NVS filesystem, and
		 * NVS will not clobber it blindly. A board that has run other
		 * firmware has that firmware's data here, for example the
		 * LittleFS image an Arduino build writes to the same partition.
		 *
		 * Erase once and retry. This is safe here because nothing else
		 * on the Zephyr side uses this partition, and the previous
		 * firmware's data is never read back.
		 */
		printk("store: partition is not NVS (-EDEADLK): erasing it "
		       "once (left by earlier firmware) and retrying\n");

		err = flash_erase(fs.flash_device, fs.offset,
				  FIXED_PARTITION_SIZE(storage_partition));
		if (err) {
			printk("store: erase failed (%d)\n", err);
			return err;
		}

		err = nvs_mount(&fs);
	}

	if (err) {
		printk("store: nvs_mount failed (%d)\n", err);
		return err;
	}

	g_mounted = true;
	printk("store: NVS mounted (sector %u x %u at 0x%lx)\n",
	       (unsigned)fs.sector_size, (unsigned)fs.sector_count,
	       (unsigned long)fs.offset);
	return 0;
}

bool switch2_store_ready(void)
{
	return g_mounted;
}

int switch2_store_save_key(const uint8_t key[16])
{
	ssize_t n;

	if (!g_mounted) {
		return -ENODEV;
	}

	/* NVS skips the write if the value is unchanged, so re-saving the same
	 * key on every registration puts no extra wear on the flash.
	 */
	n = nvs_write(&fs, NVS_ID_LINK_KEY, key, 16);
	if (n < 0) {
		printk("store: saving the link key FAILED (%d): an autonomous "
		       "wake after a power cut will not encrypt\n", (int)n);
		return (int)n;
	}

	printk("store: link key persisted\n");
	return 0;
}

bool switch2_store_load_key(uint8_t key[16])
{
	ssize_t n;

	if (!g_mounted) {
		return false;
	}

	n = nvs_read(&fs, NVS_ID_LINK_KEY, key, 16);
	if (n != 16) {
		printk("store: no stored link key (%d): the console will have "
		       "to register us again\n", (int)n);
		return false;
	}

	return true;
}

/*
 * The bonded console's address, wire order.
 *
 * This MUST be learned, not compiled in. A hardcoded address has every puck
 * inviting one particular Switch, and naming the wrong address in the reconnect
 * field the console keys on.
 *
 * Learned from the peer address of the connection that completes registration.
 * No stored host means no registration has ever happened, so the puck
 * advertises for discovery instead of wake.
 */
int switch2_store_save_host(const uint8_t mac[6])
{
	ssize_t n;

	if (!g_mounted) {
		return -ENODEV;
	}

	n = nvs_write(&fs, NVS_ID_BONDED_HOST, mac, 6);
	if (n < 0) {
		printk("store: saving the bonded host FAILED (%d)\n", (int)n);
		return (int)n;
	}

	printk("store: bonded host persisted ..:%02X\n", mac[0]);
	return 0;
}

bool switch2_store_load_host(uint8_t mac[6])
{
	if (!g_mounted) {
		return false;
	}
	return nvs_read(&fs, NVS_ID_BONDED_HOST, mac, 6) == 6;
}

int switch2_store_clear_host(void)
{
	if (!g_mounted) {
		return -ENODEV;
	}

	printk("store: forgetting the bonded host\n");
	return nvs_delete(&fs, NVS_ID_BONDED_HOST);
}

int switch2_store_clear_key(void)
{
	if (!g_mounted) {
		return -ENODEV;
	}

	printk("store: clearing the stored link key\n");
	return nvs_delete(&fs, NVS_ID_LINK_KEY);
}

#define NVS_ID_REPAIR 20

int switch2_store_request_repair(void)
{
	uint8_t v = 1;

	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_REPAIR, &v, 1) < 0 ? -EIO : 0;
}

bool switch2_store_take_repair(void)
{
	uint8_t v = 0;

	if (!g_mounted) {
		return false;
	}
	if (nvs_read(&fs, NVS_ID_REPAIR, &v, 1) != 1 || !v) {
		return false;
	}
	/* Consume it before returning, so a pairing attempt that fails or is
	 * abandoned cannot leave the puck stuck in pairing mode forever.
	 */
	nvs_delete(&fs, NVS_ID_REPAIR);
	return true;
}
#define NVS_ID_BOND_BASE 16

int switch2_store_save_bond(int slot, const uint8_t rec[24])
{
	if (!g_mounted || slot < 0 || slot >= 4) {
		return -EINVAL;
	}
	return nvs_write(&fs, NVS_ID_BOND_BASE + slot, rec, 24) < 0 ? -EIO : 0;
}

bool switch2_store_load_bond(int slot, uint8_t rec[24])
{
	if (!g_mounted || slot < 0 || slot >= 4) {
		return false;
	}
	return nvs_read(&fs, NVS_ID_BOND_BASE + slot, rec, 24) == 24;
}

int switch2_store_clear_bond(int slot)
{
	if (!g_mounted || slot < 0 || slot >= 4) {
		return -EINVAL;
	}
	return nvs_delete(&fs, NVS_ID_BOND_BASE + slot);
}

/*
 * WHICH USB DEVICE THE PUCK IS.
 *
 * The puck cannot be a Valve dongle and a Pro Controller at the same time.
 * One USB device, one identity, chosen before enumeration. So the choice has to
 * survive a reboot, and it is applied at boot before usbd_init().
 *
 * Default (an unwritten cell) is the DONGLE, deliberately: that is the identity
 * Steam pairs through, and a puck that cannot be paired is harder to recover
 * from than one that is not yet a Pro Controller.
 */
#define NVS_ID_USB_MODE 21

int switch2_store_save_usb_mode(uint8_t mode)
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_USB_MODE, &mode, 1) < 0 ? -EIO : 0;
}

/*
 * Is any controller bonded? Decides what an UNCONFIGURED puck comes up as.
 */
static bool any_bond_stored(void)
{
	uint8_t rec[24];

	for (int i = 0; i < 4; i++) {
		if (nvs_read(&fs, NVS_ID_BOND_BASE + i, rec, 24) == 24) {
			return true;
		}
	}
	return false;
}

/*
 * Erase the whole storage partition: bonds, the stored console, the link key,
 * settings, calibration, every counter. Everything the puck remembers.
 *
 * This exists because there is no other way to reach it. The bootloader will
 * not write this far up the flash, so a UF2 that pretends to be a factory
 * reset silently leaves the partition untouched while erasing the application
 * above it, which looks exactly like a wipe and is not one.
 *
 * The caller is expected to reboot afterwards. NVS is left unmounted rather
 * than remounted here, because everything holding a copy of a setting in RAM
 * would otherwise carry on using it.
 */
int switch2_store_erase_all(void)
{
	int err;

	if (!g_mounted) {
		return -EIO;
	}

	err = flash_erase(fs.flash_device, fs.offset,
			  FIXED_PARTITION_SIZE(storage_partition));
	if (err) {
		printk("store: factory erase failed (%d)\n", err);
		return err;
	}

	g_mounted = false;
	printk("store: erased everything: reboot to come up empty\n");
	return 0;
}

uint8_t switch2_store_load_usb_mode(void)
{
	uint8_t v;

	if (!g_mounted || nvs_read(&fs, NVS_ID_USB_MODE, &v, 1) != 1) {
		/*
		 * No explicit choice has been made, so infer one.
		 *
		 * A puck with no controller bonded has nothing to send a
		 * console, and the only way to bond one is Steam over the
		 * dongle interfaces, so it comes up ready to pair. Defaulting
		 * to Pro Controller strands a fresh puck: Steam sees a Pro
		 * Controller rather than a dongle and cannot pair, and the
		 * back-4 + L1 + R1 chord is no help since pressing it needs a
		 * bonded controller.
		 *
		 * Once one is bonded the puck is a Pro Controller.
		 */
		return any_bond_stored() ? PUCK_USB_MODE_PRO
					 : PUCK_USB_MODE_DONGLE;
	}
	if (v == PUCK_USB_MODE_DONGLE || v == PUCK_USB_MODE_MOUSE) {
		return v;
	}
	return PUCK_USB_MODE_PRO;
}

/*
 * The Pro Controller's user-calibration mirror (SPI 0x8000-0x80FF).
 *
 * A real Pro Controller keeps the calibration the console writes during
 * "Calibrate Motion Controls" in its own flash, then reads it back and applies
 * it. Without somewhere to put it the console's calibration silently does
 * nothing, because the next read returns blank.
 */
#define NVS_ID_USER_CAL 22

int switch2_store_save_user_cal(const uint8_t cal[256])
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_USER_CAL, cal, 256) < 0 ? -EIO : 0;
}

bool switch2_store_load_user_cal(uint8_t cal[256])
{
	if (!g_mounted) {
		return false;
	}
	return nvs_read(&fs, NVS_ID_USER_CAL, cal, 256) == 256;
}
/*
 * Take the USB mode for THIS boot, consuming a dongle request.
 *
 * PRO CONTROLLER IS THE STATE ON EVERY POWER-UP. A dock drops USB power when
 * the console sleeps, so the puck reboots constantly in normal use, and a
 * persistent dongle setting would survive straight into the situation it is
 * wrong for: a Valve dongle plugged into a Switch, useless until someone
 * notices.
 *
 * So the dongle is a temporary excursion. The chord, or the console 'm', asks
 * for it; the puck reboots into it once; the request is consumed here; the next
 * boot is a Pro Controller again. Pairing through Steam is a deliberate act at
 * a PC and does not need to outlive a power cycle.
 */
uint8_t switch2_store_take_usb_mode(void)
{
	uint8_t v = switch2_store_load_usb_mode();

	/* Both the dongle and the mouse are temporary excursions: one boot, then
	 * back to being a Pro Controller.
	 */
	if ((v == PUCK_USB_MODE_DONGLE || v == PUCK_USB_MODE_MOUSE) &&
	    g_mounted) {
		nvs_delete(&fs, NVS_ID_USB_MODE);
	}
	return v;
}

/*
 * The boot breadcrumb (see puck_trace.h): how far the last boot got, and
 * whether it died. Two bytes, rewritten only when a boot reaches a stage it has
 * not reached before, so a running puck writes nothing.
 */
#define NVS_ID_TRACE 23

int switch2_store_save_trace(uint8_t stage, uint8_t fault)
{
	uint8_t rec[2] = { stage, fault };

	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_TRACE, rec, 2) < 0 ? -EIO : 0;
}

bool switch2_store_load_trace(uint8_t *stage, uint8_t *fault)
{
	uint8_t rec[2];

	if (!g_mounted || nvs_read(&fs, NVS_ID_TRACE, rec, 2) != 2) {
		return false;
	}
	*stage = rec[0];
	*fault = rec[1];
	return true;
}

/*
 * Rumble evidence that survives a power cycle.
 *
 * The counters in switch_pro_usb.c are RAM, and the puck power-cycles whenever
 * it moves between the console and a PC. The console's rumble behaviour is the
 * measurement that matters and the one that cannot be read live, so it has to
 * survive the trip back to a PC.
 *
 * Three bits, each written ONCE when it first becomes true, so flash sees at
 * most three writes per boot no matter how much rumble streams:
 *   bit0  rumble bytes arrived at all
 *   bit1  something decoded to an audible level
 *   bit2  a level was handed to the RF relay
 */
#define NVS_ID_RUMBLE 24

int switch2_store_save_rumble(uint8_t flags)
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_RUMBLE, &flags, 1) < 0 ? -EIO : 0;
}

uint8_t switch2_store_load_rumble(void)
{
	uint8_t v;

	if (!g_mounted || nvs_read(&fs, NVS_ID_RUMBLE, &v, 1) != 1) {
		return 0;
	}
	return v;
}

/*
 * WHICH IDENTITY THE LAST BOOT ACTUALLY CAME UP AS.
 *
 * Not the same question as "what is stored": the dongle and mouse-only requests
 * are CONSUMED at boot, so by the time anyone asks, the stored value has moved
 * on. Without this there is no way to tell whether a console trip ran in the
 * mode it was meant to. A mouse-only test that quietly ran as a Pro
 * Controller looks exactly like a mouse the console ignored.
 */
#define NVS_ID_LAST_MODE 25

int switch2_store_save_last_mode(uint8_t mode)
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_LAST_MODE, &mode, 1) < 0 ? -EIO : 0;
}

bool switch2_store_load_last_mode(uint8_t *mode)
{
	if (!g_mounted || nvs_read(&fs, NVS_ID_LAST_MODE, mode, 1) != 1) {
		return false;
	}
	return true;
}

/* What the host did with the mouse interface. See MEV_* in switch_pro_usb.c.
 * One write per bit, so a console trip can be read back afterwards.
 */
#define NVS_ID_MOUSE_EV 26

int switch2_store_save_mouse_ev(uint8_t flags)
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_MOUSE_EV, &flags, 1) < 0 ? -EIO : 0;
}

uint8_t switch2_store_load_mouse_ev(void)
{
	uint8_t v;

	if (!g_mounted || nvs_read(&fs, NVS_ID_MOUSE_EV, &v, 1) != 1) {
		return 0;
	}
	return v;
}

/*
 * The settings record (see puck_settings.h). ONE id for the whole struct: a
 * cell per setting would mean a growing pile of ids and no way to distinguish
 * a missing setting from a zero one.
 *
 * The length is passed in and checked on read, so a record written by a build
 * with a different struct size is rejected rather than read as garbage. The
 * version byte inside then handles genuine upgrades.
 */
#define NVS_ID_SETTINGS 27

int switch2_store_save_settings(const void *cfg, size_t len)
{
	if (!g_mounted) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_SETTINGS, cfg, len) < 0 ? -EIO : 0;
}

bool switch2_store_load_settings(void *cfg, size_t len)
{
	if (!g_mounted) {
		return false;
	}
	return nvs_read(&fs, NVS_ID_SETTINGS, cfg, len) == (ssize_t)len;
}

/*
 * The chimes' notes (see puck_melody.h), one id each rather than part of the
 * settings record, which has to keep its size for older records to load.
 * Length-checked on read the same way.
 *
 * 28 was the wake chime's id before there were others, and still is, so a
 * wake chime saved then still loads. Four ids are set aside for chimes.
 */
#define NVS_ID_CHIME_BASE 28
#define NVS_CHIME_IDS     4

int switch2_store_save_chime(int id, const void *chime, size_t len)
{
	if (!g_mounted || id < 0 || id >= NVS_CHIME_IDS) {
		return -EIO;
	}
	return nvs_write(&fs, NVS_ID_CHIME_BASE + id, chime, len) < 0 ? -EIO : 0;
}

bool switch2_store_load_chime(int id, void *chime, size_t len)
{
	if (!g_mounted || id < 0 || id >= NVS_CHIME_IDS) {
		return false;
	}
	return nvs_read(&fs, NVS_ID_CHIME_BASE + id, chime, len) == (ssize_t)len;
}
