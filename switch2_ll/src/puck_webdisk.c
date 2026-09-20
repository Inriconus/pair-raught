/*
 * Pair-Raught. The config page, carried on the puck itself.
 *
 * In dongle mode the puck also appears as a tiny read-only USB drive holding
 * RAUGHT.HTM. The page talks back to the puck over WebSerial on the CDC
 * console beside it, so the whole configuration surface travels with the
 * hardware: no file to lose, no version of the page that disagrees with the
 * firmware it is configuring.
 *
 * WHY A SYNTHESISED VOLUME AND NOT A REAL FILESYSTEM
 *
 * A RAM disk would take 128 KB of a 256 KB part, and a flash disk would need a
 * partition, a FAT format and a wear story, all to serve one file that never
 * changes between builds. Instead the FAT12 structures are computed per sector
 * as the host asks for them, and the file body is read straight out of the
 * const array the build embeds. No RAM is used at all.
 *
 * Only in DONGLE mode. The console has no use for a drive, and hanging extra
 * interfaces off a working Pro Controller is a risk with nothing to gain.
 *
 * WRITES ARE ACCEPTED AND DISCARDED. Windows writes to any volume it mounts,
 * System Volume Information, indexing, and a drive that fails those writes
 * gets reported to the user as a faulty disk. Since every read is synthesised,
 * anything "written" simply is not there next time, which is what a read-only
 * volume looks like from the host's side anyway.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <string.h>

#include "puck_webdisk.h"
#include "raught_html.h"	/* generated: RAUGHT_HTML[], RAUGHT_HTML_LEN */

#define SECTOR_SIZE	512
#define TOTAL_SECTORS	256			/* a 128 KB volume */
#define RESERVED_SECS	1
#define FAT_SECS	1
#define ROOT_SECS	1			/* 16 entries */
#define ROOT_ENTRIES	16
#define DATA_START	(RESERVED_SECS + FAT_SECS + ROOT_SECS)
#define DATA_CLUSTERS	(TOTAL_SECTORS - DATA_START)

/* Clusters the page occupies, and therefore the chain length in the FAT. */
#define FILE_CLUSTERS	((RAUGHT_HTML_LEN + SECTOR_SIZE - 1) / SECTOR_SIZE)

BUILD_ASSERT(FILE_CLUSTERS <= DATA_CLUSTERS,
	     "raught.html no longer fits the volume: raise TOTAL_SECTORS");
BUILD_ASSERT(DATA_CLUSTERS < 4085, "too many clusters to still be FAT12");

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

/*
 * Sector 0. The BPB describes a floppy-shaped FAT12 volume, which is the shape
 * every host still handles without argument.
 */
static void boot_sector(uint8_t *b)
{
	static const uint8_t JMP[3] = { 0xEB, 0x3C, 0x90 };

	memcpy(b, JMP, sizeof(JMP));
	memcpy(b + 3, "MSDOS5.0", 8);
	put16(b + 11, SECTOR_SIZE);
	b[13] = 1;			/* sectors per cluster */
	put16(b + 14, RESERVED_SECS);
	b[16] = 1;			/* one FAT; a second would be a copy */
	put16(b + 17, ROOT_ENTRIES);
	put16(b + 19, TOTAL_SECTORS);
	b[21] = 0xF8;			/* fixed disk */
	put16(b + 22, FAT_SECS);
	put16(b + 24, 1);		/* sectors per track */
	put16(b + 26, 1);		/* heads */
	b[36] = 0x80;			/* drive number */
	b[38] = 0x29;			/* extended boot signature */
	b[39] = 0x50; b[40] = 0x55; b[41] = 0x43; b[42] = 0x4B;	 /* volume id */
	memcpy(b + 43, "PAIR-RAUGHT", 11);
	memcpy(b + 54, "FAT12   ", 8);
	b[510] = 0x55;
	b[511] = 0xAA;
}

/*
 * Sector 1. FAT12 packs two 12-bit entries into three bytes: the even entry is
 * the low 12 bits, the odd entry the high 12.
 */
static void fat_put(uint8_t *fat, unsigned n, uint16_t val)
{
	unsigned off = n + (n / 2);	/* n * 1.5 */

	if (n & 1) {
		fat[off] = (uint8_t)((fat[off] & 0x0F) | ((val & 0x0F) << 4));
		fat[off + 1] = (uint8_t)(val >> 4);
	} else {
		fat[off] = (uint8_t)(val & 0xFF);
		fat[off + 1] = (uint8_t)((fat[off + 1] & 0xF0) |
					 ((val >> 8) & 0x0F));
	}
}

static void fat_sector(uint8_t *b)
{
	/* Entries 0 and 1 are reserved and carry the media byte. */
	fat_put(b, 0, 0xFF8);
	fat_put(b, 1, 0xFFF);

	/* The file starts at cluster 2 and runs contiguously. */
	for (unsigned i = 0; i < FILE_CLUSTERS; i++) {
		fat_put(b, 2 + i,
			i + 1 == FILE_CLUSTERS ? 0xFFF : (uint16_t)(2 + i + 1));
	}
}

/* One 32-byte directory entry. Names are space-padded 8.3, uppercase. */
static void dir_entry(uint8_t *e, const char *name83, uint8_t attr,
		      uint16_t first_cluster, uint32_t size)
{
	memcpy(e, name83, 11);
	e[11] = attr;
	put16(e + 22, 0x0000);		/* write time */
	put16(e + 24, 0x5A21);		/* write date, 2025-01-01 */
	put16(e + 26, first_cluster);
	e[28] = (uint8_t)(size & 0xFF);
	e[29] = (uint8_t)((size >> 8) & 0xFF);
	e[30] = (uint8_t)((size >> 16) & 0xFF);
	e[31] = (uint8_t)((size >> 24) & 0xFF);
}

static void root_sector(uint8_t *b)
{
	/* 0x08 volume label, then the page itself as read-only + archive. */
	dir_entry(b, "PAIR-RAUGHT", 0x08, 0, 0);
	dir_entry(b + 32, "RAUGHT  HTM", 0x01 | 0x20, 2, RAUGHT_HTML_LEN);
}

static int webdisk_read(struct disk_info *disk, uint8_t *buf,
			uint32_t start, uint32_t count)
{
	ARG_UNUSED(disk);

	for (uint32_t s = start; s < start + count; s++, buf += SECTOR_SIZE) {
		memset(buf, 0, SECTOR_SIZE);

		if (s >= TOTAL_SECTORS) {
			return -EIO;
		}
		if (s == 0) {
			boot_sector(buf);
		} else if (s == RESERVED_SECS) {
			fat_sector(buf);
		} else if (s == RESERVED_SECS + FAT_SECS) {
			root_sector(buf);
		} else if (s >= DATA_START) {
			uint32_t off = (s - DATA_START) * SECTOR_SIZE;

			if (off < RAUGHT_HTML_LEN) {
				uint32_t n = RAUGHT_HTML_LEN - off;

				memcpy(buf, RAUGHT_HTML + off,
				       MIN(n, (uint32_t)SECTOR_SIZE));
			}
		}
	}
	return 0;
}

/* Accepted and dropped. See the note at the top of the file. */
static int webdisk_write(struct disk_info *disk, const uint8_t *buf,
			 uint32_t start, uint32_t count)
{
	ARG_UNUSED(disk); ARG_UNUSED(buf); ARG_UNUSED(start); ARG_UNUSED(count);
	return 0;
}

static int webdisk_ioctl(struct disk_info *disk, uint8_t cmd, void *arg)
{
	ARG_UNUSED(disk);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)arg = TOTAL_SECTORS;
		return 0;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)arg = SECTOR_SIZE;
		return 0;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)arg = 1;
		return 0;
	case DISK_IOCTL_CTRL_SYNC:
	case DISK_IOCTL_CTRL_INIT:
	case DISK_IOCTL_CTRL_DEINIT:
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int webdisk_init(struct disk_info *disk)
{
	ARG_UNUSED(disk);
	return 0;
}

static int webdisk_status(struct disk_info *disk)
{
	ARG_UNUSED(disk);
	return DISK_STATUS_OK;
}

static const struct disk_operations webdisk_ops = {
	.init = webdisk_init,
	.status = webdisk_status,
	.read = webdisk_read,
	.write = webdisk_write,
	.ioctl = webdisk_ioctl,
};

static struct disk_info webdisk = {
	.name = PUCK_WEBDISK_NAME,
	.ops = &webdisk_ops,
};

int puck_webdisk_init(void)
{
	int err = disk_access_register(&webdisk);

	if (err) {
		printk("webdisk: register failed (%d): no config drive\n",
		       err);
		return err;
	}

	printk("webdisk: RAUGHT.HTM ready (%u bytes, %u of %u clusters)\n",
	       (unsigned)RAUGHT_HTML_LEN, (unsigned)FILE_CLUSTERS,
	       (unsigned)DATA_CLUSTERS);
	return 0;
}

/*
 * Bind the volume to a mass-storage LUN. The disk name must match the one
 * puck_webdisk_init() registers, or the host sees a drive with no medium.
 */
USBD_DEFINE_MSC_LUN(cfg, PUCK_WEBDISK_NAME, "Raught", "Config drive", "1.00");
