/*
 * The config page as a USB drive. See puck_webdisk.c.
 */
#ifndef PUCK_WEBDISK_H
#define PUCK_WEBDISK_H

/* Must match the disk name the MSC LUN is defined against. */
#define PUCK_WEBDISK_NAME "PUCKCFG"

/*
 * Register the synthesised volume. Call before registering the MSC class.
 * The LUN is looked up by name when the host first asks for capacity.
 */
int puck_webdisk_init(void);

#endif /* PUCK_WEBDISK_H */
