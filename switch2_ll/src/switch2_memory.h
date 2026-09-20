/*
 * MEMORY reads, ported from BleSwitch2Poc.ino's buildMemoryContent().
 */

#ifndef SWITCH2_MEMORY_H
#define SWITCH2_MEMORY_H

#include <stdint.h>

/*
 * Fill `content` with the flash page the console asked for at `addr`.
 *
 * Always fills the full `content_len`: an address with no capture behind it is
 * answered with 0xFF (erased flash), never left zeroed.
 */
void switch2_memory_build(uint32_t addr, uint8_t *content, uint8_t content_len);

#endif /* SWITCH2_MEMORY_H */
