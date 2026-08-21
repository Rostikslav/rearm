#pragma once

/**
 * Shared protocol definition for the link between the MAIN ESP32
 * and the AUX ESP32 (which drives the claw grab motor).
 *
 * IMPORTANT: this file must be IDENTICAL on both units.
 *
 * Transport: on-board I2C. Main ESP is master, aux ESP is slave.
 * The aux unit shares the bus with the EVShield banks (0x34 / 0x36),
 * so its address must not collide with those.
 *
 * --- Master -> Slave (write) ---
 * Always exactly 4 bytes:
 *   [0] command
 *   [1] payload high byte (int16, big endian)
 *   [2] payload low byte
 *   [3] checksum = byte[0] ^ byte[1] ^ byte[2] ^ AUX_CHECKSUM_SEED
 *
 * --- Slave -> Master (requestFrom) ---
 * Always exactly 4 bytes:
 *   [0] status
 *   [1] current position high byte (int16, motor degrees, big endian)
 *   [2] current position low byte
 *   [3] checksum = byte[0] ^ byte[1] ^ byte[2] ^ AUX_CHECKSUM_SEED
 *
 * The checksum is not cryptographic - it only catches the common failure
 * of a truncated / desynced I2C transfer, which otherwise shows up as the
 * claw moving to a wildly wrong position.
 */

#include <stdint.h>

#define AUX_I2C_ADDR 0x09

#define AUX_MSG_LEN 4
#define AUX_CHECKSUM_SEED 0x5A

// ---- commands (master -> slave) ----

#define AUX_CMD_PING     0x00  // payload ignored
#define AUX_CMD_MOVE_TO  0x01  // payload = absolute target in motor degrees
#define AUX_CMD_MOVE_BY  0x02  // payload = relative delta in motor degrees
#define AUX_CMD_HOME     0x03  // payload ignored; returns to remembered zero
#define AUX_CMD_SET_ZERO 0x04  // payload ignored; declares current pos as zero
#define AUX_CMD_STOP     0x05  // payload ignored; emergency stop

// ---- status (slave -> master) ----

#define AUX_ST_IDLE    0x00  // ready for a new command
#define AUX_ST_BUSY    0x01  // currently moving
#define AUX_ST_UNHOMED 0x02  // zero reference unknown - refuses to move
#define AUX_ST_ERROR   0x03  // last command failed (bad checksum / timeout)

inline uint8_t auxChecksum(uint8_t a, uint8_t b, uint8_t c) {
    return (uint8_t)(a ^ b ^ c ^ AUX_CHECKSUM_SEED);
}
