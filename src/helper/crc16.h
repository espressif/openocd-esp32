/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_HELPER_CRC16_H
#define OPENOCD_HELPER_CRC16_H

#include <stdint.h>

uint16_t crc16_le(uint16_t crc, const uint8_t *buf, unsigned int len);

#endif /* OPENOCD_HELPER_CRC16_H */
