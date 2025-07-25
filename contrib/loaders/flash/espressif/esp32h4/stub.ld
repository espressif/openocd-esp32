/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-H4 flasher stub                                   *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the HPSRAM. ESP32-H4 has unified i/d bus
    0x40810000 - code (OpenOCD workarea address)
    0x40814000 - data
  */
  iram : org = 0x40810000, len = 0x4000
  dram : org = 0x40814000, len = 0x20000
}

INCLUDE stub_common.ld
