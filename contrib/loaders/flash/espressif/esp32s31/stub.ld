/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-S31 flasher stub                                   *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the HPSRAM. ESP32-S31 has unified i/d bus
    0x2F000000 - code (OpenOCD workarea address)
    0x2F004000 - data
  */
  iram : org = 0x2F000000, len = 0x4000
  dram : org = 0x2F004000, len = 0x20000
}

INCLUDE stub_common.ld
