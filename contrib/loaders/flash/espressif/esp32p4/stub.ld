/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-P4 flasher stub                                   *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the HP-L2MEM.
     ESP32-P4 has unified i/d bus and mapped into below non-cacheable address
     0x8FF00000 - code (OpenOCD workarea address)
     0x8FF04000 - data
  */
  iram : org = 0x8FF00000, len = 0x4000
  dram : org = 0x8FF04000, len = 0x20000
}

INCLUDE stub_common.ld
