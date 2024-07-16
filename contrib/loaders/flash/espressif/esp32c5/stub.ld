/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-C6 flasher stub                                   *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the HPSRAM. ESP32-C5 has unified i/d bus
      0x40800000 - code (OpenOCD workarea address)
      0x40804000 - data
  */
  iram : org = 0x40800000, len = 0x4000
  dram : org = 0x40804000, len = 0x20000
}

INCLUDE stub_common.ld
