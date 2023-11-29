/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-C2 flasher stub                                   *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the Internal SRAM1. OpenOCD will fill the sections using data bus.
     0x3FCA0000 - code (OpenOCD workarea address)
	 0x3FCA4000 - data
  */
  iram : org = 0x40380000, len = 0x4000
  dram : org = 0x3FCA4000, len = 0x20000
}

INCLUDE stub_common.ld
