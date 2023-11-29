/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32-S3 flasher stub                                   *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/

MEMORY {
  /* Place sections by the offset 0x14000 from the Internal SRAM1. OpenOCD will fill the sections using data bus.
     0x3FC9C000 - code (OpenOCD workarea address)
	 0x3FCA0000 - data
  */
  iram : org = 0x4038C000, len = 0x4000
  dram : org = 0x3FCA0000, len = 0x20000
}

INCLUDE stub_common.ld
