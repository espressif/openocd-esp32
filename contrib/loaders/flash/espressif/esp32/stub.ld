/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   LD script for ESP32 flassher stub                                  *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/

MEMORY {
  /* Place sections by starting from the Internal SRAM1. OpenOCD will fill the sections using data bus.
     0x3FFE0000 - code (OpenOCD workarea address)
	 0x3FFE4000 - data
	 ESP32 has reverse address mapping. 0x3FFE0000 and 0x400BFFFC access the same word
	 Code will be replaced starting from 0x3FFE4000(0x400BC000) and continue by decreasing the address.
  */
  iram : org = 0x400BC000, len = 0x4000
  dram : org = 0x3FFE4000, len = 0x14000
}

INCLUDE stub_common.ld
