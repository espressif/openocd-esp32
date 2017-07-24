/***************************************************************************
 *   LD script for ESP32 flassher stub                                     *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

MEMORY {
  /* place stub at the beginning of the OpenOCD working area,
     remaining space will be used for other chunks */
  iram : org = 0x40090000, len = 0x2100
  dram : org = 0x3FFC0000, len = 0xB00
}

PHDRS
{
  text_phdr PT_LOAD;
  data_phdr PT_LOAD;
}

ENTRY(stub_main)

SECTIONS {
  .text : ALIGN(4) {
    *(.literal)
    *(.text .text.*)
  } > iram : text_phdr

  .data : ALIGN(4) {
    *(.data)
    *(.rodata .rodata.*)
  } > dram : data_phdr

  .bss (NOLOAD) : ALIGN(4) {
    _bss_start = ABSOLUTE(.);
    *(.bss)
    _bss_end = ABSOLUTE(.);
  } > dram : data_phdr
}

INCLUDE "esp32.rom.ld"
PROVIDE ( g_rom_spiflash_dummy_len_plus = 0x3ffae290 );
PROVIDE ( ets_update_cpu_frequency = 0x40008550 ); /* Updates g_ticks_per_us on the current CPU only; not on the other core */
PROVIDE ( esp_rom_spiflash_attach = 0x40062a6c );
