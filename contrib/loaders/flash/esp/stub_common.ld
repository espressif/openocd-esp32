/***************************************************************************
 *   Common LD script for ESP32 flasher stub                               *
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

PHDRS
{
  text_phdr PT_LOAD;
  data_phdr PT_LOAD;
}

ENTRY(stub_main)

SECTIONS {
  .iram_org = ORIGIN(iram);
  .iram_len = ABSOLUTE(. + LENGTH(iram));

  .dram_org = ORIGIN(dram);
  .dram_len = ABSOLUTE(. + LENGTH(dram));

  .text : ALIGN(4) {
    *(.literal)
    *(.text .text.*)
  } > iram : text_phdr

  .data : ALIGN(4) {
    _data_start = ABSOLUTE(.);
    *(.data)
    *(.rodata .rodata.*)
    *(.sdata .sdata.*)
    *(.sdata2 .sdata2.*)
    _data_end = ABSOLUTE(.);
  } > dram : data_phdr

  /* need to be put just after .data section, will be allocated in workspace area along with .data */
  .bss (NOLOAD) : ALIGN(4) {
    _bss_start = ABSOLUTE(.);
    *(.bss)
    *(.sbss .sbss.*)
    *(.sbss2 .sbss2.*)
    _bss_end = ABSOLUTE(.);
  } > dram : data_phdr
}
