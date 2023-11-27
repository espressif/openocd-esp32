/***************************************************************************
 *   LD script for ESP32-C3 flasher stub                                   *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
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
  /* Place sections by starting from the Internal SRAM1. OpenOCD will fill the sections using data bus.
     0x3FC80000  - code (OpenOCD workarea address)
	 0x3FC84000  - data
  */
  iram : org = 0x40380000, len = 0x4000
  dram : org = 0x3FC84000, len = 0x20000
}

INCLUDE stub_common.ld
