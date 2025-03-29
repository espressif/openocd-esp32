/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

MEMORY {
  /* Place sections by the offset 0x14000 from the Internal SRAM1. OpenOCD will fill the sections using data bus.
     0x3FC9C000 - code (OpenOCD workarea address)
	 0x3FCA0000 - data
  */
  iram : org = 0x4038C000, len = 0x4000
  dram : org = 0x3FCA0000, len = 0x20000
}
