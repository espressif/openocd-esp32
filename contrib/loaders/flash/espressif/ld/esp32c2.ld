/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the Internal SRAM1. OpenOCD will fill the sections using data bus.
	* 0x3FCA0000 - code (OpenOCD workarea address)
	* 0x3FCA4000 - data
	*/
	iram : org = 0x40380000, len = 0x4000
	dram : org = 0x3FCA4000, len = 0x20000
}
