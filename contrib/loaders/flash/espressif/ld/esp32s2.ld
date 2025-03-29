/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the Internal SRAM1. OpenOCD will fill the sections using data bus.
	* 0x3FFB8000 - code (OpenOCD workarea address)
	* 0x3FFBC000 - data
	* 0x3FFF4000 - apptrace_block0
	* 0x3FFF8000 - apptrace_block1
	*/
	iram            : org = 0x40028000, len = 0x4000
	dram            : org = 0x3FFBC000, len = 0x20000
	apptrace_block0 : org = 0x3FFF4000, len = 0x4000
	apptrace_block1 : org = 0x3FFF8000, len = 0x4000
}
