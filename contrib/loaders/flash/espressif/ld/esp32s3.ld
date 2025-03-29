/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by the offset 0x14000 from the Internal SRAM1. OpenOCD will fill the sections using data bus.
	* 0x3FC9C000 - code (OpenOCD workarea address)
	* 0x3FCA0000 - data
	* 0x3FCD0000 - apptrace_block0
	* 0x3FCE0000 - apptrace_block1
	*/
	iram            : org = 0x4038C000, len = 0x4000
	dram            : org = 0x3FCA0000, len = 0x20000
	apptrace_block0 : org = 0x3FCD0000, len = 0x4000
	apptrace_block1 : org = 0x3FCE0000, len = 0x4000
}
