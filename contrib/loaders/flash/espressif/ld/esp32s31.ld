/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the SRAM. ESP32-S31 has unified i/d bus
	* 0x2F000000 - code (OpenOCD workarea address)
	* 0x2F004000 - data
	*/
	iram : org = 0x2F000000, len = 0x4000
	dram : org = 0x2F004000, len = 0x20000
}
