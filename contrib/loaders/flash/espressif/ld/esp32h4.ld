/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the SRAM1. ESP32-H4 has unified i/d bus
	* 0x40810000 - code (OpenOCD workarea address)
	* 0x40814000 - data
	*/
	iram : org = 0x40810000, len = 0x4000
	dram : org = 0x40814000, len = 0x20000
}
