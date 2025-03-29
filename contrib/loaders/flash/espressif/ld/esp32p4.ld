/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the HP-L2MEM.
	* ESP32-P4 has unified i/d bus and mapped into below non-cacheable address
	* 0x8FF80000 - code (OpenOCD workarea address)
	* 0x8FF84000 - data
	*/
	iram : org = 0x8FF80000, len = 0x4000
	dram : org = 0x8FF84000, len = 0x20000
}
