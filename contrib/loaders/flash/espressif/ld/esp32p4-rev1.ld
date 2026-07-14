/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the HP-L2MEM. ESP32-P4 has unified i/d bus
	* 0x4FF00000 - code (OpenOCD workarea address)
	* 0x4FF04000 - data
	*/
	iram : org = 0x4FF00000, len = 0x4000
	dram : org = 0x4FF04000, len = 0x20000
}
