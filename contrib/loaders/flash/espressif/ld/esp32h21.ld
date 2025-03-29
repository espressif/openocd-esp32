/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

MEMORY {
	/*
	* Place sections by starting from the HPSRAM. ESP32-H21 has unified i/d bus
	* 0x40800000 - code (OpenOCD workarea address)
	* 0x40804000 - data
	*/
	iram : org = 0x40800000, len = 0x4000
	dram : org = 0x40804000, len = 0x20000
}
