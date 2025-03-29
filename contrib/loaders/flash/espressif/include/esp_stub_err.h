/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#define ESP_STUB_OK                                     0x0
#define ESP_STUB_FAIL                                   0x1

#define ESP_STUB_ERR_NOT_SUPPORTED                      0x2
#define ESP_STUB_ERR_INFLATE                            0x3
#define ESP_STUB_ERR_NOT_ENOUGH_DATA                    0x4
#define ESP_STUB_ERR_TOO_MUCH_DATA                      0x5
#define ESP_STUB_ERR_TIMEOUT                            0x6
#define ESP_STUB_ERR_FLASH_ENCRYPTION_NOT_ENABLED       0x7
#define ESP_STUB_ERR_INIT_FAILED                        0x8

#define ESP_STUB_ERR_PARTITION_NOT_FOUND                0x50

#define ESP_STUB_ERR_FLASH_ID                           0x100
#define ESP_STUB_ERR_FLASH_READ_UNALIGNED               0x101
#define ESP_STUB_ERR_FLASH_READ                         0x102
#define ESP_STUB_ERR_FLASH_WRITE_UNALIGNED              0x103
#define ESP_STUB_ERR_FLASH_WRITE                        0x104

#define ESP_STUB_ERR_APPTRACE_CANNOT_SWAP               0x1000
#define ESP_STUB_ERR_APPTRACE_RECV_INVALID_ARG          0x1001
#define ESP_STUB_ERR_APPTRACE_SEND_INVALID_ARG          0x1002
#define ESP_STUB_ERR_APPTRACE_DOWN_BUF_FAIL             0x1003
#define ESP_STUB_ERR_APPTRACE_UP_BUF_FAIL               0x1004

#define ESP_STUB_ERR_EXCEPTION                          0x2000
