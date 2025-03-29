/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#define ESP_STUB_OK                                     0x0
#define ESP_STUB_FAIL                                   0x1

#define ESP_STUB_ERR_NOT_SUPPORTED                      0x2
#define ESP_STUB_ERR_INFLATE                            0x3
#define ESP_STUB_ERR_NOT_ENOUGH_DATA                    0x4
#define ESP_STUB_ERR_TOO_MUCH_DATA                      0x5
#define ESP_STUB_ERR_INVALID_IMAGE                      0x6
#define ESP_STUB_ERR_INVALID_PARTITION                  0x7
#define ESP_STUB_ERR_INVALID_APP_MAGIC                  0x8
#define ESP_STUB_ERR_READ_PARTITION                     0x10
#define ESP_STUB_ERR_READ_APP_SEGMENT                   0x11
#define ESP_STUB_ERR_READ_APP_IMAGE_HEADER              0x12

#define ESP_STUB_ERR_FLASH_SIZE                         0x100
#define ESP_STUB_ERR_FLASH_READ_UNALIGNED               0x101
#define ESP_STUB_ERR_FLASH_READ                         0x102

#define ESP_STUB_ERR_APPTRACE_CANNOT_SWAP               0x1000
#define ESP_STUB_ERR_APPTRACE_RECV_INVALID_ARG          0x1001
#define ESP_STUB_ERR_APPTRACE_SEND_INVALID_ARG          0x1002
#define ESP_STUB_ERR_APPTRACE_DOWN_BUF_FAIL             0x1003
#define ESP_STUB_ERR_APPTRACE_UP_BUF_FAIL               0x1004
