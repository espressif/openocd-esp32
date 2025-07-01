/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#define STUB_APPTRACE_ERR_BASE         0x10000

#define APPTRACE_ERR_OK                 0
#define APPTRACE_ERR_FAIL               (STUB_APPTRACE_ERR_BASE + 0x00)
#define APPTRACE_ERR_CANNOT_SWAP        (STUB_APPTRACE_ERR_BASE + 0x01)
#define APPTRACE_ERR_NOT_INITIALIZED    (STUB_APPTRACE_ERR_BASE + 0x02)
#define APPTRACE_ERR_INVALID_ARG        (STUB_APPTRACE_ERR_BASE + 0x03)
