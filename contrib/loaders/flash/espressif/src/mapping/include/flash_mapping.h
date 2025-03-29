/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>
#include "esp_stub.h"

int stub_flash_mapping(uint32_t app_offset_hint, struct esp_stub_flash_map *out_param);
