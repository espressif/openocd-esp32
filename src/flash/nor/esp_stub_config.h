/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H
#define OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H

struct command_map {
	int command;
	const struct esp_flasher_stub_config *config;
};

#include "../../../contrib/loaders/flash/espressif/include/esp_stub.h"

#if defined(ESP_TARGET_ESP32)
#include "../../../contrib/loaders/flash/espressif/images/esp32/stub_image.h"
#elif defined(ESP_TARGET_ESP32S2)
#include "../../../contrib/loaders/flash/espressif/images/esp32s2/stub_image.h"
#elif defined(ESP_TARGET_ESP32S3)
#include "../../../contrib/loaders/flash/espressif/images/esp32s3/stub_image.h"
#elif defined(ESP_TARGET_ESP32C2)
#include "../../../contrib/loaders/flash/espressif/images/esp32c2/stub_image.h"
#elif defined(ESP_TARGET_ESP32C3)
#include "../../../contrib/loaders/flash/espressif/images/esp32c3/stub_image.h"
#elif defined(ESP_TARGET_ESP32C5)
#include "../../../contrib/loaders/flash/espressif/images/esp32c5/stub_image.h"
#elif defined(ESP_TARGET_ESP32C6)
#include "../../../contrib/loaders/flash/espressif/images/esp32c6/stub_image.h"
#elif defined(ESP_TARGET_ESP32C61)
#include "../../../contrib/loaders/flash/espressif/images/esp32c61/stub_image.h"
#elif defined(ESP_TARGET_ESP32H2)
#include "../../../contrib/loaders/flash/espressif/images/esp32h2/stub_image.h"
#elif defined(ESP_TARGET_ESP32H21)
#include "../../../contrib/loaders/flash/espressif/images/esp32h21/stub_image.h"
#elif defined(ESP_TARGET_ESP32H4)
#include "../../../contrib/loaders/flash/espressif/images/esp32h4/stub_image.h"
#elif defined(ESP_TARGET_ESP32P4)
#include "../../../contrib/loaders/flash/espressif/images/esp32p4/stub_image.h"
#endif

#endif /* OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H */
