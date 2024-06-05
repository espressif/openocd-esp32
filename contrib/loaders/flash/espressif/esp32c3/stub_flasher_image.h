/* SPDX-License-Identifier: GPL-2.0-or-later */

#define ESP_STUB_IRAM_ORG 0x040380000UL
#define ESP_STUB_IRAM_LEN 0x000004000UL
#define ESP_STUB_DRAM_ORG 0x03fc84000UL
#define ESP_STUB_DRAM_LEN 0x000020000UL

#define ESP_STUB_FLASH_READ_BSS_SIZE 0x0000031UL
#define ESP_STUB_FLASH_READ_ENTRY_ADDR 0x040380876UL
#define ESP_STUB_FLASH_READ_APPTRACE_CTRL_ADDR 0x03fc84124UL
#define ESP_STUB_FLASH_READ_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_READ_LOG_SIZE 0UL

#define ESP_STUB_FLASH_WRITE_BSS_SIZE 0x0000050UL
#define ESP_STUB_FLASH_WRITE_ENTRY_ADDR 0x04038084aUL
#define ESP_STUB_FLASH_WRITE_APPTRACE_CTRL_ADDR 0x03fc84114UL
#define ESP_STUB_FLASH_WRITE_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_WRITE_LOG_SIZE 0UL

#define ESP_STUB_FLASH_ERASE_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_ERASE_ENTRY_ADDR 0x040380000UL
#define ESP_STUB_FLASH_ERASE_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_ERASE_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_ERASE_LOG_SIZE 0UL

#define ESP_STUB_FLASH_ERASE_CHECK_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_ERASE_CHECK_ENTRY_ADDR 0x040380194UL
#define ESP_STUB_FLASH_ERASE_CHECK_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_ERASE_CHECK_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_ERASE_CHECK_LOG_SIZE 0UL

#define ESP_STUB_FLASH_MAP_GET_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_MAP_GET_ENTRY_ADDR 0x0403801c8UL
#define ESP_STUB_FLASH_MAP_GET_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_MAP_GET_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_MAP_GET_LOG_SIZE 0UL

#define ESP_STUB_FLASH_BP_SET_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_BP_SET_ENTRY_ADDR 0x04038007eUL
#define ESP_STUB_FLASH_BP_SET_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_BP_SET_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_BP_SET_LOG_SIZE 0UL

#define ESP_STUB_FLASH_BP_CLEAR_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_BP_CLEAR_ENTRY_ADDR 0x04038007eUL
#define ESP_STUB_FLASH_BP_CLEAR_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_BP_CLEAR_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_BP_CLEAR_LOG_SIZE 0UL

#define ESP_STUB_FLASH_TEST_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_TEST_ENTRY_ADDR 0x040380000UL
#define ESP_STUB_FLASH_TEST_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_TEST_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_TEST_LOG_SIZE 0UL

#define ESP_STUB_FLASH_WRITE_DEFLATED_BSS_SIZE 0x0000050UL
#define ESP_STUB_FLASH_WRITE_DEFLATED_ENTRY_ADDR 0x0403808daUL
#define ESP_STUB_FLASH_WRITE_DEFLATED_APPTRACE_CTRL_ADDR 0x03fc84114UL
#define ESP_STUB_FLASH_WRITE_DEFLATED_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_WRITE_DEFLATED_LOG_SIZE 0UL

#define ESP_STUB_FLASH_CALC_HASH_BSS_SIZE 0x00000d8UL
#define ESP_STUB_FLASH_CALC_HASH_ENTRY_ADDR 0x0403801bcUL
#define ESP_STUB_FLASH_CALC_HASH_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_CALC_HASH_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_CALC_HASH_LOG_SIZE 0UL

#define ESP_STUB_FLASH_CLOCK_CONFIGURE_BSS_SIZE 0x00UL
#define ESP_STUB_FLASH_CLOCK_CONFIGURE_ENTRY_ADDR 0x040380048UL
#define ESP_STUB_FLASH_CLOCK_CONFIGURE_APPTRACE_CTRL_ADDR 0x00UL
#define ESP_STUB_FLASH_CLOCK_CONFIGURE_LOG_ADDR 0x00UL
#define ESP_STUB_FLASH_CLOCK_CONFIGURE_LOG_SIZE 0UL

#define ESP_STUB_FLASH_WITH_LOG_BSS_SIZE 0x000112dUL
#define ESP_STUB_FLASH_WITH_LOG_ENTRY_ADDR 0x040381a50UL
#define ESP_STUB_FLASH_WITH_LOG_APPTRACE_CTRL_ADDR 0x03fc852fcUL
#define ESP_STUB_FLASH_WITH_LOG_LOG_ADDR 0x03fc853f8UL
#define ESP_STUB_FLASH_WITH_LOG_LOG_SIZE 4100UL

#include <stdint.h>

static const uint8_t s_esp_flasher_stub_flash_read_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_read_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_read_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_read_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_write_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_write_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_write_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_write_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_erase_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_erase_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_erase_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_erase_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_erase_check_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_erase_check_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_erase_check_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_erase_check_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_map_get_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_map_get_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_map_get_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_map_get_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_bp_set_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_bp_set_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_bp_set_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_bp_set_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_bp_clear_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_bp_clear_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_bp_clear_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_bp_clear_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_test_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_test_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_test_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_test_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_write_deflated_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_write_deflated_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_write_deflated_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_write_deflated_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_calc_hash_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_calc_hash_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_calc_hash_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_calc_hash_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_clock_configure_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_clock_configure_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_clock_configure_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_clock_configure_data.inc"
};
static const uint8_t s_esp_flasher_stub_flash_with_log_code[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_with_log_code.inc"
};
static const uint8_t s_esp_flasher_stub_flash_with_log_data[] = {
#include "contrib/loaders/flash/espressif/esp32c3/inc/stub_flash_with_log_data.inc"
};
