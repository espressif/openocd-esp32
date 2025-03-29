get_filename_component(OPENOCD_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../" ABSOLUTE)
get_filename_component(STUB_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(IMAGE_DIR "images")
set(OUTPUT_DIR "${STUB_SOURCE_DIR}/${IMAGE_DIR}/${ESP_TARGET}")
set(HEADER_FILE "${OUTPUT_DIR}/stub_image.h")
string(REPLACE "," ";" COMMANDS "${COMMANDS}")

set(BIN2C "${OPENOCD_SOURCE_DIR}/src/helper/bin2char.sh")
file(RELATIVE_PATH BIN2C "${CMAKE_CURRENT_BINARY_DIR}" "${BIN2C}")

if(NOT EXISTS ${BIN2C})
    message(FATAL_ERROR "bin2char.sh not found at ${BIN2C}")
endif()

file(MAKE_DIRECTORY ${OUTPUT_DIR})

# Set defaults
set(ESP_STUB_REVERSE_BINARY 0)
set(ESP_STUB_STACK_SIZE 512)

if(${ESP_TARGET} STREQUAL "esp32")
    set(ESP_STUB_REVERSE_BINARY 1)
elseif(${ESP_TARGET} STREQUAL "esp32p4")
    set(ESP_STUB_STACK_SIZE 1024)
elseif(${ESP_TARGET} STREQUAL "esp32s3")
    set(ESP_STUB_STACK_SIZE 768)
endif()

file(WRITE ${HEADER_FILE} "/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#define ESP_STUB_REVERSE_BINARY ${ESP_STUB_REVERSE_BINARY}
#define ESP_STUB_STACK_SIZE ${ESP_STUB_STACK_SIZE}

")

foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    set(STUB_ELF "${CMAKE_CURRENT_BINARY_DIR}/stub_${ESP_TARGET}_${COMMAND}.elf")
    set(CODE_SECTION "${OUTPUT_DIR}/stub_${COMMAND}_code.inc")
    set(DATA_SECTION "${OUTPUT_DIR}/stub_${COMMAND}_data.inc")

    # Extract code section
    execute_process(
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .text ${STUB_ELF} ${CMAKE_CURRENT_BINARY_DIR}/stub_${COMMAND}_code.bin
    )
    execute_process(
        COMMAND sh -c "${BIN2C} < ${CMAKE_CURRENT_BINARY_DIR}/stub_${COMMAND}_code.bin > ${CODE_SECTION}"
    )

    # Extract data section
    execute_process(
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .data ${STUB_ELF} ${CMAKE_CURRENT_BINARY_DIR}/stub_${COMMAND}_data.bin
    )
    execute_process(
        COMMAND sh -c "${BIN2C} < ${CMAKE_CURRENT_BINARY_DIR}/stub_${COMMAND}_data.bin > ${DATA_SECTION}"
    )

    # Extract symbol information
    execute_process(
        COMMAND ${CMAKE_READELF} -S ${STUB_ELF}
        COMMAND grep -F .bss
        COMMAND awk "NR==1 {print $7} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE BSS_SIZE
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w stub_main
        COMMAND grep FUNC
        COMMAND awk "{print $2}"
        OUTPUT_VARIABLE ENTRY_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w iram_org
        COMMAND awk "{print $2}"
        OUTPUT_VARIABLE IRAM_ORG
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w iram_len
        COMMAND awk "{print $2}"
        OUTPUT_VARIABLE IRAM_LEN
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w dram_org
        COMMAND awk "{print $2}"
        OUTPUT_VARIABLE DRAM_ORG
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w dram_len
        COMMAND awk "{print $2}"
        OUTPUT_VARIABLE DRAM_LEN
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w s_apptrace_ctrl
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE APPTRACE_CTRL_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w g_stub_lib_log_buf
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE LOG_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w g_stub_lib_log_buf
        COMMAND awk "NR==1 {print $3} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE LOG_SIZE
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    file(APPEND ${HEADER_FILE}
        "// ${COMMAND} definitions\n"
        "#define ESP_STUB_${COMMAND_UPPER}_BSS_SIZE 0x${BSS_SIZE}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_IRAM_ORG 0x${IRAM_ORG}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_IRAM_LEN 0x${IRAM_LEN}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_DRAM_ORG 0x${DRAM_ORG}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_DRAM_LEN 0x${DRAM_LEN}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_ENTRY_ADDR 0x${ENTRY_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_APPTRACE_CTRL_ADDR 0x${APPTRACE_CTRL_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_LOG_ADDR 0x${LOG_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_LOG_SIZE 0x${LOG_SIZE}UL\n\n"
        "static const uint8_t s_esp_flasher_stub_${COMMAND}_code[] = {\n"
        "#include \"contrib/loaders/flash/espressif/${IMAGE_DIR}/${ESP_TARGET}/stub_${COMMAND}_code.inc\"\n"
        "};\n\n"
        "static const uint8_t s_esp_flasher_stub_${COMMAND}_data[] = {\n"
        "#include \"contrib/loaders/flash/espressif/${IMAGE_DIR}/${ESP_TARGET}/stub_${COMMAND}_data.inc\"\n"
        "};\n\n"
    )

    file(APPEND ${HEADER_FILE}
        "static const struct esp_flasher_stub_config s_esp_stub_${COMMAND}_cfg = {\n"
        "	.code = s_esp_flasher_stub_${COMMAND}_code,\n"
        "	.code_sz = sizeof(s_esp_flasher_stub_${COMMAND}_code),\n"
        "	.data = s_esp_flasher_stub_${COMMAND}_data,\n"
        "	.data_sz = sizeof(s_esp_flasher_stub_${COMMAND}_data),\n"
        "	.entry_addr = ESP_STUB_${COMMAND_UPPER}_ENTRY_ADDR,\n"
        "	.bss_sz = ESP_STUB_${COMMAND_UPPER}_BSS_SIZE,\n"
        "	.apptrace_ctrl_addr = ESP_STUB_${COMMAND_UPPER}_APPTRACE_CTRL_ADDR,\n"
        "	.stack_default_sz = ESP_STUB_STACK_SIZE,\n"
        "	.log_buff_addr = ESP_STUB_${COMMAND_UPPER}_LOG_ADDR,\n"
        "	.log_buff_size = ESP_STUB_${COMMAND_UPPER}_LOG_SIZE,\n"
        "	.iram_org = ESP_STUB_${COMMAND_UPPER}_IRAM_ORG,\n"
        "	.iram_len = ESP_STUB_${COMMAND_UPPER}_IRAM_LEN,\n"
        "	.dram_org = ESP_STUB_${COMMAND_UPPER}_DRAM_ORG,\n"
        "	.dram_len = ESP_STUB_${COMMAND_UPPER}_DRAM_LEN,\n"
        "	.reverse = ESP_STUB_REVERSE_BINARY,\n"
        "};\n\n"
    )
endforeach()

file(APPEND ${HEADER_FILE} "static const struct command_map s_cmd_map[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {\n")
foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    file(APPEND ${HEADER_FILE}
        "	{ESP_STUB_${COMMAND_UPPER}, &s_esp_stub_${COMMAND}_cfg},\n"
    )
endforeach()
file(APPEND ${HEADER_FILE} "};\n")
