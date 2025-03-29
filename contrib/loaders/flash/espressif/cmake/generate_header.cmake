get_filename_component(OPENOCD_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../" ABSOLUTE)
get_filename_component(STUB_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(IMAGE_DIR "images")

# When ESP_TARGET_REV is defined, output to a revision-specific directory
# and suffix all generated symbols to avoid collisions when multiple
# revision headers are included in the same translation unit.
if(DEFINED ESP_TARGET_REV AND NOT ESP_TARGET_REV STREQUAL "")
    set(TARGET_DIR "${ESP_TARGET}-${ESP_TARGET_REV}")
    set(SUFFIX "_${ESP_TARGET_REV}")
    string(TOUPPER "_${ESP_TARGET_REV}" SUFFIX_UPPER)
else()
    set(TARGET_DIR "${ESP_TARGET}")
    set(SUFFIX "")
    set(SUFFIX_UPPER "")
endif()

set(OUTPUT_DIR "${STUB_SOURCE_DIR}/${IMAGE_DIR}/${TARGET_DIR}")
set(HEADER_FILE "${OUTPUT_DIR}/stub_image.h")
string(REPLACE "," ";" COMMANDS "${COMMANDS}")

set(BIN2C "${OPENOCD_SOURCE_DIR}/src/helper/bin2char.sh")
file(RELATIVE_PATH BIN2C "${CMAKE_CURRENT_BINARY_DIR}" "${BIN2C}")

if(NOT EXISTS ${BIN2C})
    message(FATAL_ERROR "bin2char.sh not found at ${BIN2C}")
endif()

file(MAKE_DIRECTORY ${OUTPUT_DIR})

# Set defaults
set(ESP_STUB_STACK_SIZE 1024)

file(WRITE ${HEADER_FILE} "/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#define ESP_STUB_STACK_SIZE${SUFFIX_UPPER} ${ESP_STUB_STACK_SIZE}

")

foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    set(STUB_ELF "${CMAKE_CURRENT_BINARY_DIR}/stub_${ESP_TARGET}_${COMMAND}.elf")
    set(CODE_SECTION "${OUTPUT_DIR}/stub_${COMMAND}_code${SUFFIX}.inc")
    set(DATA_SECTION "${OUTPUT_DIR}/stub_${COMMAND}_data${SUFFIX}.inc")

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
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE ENTRY_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w iram_org
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE IRAM_ORG
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w iram_len
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE IRAM_LEN
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w dram_org
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE DRAM_ORG
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w dram_len
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
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

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w g_stub_trap_record
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE TRAP_RECORD_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    execute_process(
        COMMAND ${CMAKE_READELF} -s ${STUB_ELF}
        COMMAND grep -w stub_trap_entry
        COMMAND awk "NR==1 {print $2} END {if (NR==0) print \"0\"}"
        OUTPUT_VARIABLE TRAP_ENTRY_ADDR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    file(APPEND ${HEADER_FILE}
        "// ${COMMAND} definitions\n"
        "#define ESP_STUB_${COMMAND_UPPER}_BSS_SIZE${SUFFIX_UPPER} 0x${BSS_SIZE}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_IRAM_ORG${SUFFIX_UPPER} 0x${IRAM_ORG}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_IRAM_LEN${SUFFIX_UPPER} 0x${IRAM_LEN}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_DRAM_ORG${SUFFIX_UPPER} 0x${DRAM_ORG}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_DRAM_LEN${SUFFIX_UPPER} 0x${DRAM_LEN}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_ENTRY_ADDR${SUFFIX_UPPER} 0x${ENTRY_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_APPTRACE_CTRL_ADDR${SUFFIX_UPPER} 0x${APPTRACE_CTRL_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_LOG_ADDR${SUFFIX_UPPER} 0x${LOG_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_LOG_SIZE${SUFFIX_UPPER} 0x${LOG_SIZE}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_TRAP_RECORD_ADDR${SUFFIX_UPPER} 0x${TRAP_RECORD_ADDR}UL\n"
        "#define ESP_STUB_${COMMAND_UPPER}_TRAP_ENTRY_ADDR${SUFFIX_UPPER} 0x${TRAP_ENTRY_ADDR}UL\n\n"
        "static const uint8_t s_esp_flasher_stub_${COMMAND}_code${SUFFIX}[] = {\n"
        "#include \"contrib/loaders/flash/espressif/${IMAGE_DIR}/${TARGET_DIR}/stub_${COMMAND}_code${SUFFIX}.inc\"\n"
        "};\n\n"
        "static const uint8_t s_esp_flasher_stub_${COMMAND}_data${SUFFIX}[] = {\n"
        "#include \"contrib/loaders/flash/espressif/${IMAGE_DIR}/${TARGET_DIR}/stub_${COMMAND}_data${SUFFIX}.inc\"\n"
        "};\n\n"
    )

    file(APPEND ${HEADER_FILE}
        "static const struct esp_flasher_stub_config s_esp_stub_${COMMAND}_cfg${SUFFIX} = {\n"
        "	.name = \"${COMMAND}\",\n"
        "	.code = s_esp_flasher_stub_${COMMAND}_code${SUFFIX},\n"
        "	.code_sz = sizeof(s_esp_flasher_stub_${COMMAND}_code${SUFFIX}),\n"
        "	.data = s_esp_flasher_stub_${COMMAND}_data${SUFFIX},\n"
        "	.data_sz = sizeof(s_esp_flasher_stub_${COMMAND}_data${SUFFIX}),\n"
        "	.entry_addr = ESP_STUB_${COMMAND_UPPER}_ENTRY_ADDR${SUFFIX_UPPER},\n"
        "	.bss_sz = ESP_STUB_${COMMAND_UPPER}_BSS_SIZE${SUFFIX_UPPER},\n"
        "	.apptrace_ctrl_addr = ESP_STUB_${COMMAND_UPPER}_APPTRACE_CTRL_ADDR${SUFFIX_UPPER},\n"
        "	.stack_default_sz = ESP_STUB_STACK_SIZE${SUFFIX_UPPER},\n"
        "	.log_buff_addr = ESP_STUB_${COMMAND_UPPER}_LOG_ADDR${SUFFIX_UPPER},\n"
        "	.log_buff_size = ESP_STUB_${COMMAND_UPPER}_LOG_SIZE${SUFFIX_UPPER},\n"
        "	.trap_record_addr = ESP_STUB_${COMMAND_UPPER}_TRAP_RECORD_ADDR${SUFFIX_UPPER},\n"
        "	.trap_entry_addr = ESP_STUB_${COMMAND_UPPER}_TRAP_ENTRY_ADDR${SUFFIX_UPPER},\n"
        "	.iram_org = ESP_STUB_${COMMAND_UPPER}_IRAM_ORG${SUFFIX_UPPER},\n"
        "	.iram_len = ESP_STUB_${COMMAND_UPPER}_IRAM_LEN${SUFFIX_UPPER},\n"
        "	.dram_org = ESP_STUB_${COMMAND_UPPER}_DRAM_ORG${SUFFIX_UPPER},\n"
        "	.dram_len = ESP_STUB_${COMMAND_UPPER}_DRAM_LEN${SUFFIX_UPPER},\n"
        "};\n\n"
    )
endforeach()

file(APPEND ${HEADER_FILE} "static const struct command_map s_cmd_map${SUFFIX}[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {\n")
foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    file(APPEND ${HEADER_FILE}
        "	{ESP_STUB_${COMMAND_UPPER}, &s_esp_stub_${COMMAND}_cfg${SUFFIX}},\n"
    )
endforeach()
file(APPEND ${HEADER_FILE} "};\n")
