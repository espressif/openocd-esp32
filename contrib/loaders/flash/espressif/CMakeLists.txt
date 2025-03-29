cmake_minimum_required(VERSION 3.27)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/util.cmake)

validate_esp_target(${ESP_TARGET})

include(${CMAKE_CURRENT_LIST_DIR}/cmake/build-flags.cmake)

setup_toolchain(${ESP_TARGET} "${XTENSA_TARGETS}")

project(openocd_stub LANGUAGES C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

check_toolchain_version(${CMAKE_C_COMPILER} "esp-15.2.0_20250929")

# Add common flags and definitions to all targets
add_compile_options(${TARGET_COMPILER_FLAGS})
add_link_options(${COMMON_LINKER_FLAGS})
add_compile_definitions(${COMMON_COMPILE_DEFS})

set(LINKER_SCRIPTS
    ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}.ld
    ${CMAKE_CURRENT_LIST_DIR}/ld/common.ld
)

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(SRC_FILES
    ${SRC_DIR}/stub_main.c
    ${SRC_DIR}/apptrace/src/apptrace_mem_ctrl.c
    ${SRC_DIR}/apptrace/src/stub_apptrace.c
)
if(ESP_TARGET IN_LIST XTENSA_TARGETS)
    list(APPEND SRC_FILES ${SRC_DIR}/apptrace/src/apptrace_hw_xtensa.c)
else()
    list(APPEND SRC_FILES ${SRC_DIR}/apptrace/src/apptrace_hw_riscv.c)
endif()

# Define command names. Should be the same order as the commands in the esp_stub.h file
set(COMMANDS
    # Test commands
    "cmd_test1"
    "cmd_recv_from_host"
    "cmd_send_to_host"

    # Flash commands
    "cmd_flash_read"
    "cmd_flash_write"
    "cmd_flash_erase"
    "cmd_flash_erase_check"
    "cmd_flash_map_get"
    "cmd_flash_bp_set"
    "cmd_flash_bp_clear"
    "cmd_flash_write_deflated"
    "cmd_flash_calc_hash"
    "cmd_flash_clock_configure"

    # Command groups. Combined with the flash commands
    "cmd_flash_multi_command"
    "cmd_flash_idf_binary"
    "cmd_test_all"
)

# Define enabled macros for each command
define_property(GLOBAL PROPERTY COMMAND_MACROS)
# Test commands
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_test1 STUB_CMD_TEST1)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_recv_from_host STUB_CMD_RECV_FROM_HOST)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_send_to_host STUB_CMD_SEND_TO_HOST)
# Flash commands
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_read STUB_CMD_FLASH_READ)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_write STUB_CMD_FLASH_WRITE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_erase STUB_CMD_FLASH_ERASE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_erase_check STUB_CMD_FLASH_ERASE_CHECK)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_map_get STUB_CMD_FLASH_MAP_GET)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_bp_set STUB_CMD_FLASH_BP_SET)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_bp_clear STUB_CMD_FLASH_BP_CLEAR)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_write_deflated STUB_CMD_FLASH_WRITE_DEFLATED)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_calc_hash STUB_CMD_FLASH_CALC_HASH)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_clock_configure STUB_CMD_FLASH_CLOCK_CONFIGURE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_multi_command STUB_CMD_FLASH_MULTI_COMMAND)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_flash_idf_binary STUB_CMD_FLASH_IDF_BINARY)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_test_all
    STUB_CMD_TEST1
    STUB_CMD_RECV_FROM_HOST
    STUB_CMD_SEND_TO_HOST
    # Flash commands
    STUB_CMD_FLASH_READ
    STUB_CMD_FLASH_WRITE
    STUB_CMD_FLASH_ERASE
    STUB_CMD_FLASH_ERASE_CHECK
    STUB_CMD_FLASH_MAP_GET
    STUB_CMD_FLASH_BP_SET
    STUB_CMD_FLASH_BP_CLEAR
    STUB_CMD_FLASH_WRITE_DEFLATED
    STUB_CMD_FLASH_CALC_HASH
    STUB_CMD_FLASH_CLOCK_CONFIGURE
    STUB_LOG_ENABLED
    STUB_LIB_LOG_BUF
    STUB_LIB_LOG_BUF_SIZE=8192
)

# Create a list to hold all target names
set(ALL_TARGETS)

# Iterate over each command and create a separate target
foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    set(TARGET_NAME stub_${ESP_TARGET}_${COMMAND})
    set(TARGET_ELF ${TARGET_NAME}.elf)
    set(ESP_STUB_LIB esp-stub-lib-${COMMAND})
    set(STUB_COMMAND ${COMMAND})
    string(TOUPPER ${ESP_TARGET} ESP_TARGET_NAME)

    add_executable(${TARGET_ELF}  ${SRC_FILES})

    set(PRIV_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/private
        ${CMAKE_CURRENT_SOURCE_DIR}/src/apptrace/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/test/include
    )

    target_include_directories(${TARGET_ELF}
        PRIVATE ${PRIV_INCLUDE_DIRS}
    )

    foreach(script ${LINKER_SCRIPTS})
        target_link_options(${TARGET_ELF} PRIVATE -T${script})
    endforeach()

    if(${ESP_TARGET} STREQUAL "esp8266")
        target_link_options(${TARGET_ELF} PRIVATE -Wl,--entry=stub_main_esp8266)
    endif()

    get_property(COMMAND_DEFINES GLOBAL PROPERTY COMMAND_MACROS:${COMMAND})

    set(STUB_COMPILE_DEFS "${COMMAND_DEFINES}; ${ESP_TARGET_NAME}")

    add_subdirectory(esp-stub-lib ${ESP_STUB_LIB})

    target_compile_definitions(${TARGET_ELF} PRIVATE ${STUB_COMPILE_DEFS})

    set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.map)
    target_link_options(${TARGET_ELF} PRIVATE -Wl,-Map=${MAP_FILE})
    set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES ${MAP_FILE})

    add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
        COMMAND ${CMAKE_SIZE} ${TARGET_ELF}
        COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${TARGET_ELF}> -M no-aliases > ${CMAKE_BINARY_DIR}/${TARGET_NAME}.asm
    )

    target_link_libraries(${TARGET_ELF} PRIVATE ${ESP_STUB_LIB})

    list(APPEND ALL_TARGETS ${TARGET_ELF})

endforeach()

string(REPLACE ";" "," COMMANDS_LIST "${COMMANDS}")
add_custom_target(generate_image_headers ALL
    DEPENDS ${ALL_TARGETS}
    COMMAND ${CMAKE_COMMAND}
        -DESP_TARGET=${ESP_TARGET}
        -DCOMMANDS="${COMMANDS_LIST}"
        -DCMAKE_OBJCOPY=${CMAKE_OBJCOPY}
        -DCMAKE_READELF=${CMAKE_READELF}
        -P ${CMAKE_CURRENT_LIST_DIR}/cmake/generate_header.cmake
    COMMENT "Generating stub image headers"
)
