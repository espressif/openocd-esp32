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
add_link_options(${TARGET_LINKER_FLAGS})
add_compile_definitions(${COMMON_COMPILE_DEFS})

# Default per-target linker layout. The flash_idf_binary command overrides this
# below with the constrained -idf.ld layout when the target provides one.
if(DEFINED ESP_TARGET_REV)
    set(TARGET_LD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}-${ESP_TARGET_REV}.ld)
    set(TARGET_IDF_LD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}-${ESP_TARGET_REV}-idf.ld)
else()
    set(TARGET_LD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}.ld)
    set(TARGET_IDF_LD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}-idf.ld)
endif()
set(COMMON_LD_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/ld/common.ld)

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(SRC_FILES
    ${SRC_DIR}/stub_main.c
    ${SRC_DIR}/apptrace/src/apptrace_mem_ctrl.c
    ${SRC_DIR}/apptrace/src/stub_apptrace.c
    ${SRC_DIR}/mapping/src/flash_mapping.c
)
if(ESP_TARGET IN_LIST XTENSA_TARGETS)
    list(APPEND SRC_FILES
        ${SRC_DIR}/apptrace/src/apptrace_hw_xtensa.c
    )
else()
    list(APPEND SRC_FILES
        ${SRC_DIR}/apptrace/src/apptrace_hw_riscv.c
    )
endif()

# Define command names. Should be the same order as the commands in the esp_stub.h file
set(COMMANDS
    # Test commands
    "test1"
    "recv_from_host"
    "send_to_host"

    # Flash commands
    "flash_read"
    "flash_write"
    "flash_erase"
    "flash_erase_check"
    "flash_map_get"
    "flash_bp_set"
    "flash_bp_clear"
    "flash_write_deflated"
    "flash_calc_hash"
    "flash_clock_configure"

    # Command groups. Combined with the flash commands
    "flash_multi_command"
    "flash_idf_binary"
    "test_all"
)

# Define enabled macros for each command
define_property(GLOBAL PROPERTY COMMAND_MACROS)
# Test commands
set_property(GLOBAL PROPERTY COMMAND_MACROS:test1 STUB_CMD_TEST1)
set_property(GLOBAL PROPERTY COMMAND_MACROS:recv_from_host STUB_CMD_RECV_FROM_HOST)
set_property(GLOBAL PROPERTY COMMAND_MACROS:send_to_host STUB_CMD_SEND_TO_HOST)
# Flash commands
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_read STUB_CMD_FLASH_READ)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_write STUB_CMD_FLASH_WRITE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_erase STUB_CMD_FLASH_ERASE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_erase_check STUB_CMD_FLASH_ERASE_CHECK)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_map_get STUB_CMD_FLASH_MAP_GET)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_bp_set STUB_CMD_FLASH_BP_SET)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_bp_clear STUB_CMD_FLASH_BP_CLEAR)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_write_deflated STUB_CMD_FLASH_WRITE_DEFLATED)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_calc_hash STUB_CMD_FLASH_CALC_HASH)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_clock_configure STUB_CMD_FLASH_CLOCK_CONFIGURE)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_multi_command STUB_CMD_FLASH_MULTI_COMMAND)
set_property(GLOBAL PROPERTY COMMAND_MACROS:flash_idf_binary STUB_CMD_FLASH_IDF_BINARY)
set_property(GLOBAL PROPERTY COMMAND_MACROS:test_all
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
    STUB_LIB_LOG_LEVEL=STUB_LOG_LEVEL_D
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
        ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping/include
    )

    target_include_directories(${TARGET_ELF}
        PRIVATE ${PRIV_INCLUDE_DIRS}
    )

    if("${COMMAND}" STREQUAL "flash_idf_binary" AND EXISTS ${TARGET_IDF_LD_SCRIPT})
        set(CMD_LINKER_SCRIPTS ${TARGET_IDF_LD_SCRIPT} ${COMMON_LD_SCRIPT})
    else()
        set(CMD_LINKER_SCRIPTS ${TARGET_LD_SCRIPT} ${COMMON_LD_SCRIPT})
    endif()

    foreach(script ${CMD_LINKER_SCRIPTS})
        target_link_options(${TARGET_ELF} PRIVATE -T${script})
    endforeach()

    get_property(COMMAND_DEFINES GLOBAL PROPERTY COMMAND_MACROS:${COMMAND})

    set(STUB_COMPILE_DEFS "${COMMAND_DEFINES}; ${ESP_TARGET_NAME}")
    if(ESP_TARGET IN_LIST RISCV_TARGETS)
        set(STUB_COMPILE_DEFS "${STUB_COMPILE_DEFS}; STUB_ARCH_RISCV")
    elseif(ESP_TARGET IN_LIST XTENSA_TARGETS)
        set(STUB_COMPILE_DEFS "${STUB_COMPILE_DEFS}; STUB_ARCH_XTENSA")
    endif()

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

    # Build and link the trap/exception handler only for commands whose macro set includes STUB_LOG_ENABLED
    if("STUB_LOG_ENABLED" IN_LIST COMMAND_DEFINES)
        set(STUB_TRAP_LIB stub_trap-${COMMAND})
        add_subdirectory(src/exception exception-${COMMAND})
        target_link_libraries(${TARGET_ELF} PRIVATE ${STUB_TRAP_LIB})
        target_link_options(${TARGET_ELF} PRIVATE -Wl,--undefined=stub_trap_entry)
    endif()

    list(APPEND ALL_TARGETS ${TARGET_ELF})

endforeach()

string(REPLACE ";" "," COMMANDS_LIST "${COMMANDS}")
add_custom_target(generate_image_headers ALL
    DEPENDS ${ALL_TARGETS}
    COMMAND ${CMAKE_COMMAND}
        -DESP_TARGET=${ESP_TARGET}
        -DESP_TARGET_REV=${ESP_TARGET_REV}
        -DCOMMANDS="${COMMANDS_LIST}"
        -DCMAKE_OBJCOPY=${CMAKE_OBJCOPY}
        -DCMAKE_READELF=${CMAKE_READELF}
        -P ${CMAKE_CURRENT_LIST_DIR}/cmake/generate_header.cmake
    COMMENT "Generating stub image headers"
)
