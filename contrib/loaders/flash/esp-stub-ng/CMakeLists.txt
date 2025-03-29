cmake_minimum_required(VERSION 3.27)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/util.cmake)

validate_esp_target(${ESP_TARGET})

include(${CMAKE_CURRENT_LIST_DIR}/cmake/build-flags.cmake)

setup_toolchain(${ESP_TARGET} "${XTENSA_TARGETS}")

project(openocd_stub LANGUAGES C CXX ASM)

# Add compiler and linker flags to all targets
add_compile_options(${TARGET_COMPILER_FLAGS})
add_link_options(${COMMON_LINKER_FLAGS})

set(SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
set(LINKER_SCRIPTS
    ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}.ld
    ${CMAKE_CURRENT_LIST_DIR}/ld/common.ld
)
file(GLOB SRC_FILES "${SRC_DIR}/*.c" "${SRC_DIR}/*.S")

# Define command names. Should be the same order as the commands in the esp_stub.h file
set(COMMANDS
    "cmd_test1"
    "cmd_test2"
    "cmd_all"
)

# Define enabled macros for each command
define_property(GLOBAL PROPERTY COMMAND_MACROS)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_test1 STUB_CMD_TEST1_ENABLED)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_test2 STUB_CMD_TEST2_ENABLED)
set_property(GLOBAL PROPERTY COMMAND_MACROS:cmd_all 
    STUB_CMD_TEST1_ENABLED STUB_CMD_TEST2_ENABLED STUB_LOG_ENABLE)

add_subdirectory(esp-stub-lib)

# Create a list to hold all target names
set(ALL_TARGETS)

# Iterate over each command and create a separate target
foreach(COMMAND ${COMMANDS})
    string(TOUPPER ${COMMAND} COMMAND_UPPER)
    set(TARGET_NAME stub_${ESP_TARGET}_${COMMAND})
    set(TARGET_ELF ${TARGET_NAME}.elf)

    # Project setup for each command
    add_executable(${TARGET_ELF} ${SRC_FILES})
    target_include_directories(${TARGET_ELF}
        PRIVATE include
    )
    foreach(script ${LINKER_SCRIPTS})
        target_link_options(${TARGET_ELF} PRIVATE -T${script})
    endforeach()

    if(${ESP_TARGET} STREQUAL "esp8266")
        target_link_options(${TARGET_ELF} PRIVATE -Wl,--entry=stub_main_esp8266)
    endif()

    get_property(COMMAND_MACROS GLOBAL PROPERTY COMMAND_MACROS:${COMMAND})
    target_compile_definitions(${TARGET_ELF} PRIVATE asm=__asm__ ${COMMAND_MACROS})

    set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.map)
    target_link_options(${TARGET_ELF} PRIVATE -Wl,-Map=${MAP_FILE})
    set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES ${MAP_FILE})

    add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
        COMMAND ${CMAKE_SIZE} ${TARGET_ELF}
        COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${TARGET_ELF}> -M no-aliases > ${CMAKE_BINARY_DIR}/${TARGET_NAME}.asm
    )

    # Link each target with the built esp-stub-lib
    target_link_libraries(${TARGET_ELF} PRIVATE esp-stub-lib)

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
