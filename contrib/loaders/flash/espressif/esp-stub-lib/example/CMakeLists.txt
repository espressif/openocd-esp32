cmake_minimum_required(VERSION 3.27)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/util.cmake)

validate_esp_target(${ESP_TARGET})

include(${CMAKE_CURRENT_LIST_DIR}/cmake/build-flags.cmake)

# Setup toolchain and flags
setup_toolchain(${ESP_TARGET} "${XTENSA_TARGETS}")

# Add compiler and linker flags to all targets
add_compile_options(${TARGET_COMPILER_FLAGS})
add_link_options(${COMMON_LINKER_FLAGS})

# Project setup
set(APP_NAME stub_${ESP_TARGET})
project(${APP_NAME}.elf LANGUAGES C CXX ASM)

# Source and linker configuration
set(SRC_DIR .)
set(LINKER_SCRIPTS
    ${CMAKE_CURRENT_LIST_DIR}/ld/${ESP_TARGET}.ld
    ${CMAKE_CURRENT_LIST_DIR}/ld/common.ld
)
file(GLOB SRC_FILES "${SRC_DIR}/*.c" "${SRC_DIR}/*.S")

# Target configuration
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME}
    PRIVATE include
)
foreach(script ${LINKER_SCRIPTS})
    target_link_options(${PROJECT_NAME} PRIVATE -T${script})
endforeach()

if(${ESP_TARGET} STREQUAL "esp8266")
    target_link_options(${PROJECT_NAME} PRIVATE -Wl,--entry=stub_main_esp8266)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE asm=__asm__ STUB_LOG_ENABLE)

set(MAP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME}.map)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,-Map=${MAP_FILE})
set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES ${MAP_FILE})

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}
    COMMAND ${CMAKE_OBJDUMP} -d $<TARGET_FILE:${APP_NAME}.elf> -M no-aliases > ${CMAKE_BINARY_DIR}/${APP_NAME}.asm
    COMMENT ""
)

get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
add_subdirectory(${PROJECT_ROOT} esp-stub-lib)
target_link_libraries(${PROJECT_NAME} PRIVATE esp-stub-lib)
