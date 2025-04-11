set(srcs
    src/uart.c
    src/rtc_clk.c
    src/flash.c
)

set(TARGET_NAME ${ESP_TARGET}-${COMMAND})
add_library(${TARGET_NAME} STATIC ${srcs})

target_include_directories(${TARGET_NAME}
    PUBLIC include
    PRIVATE include/target
)

target_link_options(${TARGET_NAME} PUBLIC -T${CMAKE_CURRENT_LIST_DIR}/ld/esp32.rom.ld)

set(ESP_TARGET_LIB ${TARGET_NAME} PARENT_SCOPE)
