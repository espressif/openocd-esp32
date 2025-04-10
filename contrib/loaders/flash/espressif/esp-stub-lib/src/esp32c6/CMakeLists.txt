set(srcs
    src/uart.c
    src/flash.c
)

add_library(${ESP_TARGET} STATIC ${srcs})

target_include_directories(${ESP_TARGET}
    PUBLIC include
    PRIVATE include/target
)

target_link_options(${ESP_TARGET} PUBLIC -T${CMAKE_CURRENT_LIST_DIR}/ld/esp32c6.rom.ld)
