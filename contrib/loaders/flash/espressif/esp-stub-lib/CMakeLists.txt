
set(srcs
    src/log.c
    src/flash.c
)

add_library(esp-stub-lib STATIC ${srcs})

target_include_directories(esp-stub-lib
    PUBLIC include
    PRIVATE include/esp-stub-lib
)

add_subdirectory(src/${ESP_TARGET})

target_link_libraries(esp-stub-lib PUBLIC ${ESP_TARGET})
