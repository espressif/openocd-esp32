
set(srcs
    src/log.c
    src/flash.c
)

add_library(${LIB_NAME} STATIC ${srcs})

target_compile_definitions(${LIB_NAME} PRIVATE ${STUB_COMPILE_DEFS})

target_include_directories(${LIB_NAME}
    PUBLIC include
    PRIVATE include/esp-stub-lib
)

add_subdirectory(src/${ESP_TARGET})

target_link_libraries(${LIB_NAME} PUBLIC ${ESP_TARGET_LIB})
