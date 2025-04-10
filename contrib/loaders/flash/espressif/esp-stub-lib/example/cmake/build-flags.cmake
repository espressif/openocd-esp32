set(COMMON_COMPILER_FLAGS
    -Wall
    -Werror
    -Wextra
    -Wshadow
    -Wundef
    -Wconversion
    -Os
    -nostdlib
    -fno-builtin
    -fno-common
    -g
    -ffunction-sections
    -fdata-sections
    -std=gnu17
)

set(XTENSA_COMPILER_FLAGS
    ${COMMON_COMPILER_FLAGS}
    -mlongcalls
    -mtext-section-literals
    -flto
)

set(ESP8266_COMPILER_FLAGS
    ${COMMON_COMPILER_FLAGS}
    -mlongcalls
    -mtext-section-literals
    -DESP8266
)

set(RISCV_COMPILER_FLAGS
    ${COMMON_COMPILER_FLAGS}
    -flto
)

set(COMMON_LINKER_FLAGS
    "-nodefaultlibs"
    "-Wl,-static"
    "-Wl,--gc-sections"
    "-Wl,--start-group"
    "-lgcc"
    "-lc"
    "-Wl,--end-group"
    "-Wl,--undefined=s_esp_stub_desc"
)
