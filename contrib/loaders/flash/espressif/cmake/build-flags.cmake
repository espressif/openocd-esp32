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
    "-Wl,--undefined=stub_trap_entry"
    "-Werror=lto-type-mismatch"
)

set(XTENSA_LINKER_FLAGS
    ${COMMON_LINKER_FLAGS}
)

set(RISCV_LINKER_FLAGS
    ${COMMON_LINKER_FLAGS}
)

set(COMMON_COMPILE_DEFS "asm=__asm__")
