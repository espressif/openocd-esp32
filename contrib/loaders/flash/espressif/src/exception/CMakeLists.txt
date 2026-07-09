set(TRAP_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/stub_trap.c
)
if(ESP_TARGET IN_LIST XTENSA_TARGETS)
    list(APPEND TRAP_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/stub_trap_xtensa_vectors.S
    )
else()
    list(APPEND TRAP_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/stub_trap_riscv.c
        ${CMAKE_CURRENT_LIST_DIR}/stub_trap_riscv_entry.S
    )
endif()

add_library(${STUB_TRAP_LIB} STATIC ${TRAP_SOURCES})

target_include_directories(${STUB_TRAP_LIB} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../../include
    ${CMAKE_CURRENT_LIST_DIR}/../../esp-stub-lib/include
)

target_compile_definitions(${STUB_TRAP_LIB} PRIVATE ${STUB_COMPILE_DEFS})
