function(validate_esp_target ESP_TARGET)
    if(NOT DEFINED ESP_TARGET)
        message(FATAL_ERROR "ESP_TARGET not defined. Please specify -DESP_TARGET=<target>")
    endif()

    set(ESP8266_TARGET esp8266)
    set(XTENSA_TARGETS esp32 esp32s2 esp32s3)
    set(RISCV_TARGETS esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32h2 esp32h4 esp32p4)
    set(VALID_TARGETS ${ESP8266_TARGET} ${XTENSA_TARGETS} ${RISCV_TARGETS})

    if(NOT ESP_TARGET IN_LIST VALID_TARGETS)
        message(FATAL_ERROR "Invalid ESP_TARGET '${ESP_TARGET}'. Must be one of: ${VALID_TARGETS}")
    endif()

    # Return these lists to parent scope
    set(ESP8266_TARGET ${ESP8266_TARGET} PARENT_SCOPE)
    set(XTENSA_TARGETS ${XTENSA_TARGETS} PARENT_SCOPE)
    set(RISCV_TARGETS ${RISCV_TARGETS} PARENT_SCOPE)
    set(VALID_TARGETS ${VALID_TARGETS} PARENT_SCOPE)
endfunction()

function(setup_toolchain ESP_TARGET XTENSA_TARGETS)
    if(ESP_TARGET IN_LIST XTENSA_TARGETS)
        set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/xtensa-gcc-toolchain.cmake PARENT_SCOPE)
        set(TARGET_COMPILER_FLAGS ${XTENSA_COMPILER_FLAGS} PARENT_SCOPE)
    elseif(ESP_TARGET IN_LIST ESP8266_TARGET)
        set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/esp8266-gcc-toolchain.cmake PARENT_SCOPE)
        set(TARGET_COMPILER_FLAGS ${ESP8266_COMPILER_FLAGS} PARENT_SCOPE)
        set(CMAKE_LINK_DEPENDS_USE_LINKER FALSE PARENT_SCOPE)
    else()
        set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/riscv-gcc-toolchain.cmake PARENT_SCOPE)
        set(TARGET_COMPILER_FLAGS ${RISCV_COMPILER_FLAGS} PARENT_SCOPE)
    endif()

    set(TARGET_LINKER_FLAGS ${COMMON_LINKER_FLAGS} PARENT_SCOPE)
endfunction()

function(check_toolchain_version COMPILER EXPECTED_VERSION)
    execute_process(
        COMMAND ${COMPILER} --version
        RESULT_VARIABLE result
        OUTPUT_VARIABLE gcc_output
        ERROR_VARIABLE gcc_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(result EQUAL 0)
        string(REGEX MATCH "esp-[0-9]+\\.[0-9]+\\.[0-9]+_[0-9]+" toolchain_version ${gcc_output})
        if(toolchain_version STREQUAL ${EXPECTED_VERSION})
            message(STATUS "Found Toolchain version: ${toolchain_version} is OK")
        else()
            message(FATAL_ERROR "Toolchain version mismatch! Found: ${toolchain_version}, but expected: ${EXPECTED_VERSION}")
        endif()
    else()
        message(FATAL_ERROR "Failed to run ${COMPILER} --version: ${gcc_error}")
    endif()
endfunction()
