# SPDX-License-Identifier: GPL-2.0-or-later

add_library(espressif STATIC)

target_sources(espressif PRIVATE
	esp_xtensa.c
	esp_xtensa.h
	esp_xtensa_smp.c
	esp_xtensa_smp.h
	esp_xtensa_apptrace.c
	esp_xtensa_semihosting.c
	esp32.c
	esp32s2.c
	esp32s3.c
	esp32c2.c
	esp32c3.c
	esp32c6.c
	esp32h2.c
	esp32p4.c
	esp32h4.c
	esp32c5.c
	esp32c61.c
	esp.c
	esp.h
	esp_riscv.c
	esp_riscv.h
	esp_riscv_apptrace.c
	esp_riscv_apptrace.h
	esp32_apptrace.c
	esp32_apptrace.h
	esp32_sysview.c
	esp32_sysview.h
	segger_sysview.h
	esp_algorithm.c
	esp_algorithm.h
	esp_xtensa_algorithm.c
	esp_xtensa_algorithm.h
	esp_riscv_algorithm.c
	esp_riscv_algorithm.h
	esp_semihosting.c
	esp_semihosting.h
)

if(IS_ESPIDF)
	target_sources(espressif PRIVATE esp_ui_dispatcher.c)
	target_include_directories(espressif PRIVATE ${PROJECT_SOURCE_DIR})
	if(CONFIG_UI_ENABLE)
		target_link_libraries(espressif PRIVATE idf::lvgl__lvgl)
	endif()
endif()
