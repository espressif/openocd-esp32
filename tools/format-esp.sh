#!/bin/sh
# Run the beautifier "Uncrustify" on Espressif source files.

# Gcov related files
./tools/uncrustify1.sh src/jtag/drivers/jtag_esp_remote.c
./tools/uncrustify1.sh src/jtag/drivers/esp_usb_jtag.c
./tools/uncrustify1.sh src/target/xtensa/xtensa.c
./tools/uncrustify1.sh src/target/xtensa/xtensa.h
./tools/uncrustify1.sh src/target/xtensa/xtensa_regs.h
./tools/uncrustify1.sh src/target/xtensa/xtensa_algorithm.c
./tools/uncrustify1.sh src/target/xtensa/xtensa_debug_module.c
./tools/uncrustify1.sh src/target/xtensa/xtensa_algorithm.h
./tools/uncrustify1.sh src/target/xtensa/xtensa_debug_module.h
./tools/uncrustify1.sh src/target/espressif/esp_xtensa.c
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_apptrace.c
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_semihosting.c
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_smp.c
./tools/uncrustify1.sh src/target/espressif/esp_riscv_apptrace.c
./tools/uncrustify1.sh src/target/espressif/esp.c
./tools/uncrustify1.sh src/target/espressif/esp32.c
./tools/uncrustify1.sh src/target/espressif/esp32s2.c
./tools/uncrustify1.sh src/target/espressif/esp32s3.c
./tools/uncrustify1.sh src/target/espressif/esp32c3.c
./tools/uncrustify1.sh src/target/espressif/esp32c2.c
./tools/uncrustify1.sh src/target/espressif/esp32_apptrace.c
./tools/uncrustify1.sh src/target/espressif/esp32_sysview.c
./tools/uncrustify1.sh src/target/espressif/esp.h
./tools/uncrustify1.sh src/target/espressif/esp_xtensa.h
./tools/uncrustify1.sh src/target/espressif/esp_riscv.c
./tools/uncrustify1.sh src/target/espressif/esp_riscv.h
./tools/uncrustify1.sh src/target/espressif/esp_semihosting.c
./tools/uncrustify1.sh src/target/espressif/esp_semihosting.h
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_apptrace.h
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_semihosting.h
./tools/uncrustify1.sh src/target/espressif/esp_xtensa_smp.h
./tools/uncrustify1.sh src/target/espressif/esp_riscv_apptrace.h
./tools/uncrustify1.sh src/target/espressif/esp32c2.h
./tools/uncrustify1.sh src/target/espressif/esp32c3.h
./tools/uncrustify1.sh src/target/espressif/esp32_apptrace.h
./tools/uncrustify1.sh src/target/espressif/esp32_sysview.h
./tools/uncrustify1.sh src/target/espressif/esp_algorithm.c
./tools/uncrustify1.sh src/target/espressif/esp_algorithm.h
./tools/uncrustify1.sh src/target/espressif/esp_riscv_algorithm.c
./tools/uncrustify1.sh src/target/espressif/esp_riscv_algorithm.h
./tools/uncrustify1.sh src/flash/nor/esp_flash.h
./tools/uncrustify1.sh src/flash/nor/esp_riscv.h
./tools/uncrustify1.sh src/flash/nor/esp_xtensa.h
./tools/uncrustify1.sh src/flash/nor/esp_flash.c
./tools/uncrustify1.sh src/flash/nor/esp_riscv.c
./tools/uncrustify1.sh src/flash/nor/esp_xtensa.c
./tools/uncrustify1.sh src/flash/nor/esp32s2.c
./tools/uncrustify1.sh src/flash/nor/esp32.c
./tools/uncrustify1.sh src/flash/nor/esp32c2.c
./tools/uncrustify1.sh src/flash/nor/esp32c3.c
./tools/uncrustify1.sh src/flash/nor/esp32s3.c
./tools/uncrustify1.sh src/rtos/FreeRTOS.c
./tools/uncrustify1.sh src/rtos/rtos_nuttx_stackings.c
./tools/uncrustify1.sh src/rtos/nuttx.c

# Stub flasher related files
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/sdkconfig.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/stub_flasher_chip.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/stub_flasher_image.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/stub_flasher_chip.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/stub_spiflash_rom_patch.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32/stub_sha.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s2/sdkconfig.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s2/stub_flasher_chip.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s2/stub_flasher_image.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s2/stub_flasher_chip.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s2/stub_sha.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c2/sdkconfig.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c2/stub_flasher_chip.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c2/stub_flasher_chip.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c2/stub_flasher_image.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c2/stub_sha.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c3/sdkconfig.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c3/stub_flasher_chip.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c3/stub_flasher_chip.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c3/stub_flasher_image.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32c3/stub_sha.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s3/sdkconfig.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s3/stub_flasher_chip.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s3/stub_flasher_chip.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s3/stub_flasher_image.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/esp32s3/stub_sha.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/xtensa/stub_xtensa_chips.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/stub_flasher_int.h
./tools/uncrustify1.sh contrib/loaders/flash/esp/stub_flasher.c
./tools/uncrustify1.sh contrib/loaders/flash/esp/stub_flasher.h
