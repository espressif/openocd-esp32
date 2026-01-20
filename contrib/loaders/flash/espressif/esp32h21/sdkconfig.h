/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H21_SDKCONFIG_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H21_SDKCONFIG_H

#define CONFIG_IDF_TARGET                               "esp32h21"
#define CONFIG_IDF_TARGET_ARCH_RISCV                    1
#define CONFIG_IDF_TARGET_ESP32H21                      1
#define CONFIG_FREERTOS_UNICORE                         1
/* Use ROM flash driver patch */
// #define CONFIG_SPI_FLASH_ROM_DRIVER_PATCH            1

/* Debug UART number */
#define CONFIG_CONSOLE_UART_NUM                         0
/* Debug UART baudrate */
#define CONFIG_CONSOLE_UART_BAUDRATE                    115200

/* needed due to apptrace sources usage */
#define CONFIG_LOG_MAXIMUM_LEVEL                        0
/* needed due to various checks in IDF headers */
#define CONFIG_FREERTOS_MAX_TASK_NAME_LEN               16
/* TODO: use current clk, get it from PLL settings */
#define CONFIG_ESP32H21_DEFAULT_CPU_FREQ_MHZ            96
/* Unused by stub, just for compilation of IDF */
#define CONFIG_PARTITION_TABLE_OFFSET                   0x8000
#define CONFIG_MMU_PAGE_SIZE                            0x8000	/* 32KB */
#define CONFIG_HAL_DEFAULT_ASSERTION_LEVEL              0 /* no assert in the hal functions */
#define CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE              1
#define CONFIG_FREERTOS_NUMBER_OF_CORES                 1

#define CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES 1

#define CONFIG_LOG_DEFAULT_LEVEL                        0
#define CONFIG_LIBC_NEWLIB                              1
#define CONFIG_LOG_VERSION                              1

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H21_SDKCONFIG_H */
