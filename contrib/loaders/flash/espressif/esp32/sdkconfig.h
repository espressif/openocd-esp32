/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_SDKCONFIG_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_SDKCONFIG_H

/* Here config defines necessary to compile sources from IDF */

#define CONFIG_IDF_TARGET_ARCH_XTENSA               1
#define CONFIG_IDF_TARGET_ESP32                     1

/* Use ROM flash driver patch */
#define CONFIG_SPI_FLASH_ROM_DRIVER_PATCH           1

/* Disable application module multi-threading lock */
#define CONFIG_APPTRACE_LOCK_ENABLE                 0

/* Enable apptarce module for flash data transfers */
#define CONFIG_APPTRACE_DEST_JTAG                   1
#define CONFIG_APPTRACE_MEMBUFS_APPTRACE_PROTO_ENABLE 1
#define CONFIG_APPTRACE_ENABLE                      1
#define CONFIG_APPTRACE_BUF_SIZE                    0
#define CONFIG_APPTRACE_PENDING_DATA_SIZE_MAX       0

/* Debug UART number */
#define CONFIG_CONSOLE_UART_NUM                     0
/* Debug UART baudrate */
#define CONFIG_CONSOLE_UART_BAUDRATE                115200
/* alloc apptrace data buffers on stack */
#define CONFIG_STUB_STACK_DATA_POOL_SIZE            (2 * CONFIG_APPTRACE_BUF_SIZE)
/* needed due to apptrace sources usage */
#define CONFIG_LOG_MAXIMUM_LEVEL                    0
/* needed due to various checks in IDF headers */
#define CONFIG_FREERTOS_MAX_TASK_NAME_LEN           16
/* TODO: use current clk, get it from PLL settings */
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ           240
/* Unused by stub, just for compilation of IDF */
#define CONFIG_PARTITION_TABLE_OFFSET               0x8000

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_SDKCONFIG_H */
