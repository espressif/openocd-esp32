#ifndef _STUB_SDKCONFIG_H_
#define _STUB_SDKCONFIG_H_

// Here config defines necessary to compile sources from IDF

// Use ROM flash driver patch
#define CONFIG_SPI_FLASH_ROM_DRIVER_PATCH	1
// Disable application module multi-threading lock
#define CONFIG_ESP32_APPTRACE_LOCK_ENABLE	0
// Enable apptarce module for flash data transfers
#define CONFIG_ESP32_APPTRACE_ENABLE		1
// Send over Trace Memory Transport
#define CONFIG_ESP32_APPTRACE_DEST_TRAX		1
// Debug UART number
#define CONFIG_CONSOLE_UART_NUM				0
// Debug UART baudrate
#define CONFIG_CONSOLE_UART_BAUDRATE		115200
// ESP32 xtal freq config
#define CONFIG_ESP32_XTAL_FREQ				0
#define ESP_APPTRACE_DOWN_BUF_SIZE			16384


// needed due to apptrace sources usage
#define CONFIG_LOG_DEFAULT_LEVEL			0
// needed due to various checks in IDF headers
#define CONFIG_FREERTOS_MAX_TASK_NAME_LEN	16
// TODO: use current clk, get it from PLL settings
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ	240

#endif //_STUB_SDKCONFIG_H_