# Espressif Specific Notes

## OpenOCD Configuration Variables 

Espressif specific functionality of OpenOCD can be controled using configuration variables which can be set on the command line via option `-c`:

`openocd -c 'set ESP_RTOS none' -f board/esp32-wrover-kit-3.3v.cfg`

### Common Options

The folowing configuration variables are common for all Espressif chips:
* `ESP_RTOS` - the name of RTOS running on the target. Default is 'FreeRTOS'. To disable OS support (for bare metal system) use 'none'.
* `ESP_FLASH_SIZE` - size of the chip's flash. Default is 'auto'. To disable flash functionality set to '0'.
* `ESP_SEMIHOST_BASEDIR` - base dir for semihosting I/O. Default is OpenOCD's current working directory.

### ESP32 Options

The folowing configuration variables are common for ESP32 family chips:
* `ESP32_ONLYCPU` - the mask indicating which cores are enabled for debugging. Default is '3'. 
  Set to '1' for single core debugging.
* `ESP32_FLASH_VOLTAGE` - tell OpenOCD which SPI flash voltage is used by the board (3.3 or 1.8)
  The TDI pin of ESP32 is also a bootstrap pin that selects the voltage the SPI flash
  chip runs at. When a hard reset happens (e.g. because someone switches the board off
  and on) the ESP32 will use the current TDI value as the bootstrap value because the
  JTAG adapter overrides the pull-up or pull-down resistor that is supposed to do the
  bootstrapping. These lines basically set the idle value of the TDI line to a
  specified value, therefore reducing the chance of a bad bootup due to a bad flash
  voltage greatly. Default is 3.3.
