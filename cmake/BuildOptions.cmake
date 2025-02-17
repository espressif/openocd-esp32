# SPDX-License-Identifier: GPL-2.0-or-later

option(doxygen_as_html "Build Doxygen manual as HTML" OFF)
option(doxygen_as_pdf "Build Doxygen manual as PDF" OFF)
option(GCC_WARNINGS "Default compiler warnings" ON)
option(GCC_WEXTRA "Some extra warning flags that are not enabled by -Wall" ON)
option(GCC_WERROR "Make all warnings into errors" ON)
option(VERBOSE_JTAG_IO "Verbose JTAG I/O messages (for debugging)" OFF)
option(VERBOSE_USB_IO "Verbose USB I/O messages (for debugging)" OFF)
option(VERBOSE_USB_COMM "Verbose USB communication messages (for debugging)" OFF)
option(MALLOC_LOGGING "Include free space in logging messages (requires malloc.h)" OFF)
option(BUILD_DUMMY "Build the dummy port driver" ON)
option(BUILD_RSHIM "Build the rshim driver" OFF)
option(BUILD_DMEM "Build the dmem driver" OFF)
option(BUILD_PARPORT "Build the pc parallel port driver" OFF)
option(PARPORT_USE_PPDEV "Use of ppdev (/dev/parportN) for parport (for x86 only)" ON)
option(PARPORT_USE_GIVEIO "Use of giveio for parport (for CygWin only)" OFF)
option(BUILD_JTAG_VPI "Build support for JTAG VPI" OFF)
option(BUILD_VDEBUG "Build support for Cadence Virtual Debug Interface" OFF)
option(BUILD_JTAG_DPI "Build support for JTAG DPI" OFF)
option(BUILD_AMTJTAGACCEL "Build the Amontec JTAG-Accelerator driver" OFF)
option(BUILD_BCM2835GPIO "Build support for bitbanging on BCM2835 (as found in Raspberry Pi)" OFF)
option(BUILD_IMX_GPIO "Build support for bitbanging on NXP IMX processors" OFF)
option(BUILD_AM335XGPIO "Build support for bitbanging on AM335x (as found in Beaglebones)" OFF)
option(BUILD_EP93XX "Build support for EP93xx based SBCs" OFF)
option(BUILD_AT91RM9200 "Build support for AT91RM9200 based SBCs" OFF)
option(BUILD_GW16012 "Build support for the Gateworks GW16012 JTAG Programmer" OFF)
option(BUILD_SYSFSGPIO "Build support for programming driven via sysfs gpios" OFF)
option(BUILD_LINUXSPIDEV "Build support for Linux SPI device SWD adapter" OFF)
option(BUILD_XLNX_PCIE_XVC "Build support for Xilinx XVC/PCIe" OFF)
option(use_internal_jimtcl "Build internal jimtcl" OFF)
option(use_internal_jimtcl_maintainer "Maintainer mode when building internal jimtcl" OFF)
option(use_internal_libjaylink "Build internal libjaylink" OFF)
option(BUILD_REMOTE_BITBANG "Build support for the Remote Bitbang driver" ON)

# Espressif additions
option(BUILD_ESP_REMOTE "Build support for the ESP remote protocol over TCP or USB" ON)
option(BUILD_ESP_COMPRESSION "Build support for the ESP flasher image compression" ON)
option(USE_GCOV "Build support for the coverage" OFF)
option(BUILD_SANITIZERS "Build support with the sanitizer flags" OFF)

# USB1 Adapters #TODO check default values
option(BUILD_FTDI "MPSSE mode of FTDI based devices" ON)
option(BUILD_HLADAPTER_STLINK "ST-Link Programmer" ON)
option(BUILD_HLADAPTER_ICDI "TI ICDI JTAG Programmer" ON)
option(BUILD_ULINK "Keil ULINK JTAG Programmer" ON)
option(BUILD_ANGIE "ANGIE USB-JTAG adapter" OFF)
option(BUILD_USB_BLASTER_2 "Altera USB-Blaster II Compatible" ON)
option(BUILD_FT232R "Bitbang mode of FT232R based devices" ON)
option(BUILD_VSLLINK "Versaloon-Link JTAG Programmer" ON)
option(BUILD_XDS110 "TI XDS110 Debug Probe" ON)
option(BUILD_CMSIS_DAP_USB "CMSIS-DAP v2 Compliant Debugger" ON)
option(BUILD_OSBDM "OSBDM (JTAG only) Programmer" ON)
option(BUILD_OPENDOUS "eStick/opendous JTAG Programmer" ON)
option(BUILD_ARMJTAGEW "Olimex ARM-JTAG-EW Programmer" ON)
option(BUILD_RLINK "Raisonance RLink JTAG Programmer" ON)
option(BUILD_USBPROG "USBProg JTAG Programmer" ON)
option(BUILD_ESP_USB_JTAG "Espressif JTAG Programmer" ON)

# Deprecated USB1 Adapters
option(BUILD_AICE "Andes JTAG Programmer" OFF)

# Hidapi Adapters
option(BUILD_CMSIS_DAP_HID "CMSIS-DAP Compliant Debugger" ON)
option(BUILD_HLADAPTER_NULINK "Nu-Link Programmer" OFF)

# Hidapi USB1 Adapters
option(BUILD_KITPROG "Cypress KitProg Programmer" ON)

# LibFTDI Adapters
option(BUILD_USB_BLASTER "Altera USB-Blaster Compatible" OFF)
option(BUILD_PRESTO "ASIX Presto Adapter" OFF)

# LibFTDI USB1 Adapters
option(BUILD_OPENJTAG "OpenJTAG Adapter" ON)

# LibGPIOD Adapters
option(BUILD_LINUXGPIOD "Linux GPIO bitbang through libgpiod" OFF)

# LibJAYLINK Adapters
option(BUILD_JLINK "SEGGER J-Link Programmer" ON)

# PCIE Adapters
option(BUILD_XLNX_PCIE_XVC "Xilinx XVC/PCIe" ON)

# Serial Port Adapters
option(BUILD_BUS_PIRATE "Bus Pirate" ON)

# Optional Libraries
option(ENABLE_CAPSTONE "Build support for the Capstone disassembly framework" ON)
