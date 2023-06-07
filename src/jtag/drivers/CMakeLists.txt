# SPDX-License-Identifier: GPL-2.0-or-later

add_library(ocdjtagdrivers)

target_sources(ocdjtagdrivers PRIVATE driver.c)

if(use_libusb1)
    target_sources(ocdjtagdrivers PRIVATE libusb_helper.c)
    target_compile_options(ocdjtagdrivers PRIVATE ${LIBUSB1_CFLAGS})
    target_link_libraries(ocdjtagdrivers PRIVATE ${LIBUSB1_LINK_LIBRARIES})
endif()

if(use_libftdi)
    target_compile_options(ocdjtagdrivers PRIVATE ${LIBFTDI_CFLAGS})
    target_link_libraries(ocdjtagdrivers PRIVATE ${LIBFTDI_LINK_LIBRARIES})
endif()

if(use_libgpiod)
    target_compile_options(ocdjtagdrivers PRIVATE ${LIBGPIOD_CFLAGS})
    target_link_libraries(ocdjtagdrivers PRIVATE ${LIBGPIOD_LINK_LIBRARIES})
endif()

if(use_hidapi)
    target_compile_options(ocdjtagdrivers PRIVATE ${HIDAPI_CFLAGS})
    target_link_libraries(ocdjtagdrivers PRIVATE ${HIDAPI_LINK_LIBRARIES})
endif()

if(BUILD_JLINK)
    add_library(ocdjlink)
    target_sources(ocdjlink PRIVATE jlink.c)
    target_compile_options(ocdjlink PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
	target_include_directories(ocdjlink PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(ocdjlink PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
    if(use_internal_libjaylink)
        include(${CMAKE_SOURCE_DIR}/cmake/libjaylink.cmake)
        if(HAVE_LIBUSB1)
            target_compile_definitions(ocdjlink PRIVATE HAVE_LIBUSB)
        endif()
        target_link_libraries(ocdjlink PRIVATE libjaylink)
    else()
        target_compile_options(ocdjlink PRIVATE ${LIBJAYLINK_CFLAGS})
        target_link_libraries(ocdjlink PRIVATE ${LIBJAYLINK_LINK_LIBRARIES})
    endif()
    target_link_libraries(ocdjtagdrivers PRIVATE ocdjlink)
endif()

if(BUILD_BITBANG)
    target_sources(ocdjtagdrivers PRIVATE bitbang.c)
endif()

if(BUILD_PARPORT)
    target_sources(ocdjtagdrivers PRIVATE parport.c)
endif()

if(BUILD_DUMMY)
    target_sources(ocdjtagdrivers PRIVATE dummy.c)
endif()

if(BUILD_FTDI)
    target_sources(ocdjtagdrivers PRIVATE ftdi.c mpsse.c)
endif()

if(BUILD_LINUXGPIOD)
    target_sources(ocdjtagdrivers PRIVATE linuxgpiod.c)
endif()

if(BUILD_JTAG_VPI)
    target_sources(ocdjtagdrivers PRIVATE jtag_vpi.c)
endif()

if(BUILD_VDEBUG)
    target_sources(ocdjtagdrivers PRIVATE vdebug.c)
endif()

if(BUILD_JTAG_DPI)
    target_sources(ocdjtagdrivers PRIVATE jtag_dpi.c)
endif()

if(BUILD_USB_BLASTER OR BUILD_USB_BLASTER_2)
    add_subdirectory(usb_blaster)
    target_compile_options(ocdusbblaster PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
    target_include_directories(ocdusbblaster PRIVATE ${CMAKE_CURRENT_LIST_DIR})
	target_include_directories(ocdusbblaster PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(ocdusbblaster PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
    target_link_libraries(ocdjtagdrivers PRIVATE ocdusbblaster)
endif()

if(BUILD_FT232R)
    target_sources(ocdjtagdrivers PRIVATE ft232r.c)
endif()

if(BUILD_AMTJTAGACCEL)
    target_sources(ocdjtagdrivers PRIVATE amt_jtagaccel.c)
endif()

if(BUILD_EP93XX)
    target_sources(ocdjtagdrivers PRIVATE ep93xx.c)
endif()

if(BUILD_AT91RM9200)
    target_sources(ocdjtagdrivers PRIVATE at91rm9200.c)
endif()

if(BUILD_GW16012)
    target_sources(ocdjtagdrivers PRIVATE gw16012.c)
endif()

if(BUILD_BITQ)
    target_sources(ocdjtagdrivers PRIVATE bitq.c)
endif()

if(BUILD_PRESTO)
    target_sources(ocdjtagdrivers PRIVATE presto.c)
endif()

if(BUILD_ESP_USB_JTAG)
    target_sources(ocdjtagdrivers PRIVATE esp_usb_jtag.c)
endif()

if(BUILD_USBPROG)
    target_sources(ocdjtagdrivers PRIVATE usbprog.c)
endif()

if(BUILD_RLINK)
    target_sources(ocdjtagdrivers PRIVATE rlink.c rlink_speed_table.c)
endif()

if(BUILD_ULINK)
    target_sources(ocdjtagdrivers PRIVATE ulink.c)
    target_link_libraries(ocdjtagdrivers PRIVATE m)
    set_property(TARGET ocdjtagdrivers PROPERTY ULINK_FIRMWARE ${CMAKE_CURRENT_LIST_DIR}/OpenULINK/ulink_firmware.hex)
    #set(ULINK_FIRMWARE ${CMAKE_CURRENT_LIST_DIR}/OpenULINK/ulink_firmware.hex PARENT_SCOPE)
endif()

if(BUILD_VSLLINK)
    target_sources(ocdjtagdrivers PRIVATE 
        versaloon/usbtoxxx/usbtogpio.c
        versaloon/usbtoxxx/usbtojtagraw.c
        versaloon/usbtoxxx/usbtoswd.c
        versaloon/usbtoxxx/usbtopwr.c
        versaloon/usbtoxxx/usbtoxxx.c
        versaloon/versaloon.c
        vsllink.c
    )
endif()

if(BUILD_ARMJTAGEW)
    target_sources(ocdjtagdrivers PRIVATE arm-jtag-ew.c)
endif()

if(BUILD_BUS_PIRATE)
    target_sources(ocdjtagdrivers PRIVATE buspirate.c)
endif()

if(BUILD_REMOTE_BITBANG)
    target_sources(ocdjtagdrivers PRIVATE remote_bitbang.c)
endif()

if(BUILD_HLADAPTER_STLINK)
    target_sources(ocdjtagdrivers PRIVATE stlink_usb.c)
endif()

if(BUILD_HLADAPTER_ICDI)
    target_sources(ocdjtagdrivers PRIVATE ti_icdi_usb.c)
endif()

if(BUILD_HLADAPTER_NULINK)
    target_sources(ocdjtagdrivers PRIVATE nulink_usb.c)
endif()

if(BUILD_RSHIM)
    target_sources(ocdjtagdrivers PRIVATE rshim.c)
endif()

if(BUILD_OSBDM)
    target_sources(ocdjtagdrivers PRIVATE osbdm.c)
endif()

if(BUILD_OPENDOUS)
    target_sources(ocdjtagdrivers PRIVATE opendous.c)
endif()

if(BUILD_SYSFSGPIO)
    target_sources(ocdjtagdrivers PRIVATE sysfsgpio.c)
endif()

if(BUILD_XLNX_PCIE_XVC)
    target_sources(ocdjtagdrivers PRIVATE xlnx-pcie-xvc.c)
endif()

if(BUILD_BCM2835GPIO)
    target_sources(ocdjtagdrivers PRIVATE bcm2835gpio.c)
endif()

if(BUILD_OPENJTAG)
    target_sources(ocdjtagdrivers PRIVATE openjtag.c)
endif()

if(BUILD_CMSIS_DAP_HID)
    target_sources(ocdjtagdrivers PRIVATE cmsis_dap_usb_hid.c cmsis_dap.c)
endif()

if(BUILD_CMSIS_DAP_USB)
    target_sources(ocdjtagdrivers PRIVATE cmsis_dap_usb_bulk.c)
    if(NOT BUILD_CMSIS_DAP_HID)
        target_sources(ocdjtagdrivers PRIVATE cmsis_dap.c)
    endif()
endif()

if(BUILD_IMX_GPIO)
    target_sources(ocdjtagdrivers PRIVATE imx_gpio.c)
endif()

if(BUILD_KITPROG)
    target_sources(ocdjtagdrivers PRIVATE kitprog.c)
endif()

if(BUILD_XDS110)
    target_sources(ocdjtagdrivers PRIVATE xds110.c)
endif()

if(BUILD_AM335XGPIO)
    target_sources(ocdjtagdrivers PRIVATE am335xgpio.c)
endif()

if(BUILD_ESP_REMOTE)
    target_sources(ocdjtagdrivers PRIVATE jtag_esp_remote.c)
endif()

if(IS_ESPIDF)
    target_sources(ocdjtagdrivers PRIVATE esp_gpio.c)
    target_link_libraries(ocdjtagdrivers PRIVATE idf::driver)
endif()

target_sources(ocdjtagdrivers PRIVATE
    bitbang.h
    bitq.h
    libftdi_helper.h
    libusb_helper.h
    cmsis_dap.h
    minidriver_imp.h
    mpsse.h
    rlink.h
    rlink_dtc_cmd.h
    rlink_ep1_cmd.h
    rlink_st7.h
    versaloon/usbtoxxx/usbtoxxx.h
    versaloon/usbtoxxx/usbtoxxx_internal.h
    versaloon/versaloon.h
    versaloon/versaloon_include.h
    versaloon/versaloon_internal.h
)

# TODO try to include for all subtargets in one time..
target_compile_options(ocdjtagdrivers PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
target_include_directories(ocdjtagdrivers PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(ocdjtagdrivers PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
