# SPDX-License-Identifier: GPL-2.0-or-later

add_library(ocdusbblaster STATIC)

target_sources(ocdusbblaster PRIVATE usb_blaster.c ublast_access.h)

if(BUILD_USB_BLASTER)
	target_sources(ocdusbblaster PRIVATE ublast_access_ftdi.c)
endif()

if(BUILD_USB_BLASTER_2)
	target_sources(ocdusbblaster PRIVATE ublast2_access_libusb.c)
endif()

target_compile_options(ocdusbblaster PRIVATE ${LIBUSB1_CFLAGS} ${LIBFTDI_CFLAGS})
