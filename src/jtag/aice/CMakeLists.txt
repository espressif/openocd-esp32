# SPDX-License-Identifier: GPL-2.0-or-later

add_library(aice STATIC)

target_sources(aice PRIVATE
	aice_transport.c
	aice_interface.c
	aice_port.c
	aice_usb.c
	aice_pipe.c
	aice_transport.h
	aice_interface.h
	aice_port.h
	aice_usb.h
	aice_pipe.h
)

target_compile_options(aice PRIVATE ${LIBUSB1_CFLAGS})
