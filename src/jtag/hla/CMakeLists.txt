# SPDX-License-Identifier: GPL-2.0-or-later

add_library(hla STATIC)

target_sources(hla PRIVATE
	hla_transport.c
	hla_interface.c
	hla_layout.c
	hla_transport.h
	hla_interface.h
	hla_layout.h
)
