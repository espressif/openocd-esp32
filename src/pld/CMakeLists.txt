# SPDX-License-Identifier: GPL-2.0-or-later

add_library(pld STATIC)

target_sources(pld PRIVATE
	pld.c
	xilinx_bit.c
	virtex2.c
	pld.h
	xilinx_bit.h
	virtex2.h
)
