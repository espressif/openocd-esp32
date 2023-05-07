# SPDX-License-Identifier: GPL-2.0-or-later

add_library(pld STATIC)

target_sources(pld PRIVATE
	certus.c
	ecp2_3.c
	ecp5.c
	efinix.c
	gatemate.c
	gowin.c
	intel.c
	lattice.c
	lattice_bit.c
	pld.c
	raw_bit.c
	xilinx_bit.c
	virtex2.c
	certus.h
	ecp2_3.h
	ecp5.h
	lattice.h
	lattice_bit.h
	lattice_cmd.h
	pld.h
	raw_bit.h
	xilinx_bit.h
	virtex2.h
)
