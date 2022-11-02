# SPDX-License-Identifier: GPL-2.0-or-later

add_library(openrisc STATIC)

target_sources(openrisc PRIVATE
	or1k.c
	or1k_du_adv.c
	or1k_tap_mohor.c
	or1k_tap_vjtag.c
	or1k_tap_xilinx_bscan.c
	jsp_server.c
	or1k.h
	or1k_du.h
	or1k_tap.h
	jsp_server.h
)
