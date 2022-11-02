# SPDX-License-Identifier: GPL-2.0-or-later

add_library(server STATIC)

target_sources(server PRIVATE
	server.c
	telnet_server.c
	gdb_server.c
	server.h
	telnet_server.h
	gdb_server.h
	tcl_server.c
	tcl_server.h
	rtt_server.c
	rtt_server.h
	ipdbg.c
	ipdbg.h
)

set_property(TARGET tcl_scripts APPEND PROPERTY STARTUP_TCL_SRCS ${CMAKE_CURRENT_LIST_DIR}/startup.tcl)
