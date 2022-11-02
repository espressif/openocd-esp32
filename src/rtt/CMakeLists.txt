# SPDX-License-Identifier: GPL-2.0-or-later

add_library(rtt STATIC)

target_sources(rtt PRIVATE
	tcl.c
	rtt.c
	rtt.h
)

# circular dependency
target_link_libraries(rtt PRIVATE target)
