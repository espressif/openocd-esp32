# SPDX-License-Identifier: GPL-2.0-or-later

add_library(transport STATIC)

target_sources(transport PRIVATE
	transport.c
	transport.h
)
