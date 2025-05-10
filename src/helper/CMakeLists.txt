# SPDX-License-Identifier: GPL-2.0-or-later

add_library(helper STATIC)

target_sources(helper PRIVATE
	binarybuffer.c
	options.c
	time_support_common.c
	configuration.c
	crc16.c
	crc32.c
	log.c
	time_support.c
	replacements.c
	fileio.c
	util.c
	jep106.c
	jim-nvp.c
	nvp.c
	align.h
	binarybuffer.h
	bits.h
	configuration.h
	list.h
	util.h
	types.h
	log.h
	command.h
	crc16.h
	crc32.h
	time_support.h
	replacements.h
	fileio.h
	system.h
	jep106.h
	jep106.inc
	jim-nvp.h
	nvp.h
	sha256.c
	sha256.h
	compiler.h
	base64.c
	base64.h
PUBLIC
	command.c
)

set_property(TARGET tcl_scripts APPEND PROPERTY STARTUP_TCL_SRCS ${CMAKE_CURRENT_LIST_DIR}/startup.tcl)

# EXTRA_DIST += \
# %D%/bin2char.sh \
# %D%/update_jep106.pl

