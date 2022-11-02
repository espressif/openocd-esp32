# SPDX-License-Identifier: GPL-2.0-or-later

add_library(openocdflash STATIC)

target_sources(openocdflash PRIVATE
	common.c
	common.h
)

# sub targets of flash library
set(flash_targets nand nor)

foreach(target ${flash_targets})
	add_subdirectory(${target})
	target_compile_options(${target} PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
endforeach()

target_link_libraries(openocdflash PRIVATE ${flash_targets})

set_property(TARGET tcl_scripts APPEND PROPERTY STARTUP_TCL_SRCS ${CMAKE_CURRENT_LIST_DIR}/startup.tcl)
