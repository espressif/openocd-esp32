# SPDX-License-Identifier: GPL-2.0-or-later

add_library(jtag STATIC)

# Propogate src/jtag/ directory to all subdirs
target_include_directories(jtag PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_sources(jtag PRIVATE
	adapter.c
	adapter.h
	commands.c
	core.c
	interface.c
	interfaces.c
	tcl.c
	swim.c
	commands.h
	interface.h
	interfaces.h
	minidriver.h
	jtag.h
	swd.h
	swim.h
	tcl.h
)

# sub targets of openocd-jtag library
if(BUILD_HLADAPTER)
	list(APPEND jtag_targets hla)
endif()

if(BUILD_AICE)
	list(APPEND jtag_targets aice)
endif()

foreach(target ${jtag_targets})
	add_subdirectory(${target})
	target_compile_options(${target} PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(${target} PRIVATE $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
endforeach()

target_link_libraries(jtag PRIVATE ${jtag_targets} server)

add_subdirectory(drivers)

target_link_libraries(jtag PRIVATE ocdjtagdrivers)

set_property(TARGET tcl_scripts APPEND PROPERTY STARTUP_TCL_SRCS ${CMAKE_CURRENT_LIST_DIR}/startup.tcl)
