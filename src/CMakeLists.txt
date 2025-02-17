# SPDX-License-Identifier: GPL-2.0-or-later

add_library(openocd_main STATIC)

# Propogate src/ directory to all subdirs
target_include_directories(openocd_main PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_sources(openocd_main PRIVATE
	openocd.c
	main.c
	${CMAKE_BINARY_DIR}/startup_tcl.inc
PUBLIC
	hello.c
)

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d-%H:%M")
target_compile_definitions(openocd_main PRIVATE 
	PKGBLDDATE=\"${BUILD_TIMESTAMP}\"
	RELSTR=""
	GITVERSION=""
	LIBUSB1_COMMIT=\"${LIBUSB1_COMMIT_HASH}\"
)

target_compile_options(openocd_main PRIVATE ${OPENOCD_COMMON_COMPILER_FLAGS})
# For the "startup_tcl.inc"
target_include_directories(openocd_main PRIVATE ${CMAKE_BINARY_DIR})
if (IS_ESPIDF)
	target_include_directories(openocd_main BEFORE PRIVATE ${PLATFORM_INCLUDE_DIR})
endif()

add_custom_target(tcl_scripts ALL)

add_subdirectory(rtt)
add_subdirectory(helper)
add_subdirectory(rtos)
add_subdirectory(server)
add_subdirectory(transport)
add_subdirectory(target)
add_subdirectory(jtag)
add_subdirectory(flash)
add_subdirectory(pld)
add_subdirectory(svf)
add_subdirectory(xsvf)

# directory and library names can be different
list(APPEND openocd_targets
	rtt
	helper
	rtos
	server
	transport
	target
	jtag
	openocdflash
	pld
	svf
	xsvf
)

foreach(target ${openocd_targets})
	target_compile_options(${target} PUBLIC ${OPENOCD_COMMON_COMPILER_FLAGS})
	target_include_directories(${target} BEFORE PUBLIC $<TARGET_PROPERTY:libjimtcl,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(${target} BEFORE PUBLIC $<TARGET_PROPERTY:openocd,INTERFACE_INCLUDE_DIRECTORIES>)
	target_include_directories(${target} BEFORE PUBLIC $<TARGET_PROPERTY:openocd_main,INTERFACE_INCLUDE_DIRECTORIES>)
endforeach()

if (HAVE_UTIL)
	list(APPEND openocd_targets util)
endif()

if (HAVE_DLOPEN)
	list(APPEND openocd_targets  dl)
endif()

if(IS_MINGW)
	list(APPEND openocd_targets ws2_32)
endif()

if(BUILD_ESP_COMPRESSION)
	list(APPEND openocd_targets z)
endif()

target_link_libraries(openocd_main PUBLIC ${openocd_targets})

# Custom commands & targets
get_property(startup_tcl_sources TARGET tcl_scripts PROPERTY STARTUP_TCL_SRCS)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/startup_tcl.inc
	COMMAND cat ${startup_tcl_sources} |
    ${CMAKE_CURRENT_LIST_DIR}/helper/bin2char.sh > ${CMAKE_BINARY_DIR}/startup_tcl.inc || rm -f ${CMAKE_BINARY_DIR}/startup_tcl.inc)
