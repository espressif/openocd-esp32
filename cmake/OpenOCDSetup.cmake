cmake_minimum_required (VERSION 3.16)

project(OpenOCD LANGUAGES C)

#TODO extract from configure.ac
set(OPENOCD_VERSION 0.12.0)
set(OPENOCD_BUGREPORT "OpenOCD Mailing List <openocd-devel@lists.sourceforge.net>")

set(host_cpu ${CMAKE_SYSTEM_PROCESSOR})
set(host_os ${CMAKE_HOST_SYSTEM_NAME})

if (NOT host)
	include(cmake/GetHostTriple.cmake)
	get_host_triple(host) # TODO test in windows
endif()

message("-- host triple variable is set to: " ${host})
message("-- host_cpu variable is set to: " ${host_cpu})
message("-- host_os variable is set to: " ${host_os})