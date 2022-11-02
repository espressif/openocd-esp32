cmake_minimum_required (VERSION 3.16)

project(OpenOCD LANGUAGES C)

include(cmake/OpenOCDSetup.cmake)
include(cmake/BuildOptions.cmake)
include(cmake/configure.cmake)

add_executable(openocd)

include(cmake/jimtcl.cmake)
add_subdirectory(src)

# Propogate config.h which is auto-generated into the root directory.
target_include_directories(openocd PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(openocd PRIVATE openocd_main libjimtcl)

#TODO test if distro package works well.
install(TARGETS openocd DESTINATION bin)
install(DIRECTORY tcl/ DESTINATION share/openocd/scripts)
install(DIRECTORY contrib DESTINATION share/openocd)
install(FILES jtag/drivers/OpenULINK/ulink_firmware.hex DESTINATION share/openocd/OpenULINK)

#TODO can be added to the ADDITIONAL_CLEAN_FILES properties
if(BUILD_GCOV)
	add_custom_target(gcov-clean
		COMMAND ${CMAKE_MAKE_PROGRAM} clean
		COMMAND find . -name "*.gcno" -type f -delete
		COMMAND find . -name "*.gcda" -type f -delete
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)
endif()
