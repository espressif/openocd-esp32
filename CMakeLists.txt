cmake_minimum_required (VERSION 3.16)

project(OpenOCD LANGUAGES C)

include(cmake/OpenOCDSetup.cmake)
include(cmake/BuildOptions.cmake)
include(cmake/configure.cmake)

add_executable(openocd)

if(use_internal_jimtcl)
	include(cmake/jimtcl.cmake)
elseif(JIMTCL_FOUND)
	add_library(libjimtcl INTERFACE)
	target_include_directories(libjimtcl INTERFACE ${JIMTCL_INCLUDE_DIRS})
	target_link_libraries(libjimtcl INTERFACE ${JIMTCL_LINK_LIBRARIES})
elseif(have_jim_h)
	add_library(libjimtcl INTERFACE)
	target_link_libraries(libjimtcl INTERFACE jim)
endif()

add_subdirectory(src)

# Propogate config.h which is auto-generated into the root directory.
target_include_directories(openocd PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(openocd PRIVATE openocd_main libjimtcl)

install(TARGETS openocd DESTINATION ${bindir})
install(DIRECTORY tcl/ DESTINATION ${pkgdatadir}/scripts)
install(FILES contrib/60-openocd.rules DESTINATION ${pkgdatadir}/contrib)
install(DIRECTORY contrib/libdcc DESTINATION ${pkgdatadir}/contrib)
install(FILES tools/esp_detect_config.py DESTINATION ${pkgdatadir}/espressif/tools)
install(FILES tools/esp_problems_hints.yml DESTINATION ${pkgdatadir}/espressif/tools)
install(FILES contrib/loaders/flash/espressif/esp32c6/inc/stub_flash_idf_binary_code.inc DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32c6)
install(FILES contrib/loaders/flash/espressif/esp32c6/inc/stub_flash_idf_binary_data.inc DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32c6)
install(FILES contrib/loaders/flash/espressif/esp32c6/stub_flash_idf_image.h DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32c6)
install(FILES contrib/loaders/flash/espressif/esp32h2/inc/stub_flash_idf_binary_code.inc DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32h2)
install(FILES contrib/loaders/flash/espressif/esp32h2/inc/stub_flash_idf_binary_data.inc DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32h2)
install(FILES contrib/loaders/flash/espressif/esp32h2/stub_flash_idf_image.h DESTINATION ${pkgdatadir}/espressif/stub_bins/esp32h2)
get_property(ulink_firmware TARGET ocdjtagdrivers PROPERTY ULINK_FIRMWARE)
if(BUILD_JLINK)
	install(FILES ${ulink_firmware} DESTINATION ${pkgdatadir}/OpenULINK)
endif()

#TODO can be added to the ADDITIONAL_CLEAN_FILES properties
if(USE_GCOV)
	add_custom_target(gcov-clean
		COMMAND ${CMAKE_MAKE_PROGRAM} clean
		COMMAND find . -name "*.gcno" -type f -delete
		COMMAND find . -name "*.gcda" -type f -delete
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	)
endif()
