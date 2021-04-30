# we can not detect IDF version until ${CMAKE_CURRENT_SOURCE_DIR} is properly set by
# 'register_component' or 'idf_component_register', so just check for old-fashioned 'register_component' here
if(COMMAND register_component)
	file(GLOB GEN_UT_SRCS "*.c" "*.S")
	set(COMPONENT_SRCS ${GEN_UT_SRCS})
	set(COMPONENT_ADD_INCLUDEDIRS ".")
	register_component()
else()
	idf_component_register(SRCS ${GEN_UT_SRCS}
						INCLUDE_DIRS ".")
endif()

file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}/ut_idf_ver.h" idf_ver_header_path)
file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/get_idf_ver.py" idf_ver_script)

set(get_idf_ver NO)

if(DEFINED ENV{UT_GET_IDF_VER})
	if(NOT $ENV{UT_GET_IDF_VER} STREQUAL "0")
		set(get_idf_ver YES)
	endif()
endif()

if(get_idf_ver)
	message("Get IDF version from repo $ENV{IDF_PATH}.")
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c pack2hex
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER)
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c major
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_MAJOR)
else()
	message("Skip getting IDF version. Use the latest as default.")
	set(UT_IDF_VER 0xFFFFFFFF)
endif()

configure_file(ut_idf_ver.h.in ${idf_ver_header_path} @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

message("IDF version is ${UT_IDF_VER}")
if(UT_IDF_VER_MAJOR STREQUAL "3")
	if(CONFIG_ESP32_GCOV_ENABLE)
		target_compile_options(${COMPONENT_TARGET} PRIVATE --coverage)
	endif()

	if(NOT CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME STREQUAL "")
		target_linker_script(${COMPONENT_TARGET} "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME}")
	endif()

	target_link_libraries(${COMPONENT_TARGET} "-u include_xt_highint5")
else()
	if(CONFIG_ESP32_GCOV_ENABLE)
		target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
	endif()

	if(NOT CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME STREQUAL "")
		if(CONFIG_IDF_TARGET_ESP32)
			target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME_ESP32}")
		elseif(CONFIG_IDF_TARGET_ESP32S2)
			target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME_ESP32S2}")
		else()
			message(FATAL_ERROR "Custom linker not exist for this target. CMake will exit!")
		endif()
	endif()

	target_link_libraries(${COMPONENT_LIB} INTERFACE "-u include_xt_highint5")
endif()
