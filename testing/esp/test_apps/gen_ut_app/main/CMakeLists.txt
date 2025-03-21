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
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c minor
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_MINOR)
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c bugfix
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_BUGFIX)
else()
	message("Skip getting IDF version. Use the latest as default.")
	set(UT_IDF_VER 0xFFFFFFFF)
	set(UT_IDF_VER_MAJOR 0xFF)
	set(UT_IDF_VER_MINOR 0xFF)
	set(UT_IDF_VER_BUGFIX 0xFF)
endif()

configure_file(ut_idf_ver.h.in ${idf_ver_header_path} @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

message("IDF version is ${UT_IDF_VER}")
if(CONFIG_ESP32_GCOV_ENABLE)
	target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
endif()

if(NOT CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME STREQUAL "")
	if(CONFIG_IDF_TARGET_ESP32)
		target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME_ESP32}")
	elseif(CONFIG_IDF_TARGET_ESP32S2)
		target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME_ESP32S2}")
	elseif(CONFIG_IDF_TARGET_ESP32S3)
		target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME_ESP32S3}")
	else()
		message(FATAL_ERROR "Custom linker not exist for this target. CMake will exit!")
	endif()
endif()

target_link_libraries(${COMPONENT_LIB} INTERFACE "-u include_xt_highint5")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func0")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func1")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func2")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func3")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func4")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func5")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func6")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u unused_func7")

