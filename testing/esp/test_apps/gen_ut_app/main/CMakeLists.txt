file(GLOB GEN_UT_SRCS "*.c")
idf_component_register(SRCS ${GEN_UT_SRCS}
                    INCLUDE_DIRS ".")

if(CONFIG_ESP32_GCOV_ENABLE)
target_compile_options(${COMPONENT_LIB} PRIVATE --coverage)
endif()

if(NOT CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME STREQUAL "")
target_linker_script(${COMPONENT_LIB} INTERFACE "${CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME}")
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
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c major
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_MAJOR)
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c minor
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_MINOR)
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c bugfix
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_BUGFIX)
	execute_process(COMMAND ${idf_ver_script} $ENV{IDF_PATH} -c update
					OUTPUT_STRIP_TRAILING_WHITESPACE
					OUTPUT_VARIABLE UT_IDF_VER_UPDATE)
else()
	message("Skip getting IDF version. Use the latest as default.")
	set(UT_IDF_VER_MAJOR 0xFF)
	set(UT_IDF_VER_MINOR 0xFF)
	set(UT_IDF_VER_BUGFIX 0xFF)
	set(UT_IDF_VER_UPDATE 0xFF)
endif()

configure_file(ut_idf_ver.h.in ${idf_ver_header_path} @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
