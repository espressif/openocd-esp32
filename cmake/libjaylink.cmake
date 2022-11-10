include(ExternalProject)

set(LIBJLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libjaylink)
set(LIBJLINK_BIN ${CMAKE_CURRENT_BINARY_DIR}/libjaylink)
set(LIBJLINK_STATIC_LIB ${LIBJLINK_BIN}/lib/libjaylink.a)
set(LIBJLINK_INCLUDES ${LIBJLINK_BIN}/include)

file(MAKE_DIRECTORY ${LIBJLINK_INCLUDES})

if(${CMAKE_CROSSCOMPILING})
set(CONF_HOST --host=${host})
endif()

ExternalProject_Add(
    libjaylink_project
    PREFIX ${LIBJLINK_BIN}
    SOURCE_DIR ${LIBJLINK_DIR}
    CONFIGURE_COMMAND ${LIBJLINK_DIR}/autogen.sh && ${LIBJLINK_DIR}/configure ${CONF_HOST} --prefix=${LIBJLINK_BIN} --disable-shared
    BUILD_COMMAND make
    BUILD_IN_SOURCE 1
    # These two options are set so that Ninja immediately outputs
    # the subproject build to the terminal. Otherwise it looks like the
    # build process "hangs" for too long until jimtcl build is complete.
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
    BUILD_BYPRODUCTS ${LIBJLINK_STATIC_LIB}
)

add_library(libjaylink STATIC IMPORTED GLOBAL)

add_dependencies(libjaylink libjaylink_project)

set_target_properties(libjaylink PROPERTIES IMPORTED_LOCATION ${LIBJLINK_STATIC_LIB})
set_target_properties(libjaylink PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBJLINK_INCLUDES})
