include(ExternalProject)

set(JIMTCL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jimtcl)
set(JIMTCL_BIN ${CMAKE_CURRENT_BINARY_DIR}/jimtcl)
set(JIMTCL_STATIC_LIB ${JIMTCL_BIN}/lib/libjim.a)
set(JIMTCL_INCLUDES ${JIMTCL_BIN}/include)

file(MAKE_DIRECTORY ${JIMTCL_INCLUDES})

ExternalProject_Add(
    jimtcl_project
    PREFIX ${JIMTCL_BIN}
    SOURCE_DIR ${JIMTCL_DIR}
    CONFIGURE_COMMAND ${JIMTCL_DIR}/configure --prefix=${JIMTCL_BIN} --disable-shared
    BUILD_COMMAND make
    BUILD_IN_SOURCE 1
    USES_TERMINAL_BUILD 1
    INSTALL_COMMAND make install
    BUILD_BYPRODUCTS ${JIMTCL_STATIC_LIB}
)

add_library(libjimtcl STATIC IMPORTED GLOBAL)

add_dependencies(libjimtcl jimtcl_project)

set_target_properties(libjimtcl PROPERTIES IMPORTED_LOCATION ${JIMTCL_STATIC_LIB})
set_target_properties(libjimtcl PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${JIMTCL_INCLUDES})
