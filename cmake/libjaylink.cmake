include(ExternalProject)

set(LIBJLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libjaylink)
set(LIBJLINK_BIN ${CMAKE_CURRENT_BINARY_DIR}/libjaylink)
set(LIBJLINK_STATIC_LIB ${LIBJLINK_BIN}/lib/libjaylink.a)
set(LIBJLINK_INCLUDES ${LIBJLINK_BIN}/include)

file(MAKE_DIRECTORY ${LIBJLINK_INCLUDES})

ExternalProject_Add(
    libjaylink_project
    PREFIX ${LIBJLINK_BIN}
    SOURCE_DIR ${LIBJLINK_DIR}
    CONFIGURE_COMMAND ${LIBJLINK_DIR}/autogen.sh && ${LIBJLINK_DIR}/configure --prefix=${LIBJLINK_BIN} --disable-shared
    BUILD_COMMAND make
    BUILD_IN_SOURCE 1
    USES_TERMINAL_BUILD 1
    INSTALL_COMMAND make install
    BUILD_BYPRODUCTS ${LIBJLINK_STATIC_LIB}
)

add_library(libjaylink STATIC IMPORTED GLOBAL)

add_dependencies(libjaylink libjaylink_project)

set_target_properties(libjaylink PROPERTIES IMPORTED_LOCATION ${LIBJLINK_STATIC_LIB})
set_target_properties(libjaylink PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${LIBJLINK_INCLUDES})
