include(ExternalProject)

set(JIMTCL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/jimtcl)
set(JIMTCL_BIN ${CMAKE_CURRENT_BINARY_DIR}/jimtcl)
set(JIMTCL_STATIC_LIB ${JIMTCL_DIR}/libjim.a)
set(JIMTCL_INCLUDES ${JIMTCL_DIR})

if(${CMAKE_CROSSCOMPILING})
set(CONF_HOST --host=${host})
endif()

#TODO get config options from the configure.cmake step.
ExternalProject_Add(
    jimtcl_project
    PREFIX ${JIMTCL_BIN}
    SOURCE_DIR ${JIMTCL_DIR}
    CONFIGURE_COMMAND ${JIMTCL_DIR}/configure ${CONF_HOST} --disable-shared --disable-install-jim --with-ext=json --minimal --disable-ssl
    BUILD_COMMAND make
    BUILD_IN_SOURCE 1
    # These two options are set so that Ninja immediately outputs
    # the subproject build to the terminal. Otherwise it looks like the
    # build process "hangs" for too long until jimtcl build is complete.
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
    BUILD_BYPRODUCTS ${JIMTCL_STATIC_LIB}
)

add_library(libjimtcl STATIC IMPORTED GLOBAL)

add_dependencies(libjimtcl jimtcl_project)

set_target_properties(libjimtcl PROPERTIES IMPORTED_LOCATION ${JIMTCL_STATIC_LIB})
set_target_properties(libjimtcl PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${JIMTCL_INCLUDES})

# Make clean will only remove the file added in the BUILD_BYPRODUCTS option.
#TODO set ADDITIONAL_CLEAN_FILES for *.o *.so *.dll .exe lib*.a jimsh Tcl.html _*.c
