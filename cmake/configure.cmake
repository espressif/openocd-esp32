include(FindPkgConfig)
include(CheckIncludeFiles)
include(CheckSymbolExists)
include(CheckLibraryExists)
include(CheckTypeSize)
include("${CMAKE_CURRENT_LIST_DIR}/CheckTypeExists.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/CheckStruct.cmake")

set(IS_ESPIDF 0)
if(${host} MATCHES "(-cygwin*)")
    set(IS_WIN32 1)
    set(IS_CYGWIN 1)
elseif(${host} MATCHES "(-mingw*)")
    set(IS_WIN32 1)
    set(IS_MINGW 1)
    set(HOST_CPPFLAGS -D__USE_MINGW_ANSI_STDIO)
elseif(${host} MATCHES "darwin*")
    set(IS_DARWIN 1)
endif()

if(NOT ${host_os} MATCHES "Windows")
    set(pkgdatadir "/usr/local/share/openocd")
    set(bindir "/usr/local/bin")
else()
    # TODO
endif()

if (CMAKE_C_BYTE_ORDER MATCHES "BIG_ENDIAN")
    set(WORDS_BIGENDIAN 1)
endif()

# TODO Search 'environ' in unistd.h or available libraries
set(NEED_ENVIRON_EXTERN 1)

# Search Libs #
# TODO Does cmake linking the libraries to check the function?
check_library_exists(ioperm ioperm "" HAVE_IOPERM)
check_library_exists(dl dlopen "" HAVE_DLOPEN)
check_library_exists(util openpty "" HAVE_UTIL)

# TODO check_library_exists() seems not working as expected
if(IS_MINGW)
    set(HAVE_IOPERM 0)
    set(HAVE_UTIL 0)
    set(HAVE_DLOPEN 0)
endif()

# Check Headers #
check_include_files(limits.h STDC_HEADERS)
check_include_files(stdbool.h HAVE_STDBOOL_H)
check_include_files(stdint.h HAVE_STDINT_H)
check_include_files(stdlib.h HAVE_STDLIB_H)
check_include_files(strings.h HAVE_STRINGS_H)
check_include_files(string.h HAVE_STRING_H)
check_include_files(memory.h HAVE_MEMORY_H)
check_include_files(inttypes.h HAVE_INTTYPES_H)
check_include_files(sys/socket.h HAVE_SYS_SOCKET_H)
check_include_files(elf.h HAVE_ELF_H)
check_struct("Elf64_Ehdr" "elf.h" HAVE_ELF64)
check_include_files(dirent.h HAVE_DIRENT_H)
check_include_files (dlfcn.h HAVE_DLFCN_H)
check_include_files(fcntl.h HAVE_FCNTL_H)
check_include_files(malloc.h HAVE_MALLOC_H)
check_include_files(netdb.h HAVE_NETDB_H)
check_include_files(poll.h HAVE_POLL_H)
check_include_files(sys/ioctl.h HAVE_SYS_IOCTL_H)
check_include_files(sys/param.h HAVE_SYS_PARAM_H)
check_include_files(sys/select.h HAVE_SYS_SELECT_H)
check_include_files(sys/stat.h HAVE_SYS_STAT_H)
check_include_files(sys/sysctl.h HAVE_SYS_SYSCTL_H)
check_include_files(sys/time.h HAVE_SYS_TIME_H)
check_include_files(sys/types.h HAVE_SYS_TYPES_H)
check_include_files(unistd.h HAVE_UNISTD_H)
check_include_files(arpa/inet.h HAVE_ARPA_INET_H)
check_include_files(ifaddrs.h HAVE_IFADDRS_H)
check_include_files(netinet/in.h HAVE_NETINET_IN_H)
check_include_files(netinet/tcp.h HAVE_NETINET_TCP_H)
check_include_files(net/if.h HAVE_NET_IF_H)

# Check Functions #
check_symbol_exists(strndup "string.h" HAVE_STRNDUP)
check_symbol_exists(strnlen "string.h" HAVE_STRNLEN)
check_symbol_exists(gettimeofday "sys/time.h" HAVE_GETTIMEOFDAY)
check_symbol_exists(usleep "unistd.h" HAVE_USLEEP)
check_symbol_exists(realpath "stdlib.h" HAVE_REALPATH)

# Check types
check_type_exists("_Bool" "stdbool.h" HAVE__BOOL)
check_type_size("long long int" LONG_LONG_INT)
check_type_size("unsigned long long int" UNSIGNED_LONG_LONG_INT)

# Check modules
pkg_check_modules(LIBUSB1 libusb-1.0)
if(LIBUSB1_FOUND)
    set(HAVE_LIBUSB1 1)
    set(use_libusb1 1)
    # LIBUSB1_CFLAGS=`echo $LIBUSB1_CFLAGS | sed 's/-I/-isystem /'`
    # libusb-1.0 header bug workaround: LIBUSB1_CFLAGS changed to "$LIBUSB1_CFLAGS"
    if(${LIBUSB1_VERSION} GREATER_EQUAL "1.0.16")
        set(HAVE_LIBUSB_GET_PORT_NUMBERS 1)
    endif()
else()
    message(WARNING "libusb-1.x not found, trying legacy libusb-0.1 as a fallback; consider installing libusb-1.x instead")
endif()

pkg_check_modules(LIBFTDI libftdi1)
if(LIBFTDI_FOUND)
    set(use_libftdi 1)
    if(${LIBFTDI_VERSION} GREATER_EQUAL "1.5")
        set(HAVE_LIBFTDI_TCIOFLUSH 1)
    endif()
endif()

list(APPEND hidapi_libs
	"hidapi"
	"hidapi-hidraw"
	"hidapi-libusb"
)
foreach(hidapi_lib ${hidapi_libs})
    pkg_check_modules(HIDAPI ${hidapi_lib})
    if(HIDAPI_FOUND)
        set(use_hidapi 1)
        break()
    endif()
endforeach()

pkg_check_modules(LIBGPIOD libgpiod)
if(LIBGPIOD_FOUND)
    set(use_libgpiod 1)
endif()

pkg_check_modules(LIBJAYLINK libjaylink>=0.2)
if(LIBJAYLINK_FOUND)
    set(use_libjaylink 1)
endif()

if(ENABLE_CAPSTONE)
    pkg_check_modules(CAPSTONE capstone)
    if(CAPSTONE_FOUND)
        set(HAVE_CAPSTONE 1)
    endif()
endif()


# Process options
if(doxygen_as_html)
#TODO
endif()

if(doxygen_as_pdf)
#TODO
endif()

# set default gcc warnings
list(APPEND gcc_warnings
	"-Wall"
	"-Wstrict-prototypes"
    "-Wformat-security"
    "-Wshadow"
)

if(GCC_WEXTRA)
    list(APPEND gcc_warnings
        "-Wextra"
        "-Wno-unused-parameter"
        "-Wbad-function-cast"
        "-Wcast-align"
        "-Wredundant-decls"
        "-Wpointer-arith"
        "-Wundef"
    )
endif(GCC_WEXTRA)

if(GCC_WERROR)
    list(APPEND gcc_warnings
        "-Werror"
    )
endif(GCC_WERROR)

if(VERBOSE_USB_IO OR VERBOSE_JTAG_IO)
    set(_DEBUG_USB_IO_ 1)
endif()

if(VERBOSE_USB_COMM OR VERBOSE_JTAG_IO)
    set(_DEBUG_USB_COMMS_ 1)
endif()

if(MALLOC_LOGGING)
    set(_DEBUG_FREE_SPACE_ 1)
endif()

if(BUILD_DUMMY)
    set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
endif()

if(${host_cpu} MATCHES "(i?86|x86*)")
    if(NOT PARPORT_USE_PPDEV)
        #TODO print message disable PARPORT_USE_PPDEV is not supported by the host CPU
    endif()
endif()

if(IS_MINGW)
    if(NOT PARPORT_USE_GIVEIO)
        message(WARNING "PARPORT_USE_GIVEIO cannot be disabled on MinGW hosts")
        set(PARPORT_USE_GIVEIO ON CACHE BOOL "" FORCE)
    endif()
    if(BUILD_BUS_PIRATE)
        message(WARNING "buspirate currently not supported by MinGW32 hosts")
        set(BUILD_BUS_PIRATE OFF CACHE BOOL "" FORCE)
    endif()
elseif(IS_CYGWIN)
    if(BUILD_PARPORT)
        check_include_files(sys/io.h have_sys_io_h)
        if(NOT have_sys_io_h)
            message(SEND_ERROR "Please install the cygwin ioperm package")
        endif()
    endif()
elseif(IS_DARWIN)

else()
    if(PARPORT_USE_GIVEIO)
        message(WARNING "PARPORT_USE_GIVEIO cannot be used by ${host} hosts")
        set(PARPORT_USE_GIVEIO OFF CACHE BOOL "" FORCE)
    endif()
endif()

if(BUILD_PARPORT)
    set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
endif()

if(BUILD_ESP_COMPRESSION)
    #AC_SEARCH_LIBS([deflate], [z],[AC_MSG_ERROR([libz library was not found])])
    check_include_files(zlib.h HAVE_ZLIB_H)
endif()

if(BUILD_GCOV)
    # CFLAGS+=" -fprofile-arcs -ftest-coverage"
endif()

if(${host_cpu} MATCHES "(arm*|aarch64)")
    if(BUILD_BCM2835GPIO)
        set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
    endif()

    if(BUILD_IMX_GPIO)
        set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
    endif()

    if(BUILD_AM335XGPIO)
        set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
    endif()
else()
    set(BUILD_BCM2835GPIO OFF CACHE BOOL "" FORCE)
    set(BUILD_IMX_GPIO OFF CACHE BOOL "" FORCE)
    set(BUILD_AM335XGPIO OFF CACHE BOOL "" FORCE)
endif()

if(${host_cpu} MATCHES "(arm*)")
    if(BUILD_EP93XX)
        set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
    endif()

    if(BUILD_AT91RM9200)
        set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
    endif()
else()
    set(BUILD_EP93XX OFF CACHE BOOL "" FORCE)
    set(BUILD_AT91RM9200 OFF CACHE BOOL "" FORCE)
endif()

if(BUILD_SYSFSGPIO)
    set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
endif()

if(use_internal_jimtcl)
#TODO test if jimtcl submodule is downloaded.
#TODO add config options "--disable-install-jim --with-ext=json"
endif()

if(use_internal_jimtcl_maintainer)
#TODO add config option "--maintainer"
endif()

if(NOT ${host_os} MATCHES "(linux*)")
    if (BUILD_SYSFSGPIO)
        message(SEND_ERROR "sysfsgpio is only available on linux")
    endif()
    if (BUILD_LINUXGPIOD)
        message(SEND_ERROR "linuxgpiod is only available on linux")
    endif()
    if (BUILD_XLNX_PCIE_XVC)
        message(SEND_ERROR "xlnx_pcie_xvc is only available on linux")
    endif()
    if (BUILD_RSHIM AND NOT ${host_os} MATCHES "(freebsd*)")
        message(SEND_ERROR "build_rshim is only available on linux or freebsd")
    endif()
	if (BUILD_DMEM)
		message(SEND_ERROR "dmem is only available on linux")
	endif()
endif()

# Process adapters
if(NOT use_libusb1)
    set(BUILD_FTDI OFF CACHE BOOL "" FORCE)
    set(BUILD_HLADAPTER_STLINK OFF CACHE BOOL "" FORCE)
    set(BUILD_HLADAPTER_ICDI OFF CACHE BOOL "" FORCE)
    set(BUILD_ULINK OFF CACHE BOOL "" FORCE)
    set(BUILD_USB_BLASTER_2 OFF CACHE BOOL "" FORCE)
    set(BUILD_FT232R OFF CACHE BOOL "" FORCE)
    set(BUILD_VSLLINK OFF CACHE BOOL "" FORCE)
    set(BUILD_XDS110 OFF CACHE BOOL "" FORCE)
    set(BUILD_CMSIS_DAP_USB OFF CACHE BOOL "" FORCE)
    set(BUILD_OSBDM OFF CACHE BOOL "" FORCE)
    set(BUILD_OPENDOUS OFF CACHE BOOL "" FORCE)
    set(BUILD_ARMJTAGEW OFF CACHE BOOL "" FORCE)
    set(BUILD_RLINK OFF CACHE BOOL "" FORCE)
    set(BUILD_USBPROG OFF CACHE BOOL "" FORCE)
    set(BUILD_ESP_USB_JTAG OFF CACHE BOOL "" FORCE)
    set(BUILD_AICE OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_hidapi)
    set(BUILD_CMSIS_DAP_HID OFF CACHE BOOL "" FORCE)
    set(BUILD_HLADAPTER_NULINK OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_hidapi OR NOT use_libusb1)
    set(BUILD_KITPROG OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_libftdi)
    set(BUILD_USB_BLASTER OFF CACHE BOOL "" FORCE)
    set(BUILD_PRESTO OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_libftdi OR NOT use_libusb1)
    set(BUILD_OPENJTAG OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_libgpiod)
    set(BUILD_LINUXGPIOD OFF CACHE BOOL "" FORCE)
endif()

if(NOT use_internal_libjaylink AND NOT use_libjaylink)
    set(BUILD_JLINK OFF CACHE BOOL "" FORCE)
endif()

if(BUILD_HLADAPTER_STLINK OR BUILD_HLADAPTER_ICDI OR BUILD_HLADAPTER_NULINK)
    set(BUILD_HLADAPTER ON CACHE BOOL "" FORCE)
endif()

if(BUILD_REMOTE_BITBANG)
    set(BUILD_BITBANG ON CACHE BOOL "" FORCE)
endif()

if(BUILD_PRESTO)
    set(BUILD_BITQ ON CACHE BOOL "" FORCE)
endif()

if(BUILD_ESP_USB_JTAG)
    set(BUILD_BITQ ON CACHE BOOL "" FORCE)
endif()

# jim-config.h is needed from openocd targets.
# This is a workaround to generate jim-config.h before any openocd target builds.
# FIXME: configure or build jimtcl before openocd build
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/jim-config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/jimtcl/jim-config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config.h)
set(CONFIG_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

set(OPENOCD_COMMON_COMPILER_FLAGS
    ${HOST_CPPFLAGS}
    -DHAVE_CONFIG_H
    ${gcc_warnings}
    -DPKGDATADIR="${pkgdatadir}" -DBINDIR="${bindir}"
    -std=gnu99 -g -O2
)

# TODO: add sanitizers and gcov as custom target.
# Check https://github.com/bilke/cmake-modules/blob/master/CodeCoverage.cmake
if(BUILD_GCOV)
    set(OPENOCD_COMMON_COMPILER_FLAGS ${OPENOCD_COMMON_COMPILER_FLAGS} -fprofile-arcs -ftest-coverage)
    SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")
endif()
