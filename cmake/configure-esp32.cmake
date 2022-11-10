include(FindPkgConfig)
include(CheckIncludeFiles)
include(CheckSymbolExists)
include(CheckLibraryExists)
include(CheckTypeSize)
include("${CMAKE_CURRENT_LIST_DIR}/CheckTypeExists.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/CheckStruct.cmake")

set(host "esp-idf")
set(IS_ESPIDF 1)

set(pkgdatadir "\"/usr/local/share/openocd\"")
set(bindir "\"/usr/local/bin\"")

# TODO Search 'environ' in unistd.h or available libraries
set(NEED_ENVIRON_EXTERN 1)

# Search Libs #
check_library_exists(ioperm ioperm "" HAVE_IOPERM)
check_library_exists(dl dlopen "" HAVE_DLOPEN)
check_library_exists(util openpty "" HAVE_UTIL)

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

#TODO Look at the proper location
set(HAVE_USLEEP 1)          #newlib
set(HAVE_NETINET_IN_H 1)    #lwip
set(HAVE_NETDB_H 1)         #lwip

set(BUILD_BITBANG ON)
set(BUILD_ESP32_GPIO ON)
set(use_libusb1 0)
set(use_hidapi 0)
set(use_libftdi 0)
set(use_libgpiod 0)
set(use_libjaylink 0)

# set default gcc warnings
list(APPEND gcc_warnings
    "-Wno-error=misleading-indentation"
    "-Wno-error=format"
    "-Wno-error=stringop-overflow"
    "-Wno-error=incompatible-pointer-types"
    "-Wno-error=undef"
    "-Wno-undef"
    "-Wno-error=char-subscripts"
    "-Wno-error=override-init"
    "-Wno-error=int-conversion"
    "-Wno-error=unused-const-variable"
    "-Wno-error=implicit-function-declaration"
    "-Wno-redundant-decls"
    "-Wno-cast-function-type"
    "-Wno-implicit-fallthrough"
    "-Wno-missing-field-initializers"
    "-Wno-error=sign-compare"
    "-Wno-sign-compare"
    "-Wno-char-subscripts"
    "-Wno-incompatible-pointer-types"
    "-Wno-format"
)

# jim-config.h is needed from openocd targets.
# This is a workaround to generate jim-config.h before any openocd target builds.
# FIXME: configure or build jimtcl before openocd build
configure_file(${OPENOCD_DIR}/cmake/jim-config.h.in ${JIMTCL_DIR}/jim-config.h)
configure_file(${OPENOCD_DIR}/cmake/config.h.in ${OPENOCD_DIR}/config.h)
set(CONFIG_HEADER ${OPENOCD_DIR}/config.h)

set(OPENOCD_COMMON_COMPILER_FLAGS
    -DHAVE_CONFIG_H
    ${gcc_warnings}
    -DPKGDATADIR=${pkgdatadir} -DBINDIR=${bindir}
)
