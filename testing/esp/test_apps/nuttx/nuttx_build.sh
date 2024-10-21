#!/bin/bash

# This is a helper script used to run the NuttX tests locally. Although it shares
# some steps with what the CI scripts do, this script is not used in the CI system.
# Other means offered by Gitlab are used to build the NuttX application and download
# the artifacts.

MAKE_OPTS="-s -j"
TESTDIR="nuttx_test"
APPNAME="nuttx_openocd"

nuttxdir="nuttx"
oocddir="$PWD"
testpath="${oocddir}/testing/esp/test_apps/nuttx"
exampledir="${testpath}/${TESTDIR}/${APPNAME}"

USAGE="

USAGE: ${0} [-d <nuttx-top-dir>] [-c <chip>] [-n <nuttx-dir>] [-b <debug-board>]

Where:
  -d <nuttx-top-dir> is the full path to the NuttX top directory.  Here is where
  all the repositories are stored.
  -c <chip> The ESP32xx chip
  -n <nuttx-dir> The name of the NuttX OS repository.
  -b <debug-board> The debug board.  Depeding on the chip and the hardware used, this can take
  multiple values.

  Example usage: ./testing/esp/test_apps/nuttx/nuttx_build.sh -c esp32 -d ~/NuttX
"

while [ ! -z "$1" ]; do
    case "$1" in
    -d )
        shift
        topdir=$1
        ;;
    -c )
        shift
        chip=$1
        ;;
    -n )
        shift
        nuttxdir=$1
        ;;
    -b )
        shift
        dbgboard=$1
        ;;
    -clean )
        rm -rf ${testpath}/${TESTDIR}
        exit 0
        ;;
    -h )
        echo "${USAGE}"
        exit 0
        ;;
    esac
    shift
done

if [[ -z ${topdir} ]] || [[ ! -d ${topdir} ]]; then
  echo "ERROR: NuttX top directory not found!"
  exit 1
fi

if [ -z ${chip} ]; then
  echo "ERROR: No chip specified!"
  exit 1
fi

case "${chip}" in
  "esp32")
      config="esp32-devkitc:nsh"
      dbgboard=${dbgboard:-"esp32-wrover-kit-3.3v"}
      toolchain="xtensa-esp32-elf-"
  ;;
  "esp32s2")
      config="esp32s2-soala-1:nsh"
      dbgboard=${dbgboard:-"esp32s2-kaluga-1"}
      toolchain="xtensa-esp32s2-elf-"
  ;;
  "esp32c3")
      config="esp32c3-devkit:nsh"
      dbgboard=${dbgboard:-"esp32c3-builtin"}
      toolchain="riscv32-esp-elf-"
  ;;
  "esp32s3")
      config="esp32s3-devkit:nsh"
      dbgboard=${dbgboard:-"esp32s3-builtin"}
      toolchain="xtensa-esp32s3-elf-"
  ;;
  "esp32c6")
      config="esp32c6-devkitc:nsh"
      dbgboard=${dbgboard:-"esp32c6-builtin"}
      toolchain="riscv32-esp-elf-"
  ;;
  "esp32h2")
      config="esp32h2-devkit:nsh"
      dbgboard=${dbgboard:-"esp32h2-builtin"}
      toolchain="riscv32-esp-elf-"
  ;;
  *)
      echo "ERROR: Invalid chip:${chip}"
      exit 1
  ;;
esac

if [ ! -d ${topdir}/${nuttxdir} ]; then
  echo "ERROR: NuttX directory not found!"
  echo "Default value is 'nuttx', use -n option if needed."
  exit 1
fi

bindir="${topdir}/esp-bins"
cd ${topdir}/${nuttxdir}

echo "Building ${config} with OpenOCD example additions..."
./tools/configure.sh -E ${config} ${MAKE_OPTS} 1>/dev/null

kconfig-tweak -d NDEBUG
kconfig-tweak -e ESP32_MERGE_BINS
kconfig-tweak -e ESP32S2_MERGE_BINS
kconfig-tweak -e ESP32S3_MERGE_BINS
kconfig-tweak -e ESP32C3_MERGE_BINS
kconfig-tweak -e ESPRESSIF_MERGE_BINS
kconfig-tweak --set-str USER_ENTRYPOINT openocd_main
make olddefconfig ${MAKE_OPTS} 1>/dev/null
make ${MAKE_OPTS} 1>/dev/null

echo "Copying files..."
rm -rf ${exampledir}
mkdir -p ${exampledir}
cp nuttx ${exampledir}/nuttx_openocd.elf
cp nuttx.merged.bin ${exampledir}/nuttx_openocd.bin

echo "Running tests..."
cd ${oocddir}
PYTHONPATH=$PWD/testing/esp/py_debug_backend $PWD/testing/esp/run_tests.py -d 4 -b ${dbgboard} -i other -p 'test_nuttx' -l $PWD/oocd_tests.log -s $PWD/tcl -a ${testpath}/${TESTDIR} -t ${toolchain} -c "set ESP_RTOS nuttx"
