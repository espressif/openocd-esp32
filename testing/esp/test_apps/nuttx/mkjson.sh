#!/bin/bash

testdir=$1
appname=$2
chip=$3
flasher_file="${testdir}/${appname}/flasher_args.json"

case "$chip" in
    "esp32"|"esp32s2") btoffset=0x1000
    ;;
    *) btoffset=0x0
    ;;
esac

cp testing/esp/test_apps/nuttx/flasher_args.json ${flasher_file}
sed -i.bak "s/__APP_BIN__/${appname}.bin/g" ${flasher_file} && rm ${flasher_file}.bak
sed -i.bak "s/__BT_OFFSET__/${btoffset}/g" ${flasher_file} && rm ${flasher_file}.bak
sed -i.bak "s/__CHIP__/${chip}/g" ${flasher_file} && rm ${flasher_file}.bak
