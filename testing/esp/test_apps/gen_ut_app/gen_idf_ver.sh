#!/bin/sh

if [ -z "$IDF_PATH" ]; then
	echo "IDF_PATH is not set!"
	exit
fi

if [ "$UT_GET_IDF_VER" = "1" ]; then
    cd $IDF_PATH
    git describe --tags | cut -c2- | awk -F. -v OFS=. '{printf("MAKE_UT_IDF_VER(%d,%d,%d,%d)\n", $1, $2, $3, $4)}'
	exit
fi

echo "UT_IDF_VER_LATEST"
