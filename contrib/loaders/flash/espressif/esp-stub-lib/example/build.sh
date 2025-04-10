#!/bin/bash

TARGETS="esp8266 esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32h2 esp32p4"

ESP8266_LINUX_TOOLCHAIN_URL="https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64.tar.gz"
ESP8266_MACOS_TOOLCHAIN_URL="https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-macos.tar.gz"

download_esp8266_toolchain() {
    mkdir -p toolchain
    cd toolchain

    filename=$(basename "$ESP8266_TOOLCHAIN_URL")
    if [ ! -f "$filename" ]; then
        echo "Downloading ESP8266 toolchain... $ESP8266_TOOLCHAIN_URL"
        wget "$ESP8266_TOOLCHAIN_URL" --no-verbose
        tar -xzf "$filename"
        rm "$filename"
    else
        echo "ESP8266 toolchain already exists. Skipping download."
    fi

    cd ..
}

build_target() {
    local target=$1
    echo "Building for $target..."

    if [ "$target" = "esp8266" ]; then
        if [[ "$OSTYPE" == "windows"* ]]; then
            echo "ESP8266 build is only supported on unix-like systems. Skipping..."
            return
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            ESP8266_TOOLCHAIN_URL=$ESP8266_MACOS_TOOLCHAIN_URL
        else
            ESP8266_TOOLCHAIN_URL=$ESP8266_LINUX_TOOLCHAIN_URL
        fi

        # Download and setup ESP8266 toolchain if needed
        if [ ! -d "toolchain/xtensa-lx106-elf" ]; then
            download_esp8266_toolchain
        fi
        export PATH="$PWD/toolchain/xtensa-lx106-elf/bin:$PATH"
    fi

    mkdir -p build/$target
    cd build/$target
    cmake -DESP_TARGET=$target -GNinja ../..
    ninja
    cd ../..
}

if [ "$1" = "all" ]; then
    # Build all targets
    for target in $TARGETS; do
        build_target $target
    done
elif [ "$1" = "clean" ]; then
    rm -rf build
else
    # Build specific target
    if [[ ! " $TARGETS " =~ " $1 " ]]; then
        echo "Usage: $0 <target|all|clean>"
        echo "Available targets: $TARGETS"
        exit 1
    fi
    build_target $1
fi

# usage: ./build.sh all
# usage: ./build.sh esp32
# usage: ./build.sh clean
