# ESP32 Stubs

These are small bits of code that run on the ESP32 to facilitate OpenOCD operation on the chip. The generated `.inc` files in these directories are included in the final `openocd` binary, but these `.inc` files need to be generated separately.

## Building Stubs

The stubs use a specific build of ESP-IDF. Look at the `stub_flasher_image.h` file for the `ESP32*_STUB_BUILD_IDF_REV` comment to find the ESP-IDF SHA that you need to use to build the stub. The process for each chip (e.g. esp32, esp32s3, etc) is roughly the same, though the stubs may use different versions of IDF. The process is:

1. Download the git repo for the ESP-IDF version.
```bash
git clone https://github.com/espressif/esp-idf.git
```

2. Checkout the right version of ESP-IDF based on the comment in `stub_flasher_image.h` for your target chip (e.g. `036bd3eb26` from `esp32s3/stub_flasher_image.h`)
```bash
cd esp-idf
git checkout 036bd3eb26 # or whatever the SHA is
git submodule update --init --recursive
```

3. Set up the ESP-IDF environment
```bash
./install.sh
source export.sh
```

4. Go back to your `openocd-esp32` directory and build the stub for your target chip
```bash
cd ../openocd-esp32/contrib/loaders/flash/espressif/esp32s3
```

5. Build the stub for all makefile targets. Currently, one target with disabled logs, and the other with `-DSTUB_LOG_ENABLE=1` macro. Latter will help to transfer the stub logs to the OpenOCD screen.
```bash
make all
```

6. Now you can go through the normal OpenOCD build process, and the stub images will be included in the final binary.
