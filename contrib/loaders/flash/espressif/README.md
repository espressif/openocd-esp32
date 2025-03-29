## Building Stub for OpenOCD

### Using build script

The simplest way to build is using the provided build script:

```bash
# Build for all supported targets
./build.sh all

# Or specify a different target
./build.sh esp32s2

# Clean all built files and directories
./build.sh clean
```

### Manual Build

1. Create and enter build directory:
```bash
mkdir -p build && cd build
```

2. Configure CMake with your ESP target:
```bash
mkdir esp32 && cd esp32
cmake -DESP_TARGET=esp32 ../.. # Replace esp32 with your target (esp32s2, etc.)
```

3. Build the project:
```bash
cmake --build .

# Or
make -j4
```

## Build Outputs

After a successful build, you'll find:
- `build/<target>/stub_<target>.elf` - The compiled binary
- `build/<target>/stub_<target>.map` - Memory map file
- `build/<target>/stub_<target>.asm` - Disassembly output

## Notes

- Make sure you have exported the esp-idf environment with the correct toolchain for your target ESP chip. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html
- The ESP8266 toolchain is older than the others. `./build.sh esp8266` will handle installing it.
- The example assumes the parent directory contains the ESP stub library
