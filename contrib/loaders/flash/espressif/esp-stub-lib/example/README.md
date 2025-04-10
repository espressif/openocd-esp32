## Building the Example

### Using build script

The simplest way to build is using the provided build script:

```bash
# Build for all supported targets
./build.sh all

# Or specify a different target
./build.sh esp8266

# Or to clean all built files
./build.sh clean
```

### Manual Build

1. Create and enter build directory:
```bash
cd example
mkdir -p build && cd build
```

2. Configure CMake with your ESP target:
```bash
cmake -DESP_TARGET=esp32 .. # Replace esp32 with your target (esp32s2, etc.)
```

3. Build the project:
```bash
cmake --build .
```

## Build Outputs

After a successful build, you'll find:
- `build/stub_<target>.elf` - The compiled binary
- `build/stub_<target>.map` - Memory map file
- `build/stub_<target>.asm` - Disassembly output

## Notes

- Make sure you have exported the esp-idf environment with the correct toolchain for your target ESP chip. https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html
- The ESP8266 toolchain is older than the others. `./build.sh esp8266` will handle installing it.
- The example assumes the parent directory contains the ESP stub library
