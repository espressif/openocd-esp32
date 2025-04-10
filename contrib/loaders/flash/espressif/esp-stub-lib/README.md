[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/espressif/esp-stub-lib/master.svg)](https://results.pre-commit.ci/latest/github/espressif/esp-stub-lib/master)

# esp-stub-lib

This project is experimental and not yet ready for production use.

## Supported Targets

- ESP8266
- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C2
- ESP32-C3
- ESP32-C5
- ESP32-C6
- ESP32-C61
- ESP32-H2
- ESP32-P4

## How to use

The library provides a simple interface for creating stubs that can be loaded onto ESP chips.

A complete example project is provided in the [example](example/) directory. It demonstrates:
- Basic stub implementation
- Flash operations
- Target-specific configurations
- Build system integration

See the [example README](example/README.md) for build instructions.

## Contributing

Please install the [pre-commit](https://pre-commit.com/) hooks to ensure that your commits are properly formatted:

```bash
pip install pre-commit
pre-commit install -t pre-commit -t commit-msg
```

# How To Release (For Maintainers Only)

```bash
pip install commitizen
git fetch
git checkout -b update/release_v1.1.0
git reset --hard origin/master
cz bump
git push -u
git push --tags
```
Create a pull request and edit the automatically created draft [release notes](https://github.com/espressif/esp-stub-lib/releases).

## License

This document and the attached source code are released as Free Software under either the [Apache License Version 2](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
