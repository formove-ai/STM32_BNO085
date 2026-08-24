# BNO085 SPI Library Mocks

This directory contains CMock-generated mocks for the BNO085 SPI Library, along with the CMock and Unity runtime files needed to use them.

## What's Included

- `MockBNO085_SPI_Library.c/h` - Generated mock for all public functions in `BNO085_SPI_Library.h`
- `cmock.c/h` and `cmock_internals.h` - CMock runtime (v2.7.0)
- `unity.c/h` and `unity_internals.h` - Unity test framework runtime (CMock dependency)
- `cmock_config.yml` - CMock configuration used for generation
- `generate_mocks.sh` - Script to regenerate mocks
- `docker-compose.yml` - Docker-based mock generation (no local Ruby needed)

## Using the Mocks

### As a PlatformIO Library

Add this directory as a library dependency in your `platformio.ini`:

```ini
[env:your_test_env]
platform = native
lib_deps =
    symlink://path/to/STM32_BNO085/mock
```

The mock directory is structured as a complete PlatformIO library and includes all necessary runtime files.

### In Your Tests

```c
#include "MockBNO085_SPI_Library.h"
#include "unity.h"

void test_example(void) {
    sensor_meta sensor;
    
    // Set up mock expectations
    register_Sensor_Expect(&sensor, 1, 0x01, NULL, 0x02, NULL, 0x03, NULL);
    data_available_ExpectAndReturn(&sensor, true);
    get_Accelerometer_X_ExpectAndReturn(&sensor, 1.5f);
    
    // Call your code that uses the library
    // ...
    
    // Unity will verify all expectations were met
}
```

## Regenerating Mocks

Regenerate mocks whenever `include/BNO085_SPI_Library.h` changes.

### Option 1: Docker (Recommended - Works on Windows/Linux/macOS)

From the `mock/` directory, run:

```bash
docker compose run --build --rm generate-mocks
```

No local Ruby installation required. Docker will download CMock v2.7.0, generate the mocks, and update the files in this directory.

### Option 2: Local Ruby

If you have Ruby >= 2.5 and git installed, run from the `mock/` directory:

```bash
./generate_mocks.sh
```

Both methods:

- Clone CMock v2.7.0 from GitHub
- Generate fresh mocks from the current header
- Update `MockBNO085_SPI_Library.c/h` in place
- Copy CMock and Unity runtime files if missing

## CI Mock Freshness Check

The repository includes a GitHub Actions workflow (`.github/workflows/check-mocks.yaml`) that automatically regenerates mocks on every pull request and fails if they don't match the checked-in versions. This ensures the mocks never drift from the header.

## CMock Configuration

The mock generation behavior is controlled by `cmock_config.yml`:

- **Plugins**: `ignore` and `callback` plugins enabled
- **Type handling**: Custom types like `uint8`, `uint16`, `uint32` mapped to hex/int formats
- **Strict ordering**: Enabled by default - mock calls must occur in the expected order

## Version Pinning

- **CMock**: v2.7.0 (pinned for reproducibility)
- **Unity**: Latest from main branch (as of mock generation)

Pinning CMock to v2.7.0 ensures that regenerating mocks produces consistent output across different environments and over time.

## Learn More

- [CMock Documentation](https://github.com/ThrowTheSwitch/CMock)
- [Unity Test Framework](https://github.com/ThrowTheSwitch/Unity)
- [PlatformIO Native Testing](https://docs.platformio.org/en/latest/plus/unit-testing.html)
