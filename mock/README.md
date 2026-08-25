# Generated mock of the BNO085 SPI library

A CMock mock of every function in `include/BNO085_SPI_Library.h`, checked in
next to the header it mirrors and shipped as a PlatformIO library. Projects that
build on this library can unit test their own code on the host without an STM32,
without Ruby and without Docker.

## Contents

- `MockBNO085_SPI_Library.c` / `.h` - the generated mock.
- `cmock.c`, `cmock.h`, `cmock_internals.h` - the CMock runtime, vendored so
  that consumers need no Ruby. Unity is deliberately *not* vendored: PlatformIO
  supplies it to test builds and a second copy would clash at link time.
- `cmock_strict_order.c` - the two counters CMock's strict ordering expects the
  test runner to define.
- `bno085_native_hal_stub.h` - host stand-ins for the three STM32 HAL types the
  public headers name.
- `cmock_config.yml`, `generate_mocks.sh`, `docker-compose.yml` - regeneration.

## Using it

Add the mock as a dependency and point the compiler at this library's headers.
The library sources themselves must *not* be built, because the mock provides
every one of their symbols:

```text
[env:native_mock]
platform = native
build_flags =
    -D BNO085_BUILD_WITH_NATIVE_HAL_STUB
    -lm
    -I path/to/STM32_BNO085/include
lib_deps =
    symlink://path/to/STM32_BNO085/mock
```

`BNO085_BUILD_WITH_NATIVE_HAL_STUB` makes `stm32_hal.h` pick up
`bno085_native_hal_stub.h` instead of a vendor HAL. The stub declares only the
types that appear in the public signatures - `GPIO_TypeDef`, `SPI_TypeDef` and
`SPI_HandleTypeDef` - and no HAL functions, so a test that accidentally reaches
the HAL fails to link rather than silently doing nothing.

A test then sets expectations instead of talking to hardware:

```c
#include <unity.h>

#include "MockBNO085_SPI_Library.h"

void setUp(void) { MockBNO085_SPI_Library_Init(); }

void tearDown(void) {
  MockBNO085_SPI_Library_Verify();
  MockBNO085_SPI_Library_Destroy();
}

void test_reader_publishes_a_sample(void) {
  sensor_meta sensor;

  data_available_ExpectAndReturn(&sensor, true);
  get_Quat_I_ExpectAndReturn(&sensor, 0.25f);

  // ... exercise the code under test ...
}
```

`MockBNO085_SPI_Library_Verify()` in `tearDown` is what makes an expectation
that never happened fail the test.

`test/test_mock_consumer/` in this repository is a complete worked example and
runs in CI, so it stays correct.

### Call ordering

`cmock_config.yml` enables strict ordering: calls must arrive in the order they
were expected, and an out-of-order call fails with *Called earlier than
expected*. Use `<function>_Ignore()` for calls whose ordering does not matter.

## Regenerating

Required whenever `include/BNO085_SPI_Library.h` changes. CI fails the pull
request otherwise.

Without a local Ruby, from this directory:

```bash
docker compose run --rm generate-mocks
```

With Ruby 2.5 or newer and git, from anywhere:

```bash
./mock/generate_mocks.sh
```

Both routes clone CMock at the pinned `v2.7.0` tag together with the Unity
revision that release pins, so regenerating produces byte-identical output on
Linux, macOS and Windows.

## Learn more

- [CMock](https://github.com/ThrowTheSwitch/CMock)
- [Unity](https://github.com/ThrowTheSwitch/Unity)
- [PlatformIO unit testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html)
