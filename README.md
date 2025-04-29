# Library for CEVA BNO085 IMU on STM32

## Version

[![Version](https://img.shields.io/badge/version-<VERSION>-blue)](https://github.com/formove-ai/STM32_BNO085/releases)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

---

## Table of Contents

- [About](#about)
- [Installation](#installation)
- [Configuration and Prerequisites](#configuration-and-prerequisites)
- [Usage](#usage)
- [Versioning](#versioning)
- [Contributing](#contributing)
- [Code of Conduct](#code-of-conduct)
- [Testing](#testing)
- [License](#license)
- [Authors & Contributors](#authors--contributors)
- [Acknowledgements](#acknowledgements)
- [Support](#support)

---

## About

This library was developed to connect multiple IMUs BNO085 from CEVA, Inc. to a STM32
microcontroller.
Key facts:

- The library connects via SPI the IMUs BNO085 from CEVA, Inc. to a STM32 microcontroller.

- This version is tested with STM32 of type STM32G0B1 and its derivatives.

- It is possible to select different sensor fusion modes and to send different commands.

- It is possible to connect several IMUs to one microcontroller. Hardware chip select
lines allow conflict free communication over the SPI bus.

- Each IMU gets a struct variable which stores the metadata of the sensor.

## Installation

## Configuration and Prerequisites

The library configures the GPIO ports and pins as well as peripherals like the
SPI interface. Depending on the type of microcontroller, slightly different
configurations are necessary. In the data sheet of the corresponding type of
microcontroller the necessary parameters can be found.

Among other things, the following must be checked / implemented:

- Static pins: If the IMU is connected to the microcontroller with other pins,
which are not directly necessary for SPI, the respective pins must be statically set to
the corresponding logic level (high or low). Information about this can be obtained
from the data sheet of the IMU [1]. Usually the following applies:
  - PS0, PS1 and BOOTN: High
  - RSTN: Low
- Shims: The user must implement a delay function `delay_Us` for microseconds, which is
  implemented as a shim.
- The peripheral clocks for relevant GPIO lines (e.g. A,B,C,D) or modules (e.g. TIM2)
must be enabled, e.g. Timer 2 Clock: `__HAL_RCC_TIM2_CLK_ENABLE()`

- SPI:
  - To initialize the SPI interface, the user must initialize a struct of the type
    `bno085_library_spi_config_struct` and fill it according to his use case
    (MISO, MOSI, CSN, Port, SPI-Instance).
  - Depending on which SPI interface and which STM32 microcontroller is used, SPI uses
    one of the APB peripheral clocks (APBx clocks). The user must calculate a prescaler
    according to the respective APBx clock. The maximum allowed clock frequency is 3 MHz
    (cf. [1], p. 47).
  - Alternate function mapping: The parameter describe which pin configuration is used
    for SPI. Depending on the type of microcontroller and hardware / board layout the
    corresponding parameter must be selected.
- IMU initialisation: Every IMU needs a pointer to a sensor struct containing the IMU
metadata and configuration data. The initialisation can be found in the [Usage](#usage)
section.

## Usage

## Versioning

Current release: **[`<VERSION>`](https://github.com/formove-ai/STM32_BNO085/releases/tag/<VERSION>)**

## Contributing

All contributions are welcome! Please read [how to contribute](CONTRIBUTING.md)
for guidelines.

## Code of Conduct

This project adheres to the [Contributor Covenant](CODE_OF_CONDUCT.md).
By participating, you are expected to uphold this code.

## Testing

For testing the library with Unity, switch to the PlatformIO project environment
*native*, open a PlatformIO Core CLI and run:

```bash
platformio test --environment native
```

## License

Copyright (c) 2025 Paul Gollwitzer, Alexander Oldermeier

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors & Contributors

- Paul Gollwitzer – *maintainer*
- Alexander Oldermeier - *maintainer*

## Acknowledgements

- [1] BNO08X Data Sheet v1.16, CEVA, Hillcrest Labs
- [2] SH-2 Reference Manual v1.9, CEVA, Hillcrest Labs
- [3] Some function prototypes are based on the Arduino library
[Sparkfun BNO080](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library),
which is released under the MIT License.

## Support

If you have any questions, please read [how to contribute](CONTRIBUTING.md) and
ask the authors directly. Do not open an issue ticket.
