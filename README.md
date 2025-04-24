# Description of the library "BNO085_SPI_Library"

Contact person and author: Paul Gollwitzer (TUM, MiMed), email: <paul.gollwitzer@tum.de>

## Library functionality

The library connects via SPI the IMUs BNO085 from CEVA, Inc. to a STM32 microcontroller.
This version is currently adapted to the STM32 of type STM32G0B1 and its derivatives.
It is possible to select different sensor fusion modes and to send different commands.
It is possible to connect several IMUs to one microcontroller. Hardware chip select
lines allow conflict free communication over the SPI bus. Each sensor gets a struct
variable which stores the metadata of the sensor. The functions are passed this
metadata when calling functions with call by reference. Each function returns a
return value that indicates whether the function or command was executed correctly.
Exceptions are functions that return data (for example to store them in global
variables) or update functions.

## References

- [1] BNO08X Data Sheet v1.16, CEVA, Hillcrest Labs
- [2] SH-2 Reference Manual v1.9, CEVA, Hillcrest Labs
- Some function prototypes are based on the Arduino library for the BNO080 of
  Sparkfun, <https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library>,
  which is released under the MIT License.

## Copyright

See the original [readout library](/sensorsuit/experimental/imu_readout_library)
for details.

## Configuration of Hardware

In the file Hardware_Init.c GPIO ports and pins as well as peripherals like the
SPI bus are configured. The functions called in this file make a graphical
initialization in the CubeIDE .ioc file unnecessary. Depending on the type of
microcontroller slightly different configurations are necessary. In the data sheet
of the corresponding type of microcontroller the necessary adjustments can be found.

Among other things, the following must be checked:

- Static pins: If the IMU is connected to the microcontroller with other pins,
which are not necessary for SPI, the respective pins must be statically set to
the corresponding logic level (high or low). Information about this can be obtained
from the data sheet [1] of the IMU.
- Timer: A timer is configured in the function HAL_Init_Timer(void). Not every
microcontroller has a PCLK1 clock. Other clocks can be integrated accordingly,
the prescaler is automatically recalculated.
- Port clocks: If other ports are used, the respective peripheral clocks must be
enabled. For example with __HAL_RCC_GPIOA_CLK_ENABLE() the clock of port A is enabled.
Adjustments take place in the functions: HAL_Init_GPIO_Sensor(),
HAL_Init_GPIO_RSTN() and HAL_Init_SPI().
- SPI:
  - Not every microcontroller has a HCLK clock. Other clocks can be integrated
  accordingly, the prescaler is automatically recalculated and limited.
  The maximum clock frequency is 3 MHz (cf. [1], p. 47).
  - Alternate function mapping: The parameter GPIO_AF5_SPI1 oder GPIO_AF0_SPI1
  describe which pin configuration is used for SPI1. Depending on the type of
  microcontroller and hardware / board layout one of the two parameters must be selected.

## Steps before building the project

1.Enable HAL_TIM_MODULE_ENABLED and HAL_SPI_MODULE_ENABLED in Core/Inc/stm32l5xx_hal_conf.h

- Reason: Code Generation normally enables the modules automatically. But this
  project doesn't use code generation for those modules
- Notice: After every automatic code generation this step has to be repeated

2.Copy the needed drivers for SPI in the drivers directory
  (Drivers/STM32L5xx_HAL_Driver/SRC or .../INC)

- These are: stm32l5xx_hal_spi.c and .h; stm32l5xx_hal_spi_ex.c and .h

**Notice:** Depending on the type of microcontroller the files are called differently.

## Functions and commands

In the following the available functions and commands are explained.

### Initialization of Hardware

---

These functions are used to initialize the hardware.

- ```uint8_t init_Hardware(void)```

  - **Description:** Initializes Hardware: SPI interface, Timer, GPIOs.
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```void init_GPIO_IMU(sensor_meta *sensor)```

  - **Description:** Initializes GPIOs for IMUs.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data

### Initialization / startup of IMU

---

This function registers the sensors and links them to the sensor structs.

- ```c
  void register_Sensor(sensor_meta *sensor, uint8_t sensor_number, uint16_tsensor_CSN_Pin, GPIO_TypeDef *sensor_CSN_Port, uint16_t sensor_INTN_Pin, GPIO_TypeDef *sensor_INTN_Port)```

  - **Description:** Register the corresponding sensor, store the sensor number, pins and ports in the corresponding sensor struct
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - sensor_number: Manually assigned number of sensor, identifier of sensor
    - sensor_CSN_Pin: Desired CSN Pin of sensor
    - sensor_CSN_Port: Pointer to desired CSN Port of sensor
    - sensor_INTN_Pin: Desired INTN Pin of sensor
    - sensor_INTN_Port: Pointer to desired INTN Port of sensor

### Reset

---

The following functions perform a reset. A distinction is made:

- Hardware reset (harrest_IMUs()): All IMUs are reset via the common reset line
  of all IMUs. At startup, each IMU delivers an advertisement message followed
  by an unsolicited initialization message.
- Software reset: The IMU is reset by command. Each IMU can be reset separately.
  A distinction is made:
  - softreset_IMU(): Reset without deleting the Dynamic Calibration Data (DCD).
  - softresetDCD_IMU(): Reset with simultaneous deletion of the Dynamic Calibration
    Data (DCD)
- **Important:**
  - After the hardware reset, it is mandatory to delete the advertising message.
    For this the function get_and_clear_Reset_Status() must be called directly
    afterwards for the sensor for which a command or report is to be set subsequently.
  - With softressetDCD_IMU() this happens automatically for corresponding IMU.
    With softreset_IMU() no advertising message is sent.
  - Subsequently a successful reset can be checked with the function check_Command_Success().

- ```c
  void hardreset_IMUs(void)```

  - **Description:** Hardware reset of IMUs (one reset line for all IMUs)
  - **Note:** Also necessary for booting in SPI mode.

- ```c

  uint8_t softreset_IMU(sensor_meta *sensor)```

  - **Description:** Sends reset command. softreset_IMU() does not clear DCD
    (Dynamic Calibration Data).
  - **Note:**
    - Software reset can be used instead of hardware reset reset_IMUs() to keep
      DCD (cf. [1], p. 23; p. 39).
    - If DCD should also be reset, then use hardware reset reset_IMUs() or softresetDCD_IMU().
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t softresetDCD_IMU(sensor_meta *sensor)```

  - **Description:** Sends reset command. softresetDCD_IMU() does clear DCD
    (Dynamic Calibration Data).
  - **Note:**
    - Software reset can be used instead of hardware reset reset_IMUs() (cf. [2],
      p. 55).
    - If DCD should not be reset, then use softreset_IMU().
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  bool get_and_clear_Reset_Status(sensor_meta *sensor)```

  - **Description:** Indicates a received reset complete packet. Once it's been
    read, the state will reset to false until another reset complete packet is found.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** true if reset, false if not

- ```c

  uint8_t clear_init_Message_IMU(sensor_meta *sensor)```

  - **Description:** Reads any init message and throws it away.
  - **Note:**
    - The advertisement message is not needed.
    - Could also be used to delete a unsolicited timestamp response.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

### Reports / Modes

---

The following report modes can be set:

- Rotation Vector
- ARVR-Stabilized Rotation Vector
- Game Rotation Vector
- ARVR-Stabilized Game Rotation Vector
- Stability Classifier
- Tap Detector

**Important:** Multiple modes can also be activated for one sensor, but not
multiple rotation vector modes. For example, the Rotation Vector mode can be
combined with the Stability Classifier mode.

- ```c
  uint8_t enable_RotationVector(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report Rotation Vector and sets the desired report
    delay (frequency).
  - **Note:** Calls set_FeatureCommand with no specific_Config which sends the
    config package.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t enable_ARVR_stabilized_RotationVector(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report ARVR-Stabilized Rotation Vector and sets the desired report delay (frequency).
  - **Note:** Calls set_FeatureCommand with no specific_Config which sends the config package.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t enable_GameRotationVector(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report Game Rotation Vector and sets the desired report delay (frequency).
  - **Note:** Calls set_FeatureCommand with no specific_Config which sends the config package.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t enable_ARVR_stabilized_GameRotationVector(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report ARVR-Stabilized Game Rotation Vector and sets the desired report delay (frequency).
  - **Note:** Calls set_FeatureCommand with no specific_Config which sends the config package.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t enable_StabilityClassifier(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report StabilityClassifier and sets the desired report delay (frequency).
  - **Note:**
    - Calls set_FeatureCommand with no specific_Config which sends the config package.
    - Possible classifiers: 0 = unknown, 1 = on table, 2 = stationary, 3 = stable, 4 = motion (cf. [2], p. 77 f.)
    - Classifier 3 only available if gyroscope calibration enabled
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t enable_TapDetector(sensor_meta *sensor, uint16_t time_between_reports)```

  - **Description:** Enables the report Tap Detector and sets the desired report delay (frequency).
  - **Note:** Calls set_FeatureCommand with no specific_Config which sends the config package.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - time_between_reports: Desired time in ms between two reports
  - **Return Value:** status, 1 no error occurred, 0 error occurred

### Update functions (have to be called periodically)

---

These functions must be called periodically in the main loop. If data is available,
the functions update the corresponding sensor data.

- ```c
  bool data_available(sensor_meta *sensor)```

  - **Description:** Executes the sensor routine (call it periodically / as often as possible), updates variables if possible.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** True if new readings available, false if not

- ```c

  void update_TapDetector(sensor_meta *sensor)```

  - **Description:** Update the Tap Detector, checks if the tap is a single or a double tap, sets the specific flag and counts the taps.
  - **Note:** Taps in all spatial directions are classified as taps in this case (cf. [2], p. 75).
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data

### Functions that read or calculate data

---

These functions return values from the sensor struct. Some functions convert the
values or calculate a return value.

- ```c
  float get_Quat_I(sensor_meta *sensor)```

  - **Description:** Calculate the unit quaternion i component with the specific Q point.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** quat (float), Unit quaternion component i  

- ```c

  float get_Quat_J(sensor_meta *sensor)```

  - **Description:** Calculate the unit quaternion j component with the specific Q point.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** quat (float), Unit quaternion component j

- ```c

  float get_Quat_K(sensor_meta *sensor)```

  - **Description:** Calculate the unit quaternion k component with the specific Q point.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** quat (float), Unit quaternion component k

- ```c

  float get_Quat_Real(sensor_meta *sensor)```

  - **Description:** Calculate the unit quaternion real component with the specific Q point.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** quat (float), Unit quaternion component real

- ```c

  float get_Quat_Radian_Accuracy(sensor_meta *sensor)```

  - **Description:** Calculate the estimated rotation vector accuracy in radian with the specific Q point (only for report Rotation Vector and ARVR-Stabilized Rotation Vector).
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** acc (float), estimated (ARVR-Stabilized) Rotation Vector
    accuracy in radian

- ```c

  uint8_t get_Quat_Accuracy(sensor_meta *sensor)```

  - **Description:** Return the (ARVR-Stabilized / Game) Rotation Vector accuracy.
  - **Note:** Assignment: 0 = Unreliable, 1 = Accuracy Low, 2 = Accuracy Medium, 3 = Accuracy High (cf. [1], p. 38)
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** (ARVR-Stabilized / Game) Rotation Vector accuracy

- ```c

  uint8_t get_StabilityClassifier(sensor_meta *sensor)```

  - **Description:** Return the Stability Classifier.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** Stability Classifier

### Commands

---

The respective commands are set via a corresponding function.

- To check if the command was set correctly, the IMU returns a response. This is
  read and checked with the function check_Command_Success().
- **Important:** All Tare commands and config_periodic_DCD_IMU() have no response.
  A check with the function check_Command_Success() does not work and leads to
  an error.

- ```c
  uint8_t get_ProductID(sensor_meta *sensor)```

  - **Description:** Gets the Product ID via Product ID Request, waits for the response and stores data.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t get_Reset_Reason(sensor_meta *sensor)```

  - **Description:** Reads and stores the reset reason for the last reset in sensor struct.
  - **Note:**
    - Not working for softreset_IMU(), because softreset_IMU() is executed on an other channel.
    - Assignment: 0 = Not applicable, 1 = Power on reset, 2 = Internal System Reset, 3 = Watchdog Timeout, 4 = External reset, 5 = Other
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t check_Connection_IMU(sensor_meta *sensor)```

  - **Description:** Checks the connection of IMU via Product ID Request and Response.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t tare_IMU(sensor_meta *sensor, bool all_Axis)```

  - **Description:** Sends Tare Now command. User can decide between all axis Tare or z axis Tare.
  - **Note:**
    - If Rotation Vector, then reorientation of all motion outputs.
    - If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be found before Tare. The user can f.ex. use a loop in main to repeat Tare until accuracy is 3 (cf. [1], p. 42).
    - The function detects if the magnetic north has not yet been found.
    - If Game Rotation Vector, then only tare of Game Rotation Vector itself. Z axis Tare cannot be persistent (no absolute reference for heading).
    - If z axis Tare and (ARVR-Stabilized) Game Rotation Vector, the function use softreset_IMU() to start with new config (cf. [1], p. 42).
    - If ARVR-Stabilized modes, then tare of underlying modes.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - all_Axis: Boolean variable to choose Tare mode. True for all axis Tare.
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t tare_persist_IMU(sensor_meta *sensor)```

  - **Description:** Store the last Tare command to flash.
  - **Note:**
    - Only persists if mode is (ARVR-Stabilized) Rotation Vector.
    - If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be found before Tare. The user can f.ex. use a loop in main to repeat Tare until accuracy is 3 (cf. [1], p. 42).
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t tare_set_reorientation_IMU(sensor_meta *sensor)```

  - **Description:** Set Reorientation of Tare, set current run-time sensor orientation.
  - **Note:**
    - Does not replace any persistent Tare settings. To clear Tare settings of flash by calling tare_clear_IMU().
    - If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be found before Tare. The user can f.ex. use a loop in main to repeat Tare until accuracy is 3 (cf. [1], p. 42).
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t tare_clear_IMU(sensor_meta *sensor)```

  - **Description:** Clears current Tare by setting current run-time sensor orientation to zero.
  - **Note:** Clears persistent Tare settings.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t reinitialize_IMU(sensor_meta *sensor)```

  - **Description:** Reinitializes the IMU.
  - **Note:** The initialize response is not unsolicited. The advertisement message is cleared.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t save_DCD_IMU(sensor_meta *sensor)```

  - **Description:** Saves Dynamic Calibration Data (DCD).
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t configure_ME_Calibration_IMU(sensor_meta *sensor, bool enable_Accel_Cal, bool enable_Gyro_Cal, bool enable_Mag_Cal, bool enable_Planar_Accel_Cal, bool enable_On_Table_Cal)```

  - **Description:** Configure calibration routines, enable / disable accelerometer, gyro or magnetometer calibration routine.
  - **Note:**
    - Calibration settings do not persist across reset.
    - General recommendations (cf. [1], p. 38):
    - Enable Accel Cal for reducing zero-g offset (ZGO), enable Planar Accel Cal if calibration is only in 2D possible.
    - Enable Gyro Cal for reducing zero rate offset (ZRO) / drift. For successful calibration the device to which the IMU is mounted must have sufficient tremor (f.ex. tremor of human hand).
    - If you perform slow horizontal rotations around the gravity vector, the calibration can be fooled. Disable the gyro cal.
    - Enable Mag Cal for compensation of hard-iron effects (magnets) or soft-iron (ferrous materials) effects. Dynamic cal is recommended.
    - Default: Acc and Mag Cal enabled.
    - Motion Tracking in relatively stable magnetic field and sufficient tremor: Enable Gyro Cal.
    - VR applications: Only enable Accel Cal to prevent jumps.
    - Calibration Steps: see [1], p. 39
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - enable_Accel_Cal: Boolean to enable (true) or disable (false) accelerometer calibration
    - enable_Gyro_Cal: Boolean to enable (true) or disable (false) gyro calibration
    - enable_Mag_Cal: Boolean to enable (true) or disable (false) magnetometer calibration
    - enable_Planar_Accel_Cal: Boolean to enable (true) or disable (false) planar accelerometer calibration. Use the planar one if calibration is only in 2D possible (f.ex. robot)
    - enable_On_Table_Cal: Boolean to enable (true) or disable (false) on Table calibration
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t get_ME_Calibration_Config_IMU(sensor_meta *sensor)```

  - **Description:** Get ME calibration config, get enabled / disabled calibration routines.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t config_periodic_DCD_IMU(sensor_meta *sensor, bool enable_periodic_DCD)```

  - **Description:** Enable / disable the periodic DCD routines.
  - **Note:**
    - This command does not have a response! The function check_Command_Success() does not work!
    - This command does not inhibit the Save DCD command save_DCD_IMU().
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - enable_periodic_DCD: Boolean variable to enable (true) or disable (false) periodic DCD routine.
  - **Return Value:** status, 1 no error occurred, 0 error occurred

- ```c

  uint8_t check_Command_Success(sensor_meta *sensor, uint8_t status_command)```

  - **Description:** Checks if the executed command was successful via the command response.
  - **Note:** Only if the passed status is not an error, the function will be executed. This prevents unnecessary blocking.
  - **Parameter:**
    - *sensor: Pointer to corresponding sensor meta data
    - status_command: Status of executed command (f.ex. startup_status), only check success if status_command is not an error
  - **Return Value:** status, 1 no error occurred, 0 error occurred

## Usage of the library in the main.c file - Typical main() function

Example for two IMUs. Integrate the code at the appropriate place in the
autogenerated main.c file:

```c
// Sensor data
// More sensors: Add pointer for every sensor or implement the structs as as a
two-dimensional array
sensor_meta *sensor1 = &((sensor_meta){});
sensor_meta*sensor2 = &((sensor_meta){});

// Quaternions data
// More sensors: Add for every sensor own gloabl variables
// Sensor 1
loat quatI1, quatJ1, quatK1, quatReal1;
uint8_t accuracy1, stabilityClassifier1;
int radianAccuracy1;

// Sensor 2
float quatI2, quatJ2, quatK2, quatReal2;
uint8_t accuracy2;
int radianAccuracy2;

// error variables
volatile uint8_t startup_status;
uint16_t const startup_timeout = 100;
volatile uint16_t startup_timeout_counter = 0;

int main(void)
{
    // Init functions of microcontroller
    ...

    // Register sensors, writes pins in corresponding sensor struct
    // More sensors: Call this function for every sensor
    register_Sensor(sensor1, 1, CSN_PIN_S1, CSN_PORT_S1, INTN_PIN_S1, INTN_PORT_S1);
    register_Sensor(sensor2, 2, CSN_PIN_S2, CSN_PORT_S2, INTN_PIN_S2, INTN_PORT_S2);

    // Initialization of SPI and IMU, if error occurs the IMUs reset and restart again
    while(!startup_status)
    {
        startup_status = N_ERR;

        // Init Hardware (SPI, Timer, GPIOs)
        startup_status &= init_Hardware();

        // Init GPIOs IMU
        // More sensors: Call function init_GPIO_IMU(sensorx->ports_pins) for every sensor
        init_GPIO_IMU(sensor1);
        init_GPIO_IMU(sensor2);

        // Reset for initialization of SPI mode
        hardreset_IMUs();
        // Clear advertisement message after hardware reset
        startup_status &= clear_init_Message_IMU(sensor1);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor1, startup_status);
        // Check connection of IMU via Product ID Request
        startup_status &= check_Connection_IMU(sensor1);

        // More sensors: Repeat softresetDCD_IMU(sensorx), check_Command_Success(sensorx) and check_Connection_IMU(sensorx) for every sensor
        startup_status &= softresetDCD_IMU(sensor2);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor2, startup_status);
        // Check connection of IMU via Product ID Request
        startup_status &= check_Connection_IMU(sensor2);

        // Clear all reset flags so that unwanted resets can be tracked from now on.
        // More sensors: Repeat get_and_clear_Reset_Status(sensorx)
        get_and_clear_Reset_Status(sensor1);
        get_and_clear_Reset_Status(sensor2);

        // Enable desired reports
        // More sensors: call the corresponding mode enable function for every sensor
        startup_status &= enable_ARVR_stabilized_RotationVector(sensor1, 10);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor1, startup_status);
        startup_status &= enable_ARVR_stabilized_RotationVector(sensor2, 10);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor2, startup_status);
        startup_status &= enable_StabilityClassifier(sensor1, 10);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor1, startup_status);
        startup_status &= enable_TapDetector(sensor1, 10);
        // Check if the command was set successfully
        startup_status &= check_Command_Success(sensor1, startup_status);

        // Error handling
        ++startup_timeout_counter;
        if(startup_timeout_counter >= startup_timeout)
        {
            Error_Handler();
        }
    }

    // Infinite loop
    while (1)
    {
        // Check if data is available, store the data and print if possible
        // More sensors: Call if(data_available(sensorx) == true) for every sensor
        if (data_available(sensor1) == true)
        {
            quatI1 = get_Quat_I(sensor1);
            quatJ1 = get_Quat_J(sensor1);
            quatK1 = get_Quat_K(sensor1);
            quatReal1 = get_Quat_Real(sensor1);
            accuracy1 = get_Quat_Accuracy(sensor1);
            radianAccuracy1 = get_Quat_Radian_Accuracy(sensor1);
            stabilityClassifier1 = get_StabilityClassifier(sensor1);
        }

        if (data_available(sensor2) == true)
        {
            quatI2 = get_Quat_I(sensor2);
            quatJ2 = get_Quat_J(sensor2);
            quatK2 = get_Quat_K(sensor2);
            quatReal2 = get_Quat_Real(sensor2);
            accuracy2 = get_Quat_Accuracy(sensor2);
            radianAccuracy2 = get_Quat_Radian_Accuracy(sensor2);
        }

        // More sensors: Add here the next call if(data_available(sensorx) == true)

        // Other update functions (f.ex. update_TapDetector(sensorx))
        update_TapDetector(sensor1);
    }
}
```
