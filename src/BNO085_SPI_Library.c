/*
 * @file BNO085_SPI_Library.c
 * @brief SPI Interface and service routine functions for BNO085 via SHTP
 * protocol used by the manufacturer CEVA.
 */

// --- Includes -------------------------------------------------------
// Private includes
#include "BNO085_SPI_Library.h"

// --- Macros and data ------------------------------------------------
// Channels for communication via SHTP (cf. [1], p. 22)
typedef uint8_t byte;
const byte CHANNEL_COMMAND = 0;
const byte CHANNEL_EXECUTABLE = 1;
const byte CHANNEL_CONTROL = 2;
const byte CHANNEL_REPORTS = 3;
// wake and gyro reports not used, only for completeness
const byte CHANNEL_WAKE_REPORTS = 4;
const byte CHANNEL_GYRO = 5;

// Report ID Convention (cf. [2], p. 28 f., figure 32)
// All possible configurations of the BNO085
// These are used for low level communication with the sensor on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

// Reset Report ID (cf. [1], p. 23; [2], p. 55)
#define SHTP_RESET_COMMAND_RESPONSE 1
#define SHTP_RESET_COMMAND_REQUEST 1

// FRS Read/Write (cf. [2], p. 41 ff.)
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_FRS_WRITE_RESPONSE 0xF5
#define SHTP_REPORT_FRS_WRITE_DATA_REQUEST 0xF6
#define SHTP_REPORT_FRS_WRITE_REQUEST 0xF7

// Commands we want use (cf. [2], p. 44 f.; p. 47 ff.)
#define SENSOR_COMMAND_TARE 0x03
#define SENSOR_COMMAND_TARE_NOW 0x00
#define SENSOR_COMMAND_TARE_PERSIST 0x01
#define SENSOR_COMMAND_TARE_SET_REORIENTATION 0x02
#define SENSOR_COMMAND_TARE_AXIS_Z 0x04
#define SENSOR_COMMAND_TARE_AXIS_ALL 0x07
#define SENSOR_COMMAND_INITIALIZE 0x04
#define SENSOR_COMMAND_INITIALIZE_RESPONSE 0x04
#define SENSOR_COMMAND_INITIALIZE_RESPONSE_UNSOLICITED 0x84
#define SENSOR_COMMAND_SAVE_DCD 0x06
#define SENSOR_COMMAND_ME_CAL 0x07
#define SENSOR_COMMAND_ME_CAL_CONFIG 0x00
#define SENSOR_COMMAND_ME_CAL_GET 0x01
#define SENSOR_COMMAND_CONFIG_DCD 0x09
#define SENSOR_COMMAND_CLEAR_DCD_RESET 0x0B
#define SENSOR_COMMAND_CALIBRATION 0x0C  // Not implemented yet

// Feature reports we want use and can get reports from (cf. [2], p. 38 f., p.
// 71 f., p. 84 f.)
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_ARVR_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR 0x29
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13

// Reset of the executable channel, reset complete packet (cf. [1], p.23, figure
// 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

// Limit sensor reports to 128 bytes (RAM restriction microcontroller), for this
// application sufficient
#define MAX_PACKET_SIZE \
  128  // Max would be up to 32766 minus the header bytes (cf. [1], p. 22)
// Each packet has a header of 4 bytes (cf. [1], p. 22, figure 1-26)
#define HEADER_PACKET_SIZE 4

// There are 6 command channels.
#define CHANNEL_MAX_NUMBER 6

// Reset delay, keeps reset asserted this long (cf. [1], p. 47)
#define RESET_DELAY_US 2  // Minimum is 0.01 us, 2 us is enough

// Q values for quaternion calculation (cf. [1], p. 25, figure 1-32 and [2], p.
// 71 f.)
int16_t rotationVector_Q1 = 14;
int16_t rotationVectorAccuracy_Q1 =
    12;  // Heading accuracy estimate in radians. The Q point is 12.
int16_t accelerometer_Q1 = 8;  // ... the Q point is 8, see [2] pp. 66ff.

// Debug
bool debug_print = false;

// --- SPI interface --------------------------------------------------

// Helper functions to transmit and receive data via SPI
/**
 * @brief Transmit data byte and receive data byte but doesn't store returned
 * byte.
 * @note The corresponding HAL Receive only functions seems not to work
 * properly, therefore read rx_data byte and ignore it.
 * @param tx_data: Data byte for transmit
 */
void SPI_Transmit(uint8_t tx_data) {
  uint8_t rx_data;  // receive buffer
  HAL_SPI_TransmitReceive(&hspi, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Transmit data byte and receive data byte by returning the received
 * byte.
 * @note The corresponding HAL Receive only functions seems not to work
 * properly, therefore use tx dummy data if needed.
 * @param tx_data: Data byte for transmit
 * @return rx_data: Received data byte
 */
uint8_t SPI_TransmitReceive_Return_Byte(uint8_t tx_data) {
  uint8_t rx_data;  // receive buffer
  HAL_SPI_TransmitReceive(&hspi, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
  return rx_data;
}

// --- Private methods ------------------------------------------------

/**
 * @brief Set or reset the RSTN Pin.
 * @param state: Boolean variable to set (true) or reset (false) the RSTN Pin of
 * all sensors.
 */
static void rstn(ports_pins ports_pins_config, bool state) {
  HAL_GPIO_WritePin(ports_pins_config.RSTN_Port, ports_pins_config.RSTN_Pin,
                    state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Set or reset the CSN Pin.
 * @param ports_oins_config: Struct of ports and pins configuration of
 * corresponding sensor
 * @param state: Boolean variable to set (true) or reset (false) the CSN Pin of
 * the corresponding sensor
 */
static void csn(ports_pins ports_pins_config, bool state) {
  HAL_GPIO_WritePin(ports_pins_config.CSN_Port, ports_pins_config.CSN_Pin,
                    state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Resets all sensors connected to the RSTN Pin.
 */
static void reset_Hardware(ports_pins ports_pins_config) {
  rstn(ports_pins_config, false);
  // Delay for RESET_DELAY_US to ensure reset takes effect
  delay_Us(RESET_DELAY_US);
  rstn(ports_pins_config, true);
}

/**
 * @brief Checks if the sensor requires attention, checks if flag is set.
 * @note Blocking function, waits until corresponding INTN Pin is LOW or timeout
 * occurs.
 * @note Flags has to be cleared somewhere after usage.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return status: 1 no error occurred, 0 an error occurred
 */
static uint8_t wait_for_INTN(sensor_meta *sensor) {
  uint8_t status = N_ERR;
  uint32_t timeout_counter = 0;
  while (sensor->INTN_ready != 1) {
    // Read INTN Pin value, LOW means INTN_ready = 1;
    sensor->INTN_ready =
        (HAL_GPIO_ReadPin(sensor->ports_pins.INTN_Port,
                          sensor->ports_pins.INTN_Pin) == GPIO_PIN_RESET)
            ? GPIO_PIN_SET
            : GPIO_PIN_RESET;
    ++timeout_counter;
    // After a reset, at least 90 ms elapse until first assertion of INTN (cf.
    // [1], p. 47) If delay is delay_Us(1000) minimum timeout_counter > 90
    if (timeout_counter >= 500) {
      // Timeout error handling
      status = D_ERR;
      break;
    }
    delay_Us(1000);  // experimental value
  }

  return status;
}

/**
 * @brief Checks whether an INTN Pin has been set by one sensor and stores the
 * changes as flags.
 * @note Has to be called periodically (inside loop in main or functions within
 * the main loop).
 * @note Flags has to be cleared somewhere after usage.
 * @param *sensor: Pointer to corresponding sensor meta data
 */
static void check_INTN(sensor_meta *sensor) {
  sensor->INTN_ready =
      (HAL_GPIO_ReadPin(sensor->ports_pins.INTN_Port,
                        sensor->ports_pins.INTN_Pin) == GPIO_PIN_RESET)
          ? GPIO_PIN_SET
          : GPIO_PIN_RESET;
}

/**
 * @brief Prints the contents of the current shtp header (only when debug_print
 * enabled).
 * @param data: shtp_package struct of corresponding sensor
 */
static void print_Header(shtp_package data) {
  if (debug_print == true) {
    // Print the four byte header
    printf("Header:");
    for (uint8_t x = 0; x < 4; x++) {
      printf(" ");
      if (data.shtp_Header[x] < 0x10) {
        printf("0");
      }
      printf("%d", data.shtp_Header[x]);
    }
    printf("\n");
  }
}

/**
 * @brief Prints the contents of the current shtp header and data packets (only
 * when debug_print enabled).
 * @param data: shtp_package struct of corresponding sensor
 */
static void print_Data(shtp_package data) {
  if (debug_print == true) {
    // Calculate packet_Length, combine two bytes into single 16-bit integer
    // (bytes 0:1)
    uint16_t packet_Length =
        (uint16_t)data.shtp_Header[1] << 8 | data.shtp_Header[0];

    // Print the four byte header
    print_Header(data);

    uint8_t print_Length = packet_Length - 4;
    if (print_Length > 40) print_Length = 40;  // Artificial limit for printing

    // Print body (data)
    printf("Body:");
    for (uint8_t x = 0; x < print_Length; x++) {
      printf(" ");
      if (data.shtp_Data[x] < 0x10) {
        printf("0");
      }
      printf("%d", data.shtp_Data[x]);
    }
    printf("\n");

    // Check if it is a continued packet
    if (packet_Length & 1 << 15) {
      printf(" [Continued packet] \n");
      packet_Length &= ~(1 << 15);  // Clear the MSB
    }

    // Print length of packet
    printf("Length: ");
    printf("%d, ", packet_Length);

    // Print channel
    printf("Channel: ");
    if (data.shtp_Header[2] == 0)
      printf("Command");
    else if (data.shtp_Header[2] == 1)
      printf("Executable");
    else if (data.shtp_Header[2] == 2)
      printf("Control");
    else if (data.shtp_Header[2] == 3)
      printf("Sensor-report");
    else if (data.shtp_Header[2] == 4)
      printf("Wake-report");
    else if (data.shtp_Header[2] == 5)
      printf("Gyro-vector");
    else
      printf("%d", data.shtp_Header[2]);

    printf("\n");
  }
}

/**
 * @brief Converts data to float variable, given a fixed point and a Q-point.
 * Cf.: https://en.wikipedia.org/wiki/Q_(number_format)
 * @param fixpoint_Value: Raw quaternion / accuracy value
 * @param q_Point: Q-point of corresponding calculation
 */
static float fixpoint_to_float(uint16_t fixpoint_Value, uint8_t q_Point) {
  // Both arguments of powf are float. Therefore the uint16_t has to be casted.
  // Cast uint16_t fixpoint_Value to a signed int16_t to prevent data and
  // precision loss, then cast it to an float
  float fixpoint_Value_temp = (int16_t)fixpoint_Value;
  fixpoint_Value_temp *= powf(2, q_Point * -1);
  return (fixpoint_Value_temp);
}

/**
 * @brief Read the contents of the incoming packet into the corresponding
 * shtp_Data array.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return status: 1 no error occurred, 0 an error occurred
 */
static uint8_t receive_Data(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Notice: wait_for_INTN() == D_ERR would block, therefore check_INTN() is
  // periodically called
  if (sensor->INTN_ready != 1) {
    status = D_NOT;
    return status;
  }

  // Clear the INTN flag
  sensor->INTN_ready = 0;

  // Assert CSN
  csn(sensor->ports_pins, false);

  // Get the first four bytes, the packet header
  uint8_t tx_dummy_data = 0;
  uint8_t packet_LSB = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  uint8_t packet_MSB = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  uint8_t channel_Number = SPI_TransmitReceive_Return_Byte(tx_dummy_data);
  uint8_t sequence_Number = SPI_TransmitReceive_Return_Byte(tx_dummy_data);

  // Store the header info
  sensor->shtp_package.shtp_Header[0] = packet_LSB;
  sensor->shtp_package.shtp_Header[1] = packet_MSB;
  sensor->shtp_package.shtp_Header[2] = channel_Number;
  sensor->shtp_package.shtp_Header[3] = sequence_Number;

  // Calculate the number of data bytes in this packet
  uint16_t data_Length = (((uint16_t)packet_MSB) << 8) | ((uint16_t)packet_LSB);
  data_Length &=
      ~(1 << 15);  // Clear the MSB
                   // Notice: This bit indicates if this package is a
                   // continuation of the last. Not relevant for this
                   // application. Notice: If other reports should be used, this
                   // this bit must be considered and handled.

  // Check for empty packet
  if (data_Length == 0) {
    // Packet is empty
    print_Header(sensor->shtp_package);
    return D_NOT;  // No data available
  }

  data_Length -= 4;  // Remove the header bytes from the data count

  // Store 0xFF in a new dummyTx array
  uint8_t dummyTxVal = 0xFF;  // 0xFF is often uses as dummy data, SPI device
                              // responds by sending a byte

  // Read incoming data into the shtp_Data array
  for (uint16_t data_Place = 0; data_Place < data_Length; data_Place++) {
    uint8_t data_in = SPI_TransmitReceive_Return_Byte(dummyTxVal);

    if (data_Place < MAX_PACKET_SIZE)  // BNO085 can respond with up to 270
                                       // bytes, avoid overflow
      sensor->shtp_package.shtp_Data[data_Place] =
          data_in;  // Store data into the shtp_Data array
  }

  // Deassert CSN
  csn(sensor->ports_pins, true);

  // Print data
  print_Data(sensor->shtp_package);

  // Quickly check for reset complete packet. No need for a separate parser.
  // This function is also called after softreset_IMU(), so we need to catch
  // this packet here otherwise we need to check for the reset packet in
  // multiple places.
  if (sensor->shtp_package.shtp_Header[2] == CHANNEL_EXECUTABLE &&
      sensor->shtp_package.shtp_Data[0] == EXECUTABLE_RESET_COMPLETE) {
    sensor->has_reset = true;
  }

  return status;
}

/**
 * @brief Given the data packet, send the header then the data.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param channel_Number: The command channel
 * @param data_Length: Length of data packet (without header)
 * @return status: 1 no error occurred, 0 an error occurred
 */
static uint8_t send_Data(sensor_meta *sensor, uint8_t channel_Number,
                         uint8_t data_Length) {
  uint8_t status = N_ERR;

  uint8_t packet_Length = data_Length + 4;  // Add four bytes for the header

  // Notice: send_Data is never called within the main loop, blocking does not
  // affect performance during runtime. Notice: Therefore check_INTN() would not
  // guarantee recognition of the pin assertion.
  if (wait_for_INTN(sensor) == D_ERR) {
    status = D_ERR;
    return status;
  }

  // Clear the INTN flag
  sensor->INTN_ready = 0;

  // Assert CSN
  csn(sensor->ports_pins, false);

  // create temporary transmit buffers
  uint8_t temp_tx_length_LSB = packet_Length & 0xFF;  // Packet length LSB
  uint8_t temp_tx_length_MSB = packet_Length >> 8;    // Packet length MSB
  uint8_t temp_tx_channel_number = channel_Number;    // Channel number
  uint8_t temp_tx_sequence_number =
      sensor->shtp_package
          .sequence_Number[channel_Number]++;  // Send the sequence number,
                                               // increments with each packet
                                               // sent, different counter for
                                               // each channel

  // Send the 4 byte packet header
  SPI_Transmit(temp_tx_length_LSB);       // Packet length LSB
  SPI_Transmit(temp_tx_length_MSB);       // Packet length MSB
  SPI_Transmit(temp_tx_channel_number);   // Channel number
  SPI_Transmit(temp_tx_sequence_number);  // Send the sequence number,
                                          // increments with each packet sent,
                                          // different counter for each channel

  // Send the user's data packet
  for (uint8_t i = 0; i < data_Length; i++) {
    // Temporary buffer
    uint8_t temp_shtp_Data_entry = sensor->shtp_package.shtp_Data[i];
    // Send data
    SPI_Transmit(temp_shtp_Data_entry);
  }

  // Deassert CSN
  csn(sensor->ports_pins, true);

  return status;
}

/**
 * @brief Given the command, send the report command request.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param command_number: Command number which can be sent, list cf. [2], p. 44
 * f.
 * @return status: 1 no error occurred, 0 an error occurred
 */
// Structure of request packet (cf. [2], p. 43 f.):
// shtp_Data[0]: Report ID
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Command
// shtp_Data[3]: P0
// shtp_Data[4]: P1
// shtp_Data[5]: P2
// shtp_Data[6]: P3
// shtp_Data[7]: P4
// shtp_Data[8]: P5
// shtp_Data[9]: P6
// shtp_Data[10]: P7
// shtp_Data[11]: P8

static uint8_t send_Command(sensor_meta *sensor, uint8_t command_number) {
  uint8_t status = N_ERR;
  // Fill shtp_data with the values (cf. [2], p. 64, figure 78)
  sensor->shtp_package.shtp_Data[0] =
      SHTP_REPORT_COMMAND_REQUEST;  // Set feature command
  sensor->shtp_package.shtp_Data[1] =
      sensor->shtp_package
          .command_Sequence_Number++;  // Commands have a sequence number as
                                       // well. These are inside command packet,
                                       // the header uses its own sequence
                                       // number per channel
  sensor->shtp_package.shtp_Data[2] = command_number;  // Feature flags
  // P0-P8 are set in the respective command.

  status &= send_Data(sensor, CHANNEL_CONTROL,
                      12);  // Send command, 12 bytes on CHANNEL_CONTROL

  return status;
}

/**
 * @brief Given a sensor's report ID, this tells the corresponding BNO085 to
 * begin reporting the values.
 * @note Can be called multiple times to enable different reports at the same
 * time.
 * @note This function is also called in enable_RotationVector and
 * enable_GameRotationVector.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param report_ID: Report ID of report which should be used
 * @param time_between_reports: Desired time between reports in ms, linked to
 * the operating frequency of the sensor.
 * @param specific_Config: Word (uint32_t), optional. Useful for personal
 * activity classifier. No specific command: Set specific_Config to 0.
 * @return status: 1 no error occurred, 0 an error occurred
 */
static uint8_t set_FeatureCommand(sensor_meta *sensor, uint8_t report_ID,
                                  uint16_t time_between_reports,
                                  uint32_t specific_Config) {
  uint8_t status = N_ERR;

  // Convert milliseconds in microseconds
  uint32_t micros_between_reports = (uint32_t)time_between_reports * 1000L;

  // Fill shtp_data with the values (cf. [2], p. 64, figure 78)
  sensor->shtp_package.shtp_Data[0] =
      SHTP_REPORT_SET_FEATURE_COMMAND;            // Set feature command
  sensor->shtp_package.shtp_Data[1] = report_ID;  // Feature Report ID
  sensor->shtp_package.shtp_Data[2] = 0;          // Feature flags
  sensor->shtp_package.shtp_Data[3] = 0;          // Change sensitivity (LSB)
  sensor->shtp_package.shtp_Data[4] = 0;          // Change sensitivity (MSB)
  sensor->shtp_package.shtp_Data[5] =
      (micros_between_reports >> 0) &
      0xFF;  // Report interval (LSB) in microseconds. F.ex. 0x7A120 = 500ms
  sensor->shtp_package.shtp_Data[6] =
      (micros_between_reports >> 8) & 0xFF;  // Report interval
  sensor->shtp_package.shtp_Data[7] =
      (micros_between_reports >> 16) & 0xFF;  // Report interval
  sensor->shtp_package.shtp_Data[8] =
      (micros_between_reports >> 24) & 0xFF;  // Report interval (MSB)
  sensor->shtp_package.shtp_Data[9] = 0;      // Batch Interval (LSB)
  sensor->shtp_package.shtp_Data[10] = 0;     // Batch Interval
  sensor->shtp_package.shtp_Data[11] = 0;     // Batch Interval
  sensor->shtp_package.shtp_Data[12] = 0;     // Batch Interval (MSB)
  sensor->shtp_package.shtp_Data[13] =
      (specific_Config >> 0) & 0xFF;  // Sensor-specific config (LSB)
  sensor->shtp_package.shtp_Data[14] =
      (specific_Config >> 8) & 0xFF;  // Sensor-specific config
  sensor->shtp_package.shtp_Data[15] =
      (specific_Config >> 16) & 0xFF;  // Sensor-specific config
  sensor->shtp_package.shtp_Data[16] =
      (specific_Config >> 24) & 0xFF;  // Sensor-specific config (MSB)

  // Transmit packet on channel 2 (control channel), 17 bytes
  status &= send_Data(sensor, CHANNEL_CONTROL, 17);

  return status;
}

/**
 * @brief Pulls the data from the input report and stores the data in the
 * corresponding struct of the sensor.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Returns the report ID or 0 if the report ID is not handled
 */
// Structure of responded packet (cf. [2], p. 71 f.):
// shtp_Header[0:3]: 4 byte header
// shtp_Data[0:4]: 5 byte timestamp of microsecond clicks since reading was
// taken shtp_Data[5 + 0]: Feature report ID shtp_Data[5 + 1]: Sequence number
// shtp_Data[5 + 2]: Status
// shtp_Data[5 + 3]: Delay
// shtp_Data[5 + 4 : 5 + 5]: unit quaternion i component
// shtp_Data[5 + 6 : 5 + 7]: unit quaternion j component
// shtp_Data[5 + 8 : 5 + 9]: unit quaternion k component
// shtp_Data[5 + 10 : 5 + 11]: unit quaternion real component
// shtp_Data[5 + 12 : 5 + 13]: Accuracy estimate (only Rotation Vector)

uint16_t parse_InputReport(sensor_meta *sensor) {
  if (sensor == NULL) {
    return 0;
  }

  // Calculate the number of data bytes in this packet
  int16_t data_Length = ((uint16_t)sensor->shtp_package.shtp_Header[1] << 8 |
                         sensor->shtp_package.shtp_Header[0]);
  data_Length &= ~(1 << 15);  // Clear the MSbit.
  // Notice: This bit indicates if this package is a continuation of the last.
  // Not relevant for this application. Notice: If other reports should be used,
  // this this bit must be considered and handled.

  data_Length -= 4;  // Remove the header bytes from the data count

  // Store values
  // Get status bits (byte 7), mask out all but the two least significant bits
  // of the accessed byte (cf. [2], p. 42)
  uint8_t status_report = sensor->shtp_package.shtp_Data[5 + 2] & 0x03;
  // Get raw unit quaternion i component, combine two bytes in one uint16_t
  // value (bytes 9:10)
  uint16_t data1 = (uint16_t)sensor->shtp_package.shtp_Data[5 + 5] << 8 |
                   sensor->shtp_package.shtp_Data[5 + 4];
  // Get raw unit quaternion j component, combine two bytes in one uint16_t
  // value (bytes 11:12)
  uint16_t data2 = (uint16_t)sensor->shtp_package.shtp_Data[5 + 7] << 8 |
                   sensor->shtp_package.shtp_Data[5 + 6];
  // Get raw unit quaternion k component, combine two bytes in one uint16_t
  // value (bytes 13:14)
  uint16_t data3 = (uint16_t)sensor->shtp_package.shtp_Data[5 + 9] << 8 |
                   sensor->shtp_package.shtp_Data[5 + 8];
  uint16_t data4 = 0;
  uint16_t data5 = 0;  // We would need to change this to uin32_t to capture
                       // time stamp value on Raw Accel/Gyro/Mag reports

  if (data_Length - 5 > 9) {
    // Get raw unit quaternion real component, store two bytes in one uint16_t
    // value
    data4 = (uint16_t)sensor->shtp_package.shtp_Data[5 + 11] << 8 |
            sensor->shtp_package.shtp_Data[5 + 10];
  }
  if (data_Length - 5 > 11) {
    // Get raw accuracy estimate, store two bytes in one uint16_t value (only
    // for Rotation Vector and ARVR-Stabilized Rotation Vector)
    data5 = (uint16_t)sensor->shtp_package.shtp_Data[5 + 13] << 8 |
            sensor->shtp_package.shtp_Data[5 + 12];
  }

  // Store these generic values to their proper global variable / variable of
  // sensor struct
  if (sensor->shtp_package.shtp_Data[5] == SENSOR_REPORTID_ACCELEROMETER) {
    sensor->accelerometer_data.accelerometer_Accuracy = status_report;
    sensor->accelerometer_data.raw_Accel_X = data1;
    sensor->accelerometer_data.raw_Accel_Y = data2;
    sensor->accelerometer_data.raw_Accel_Z = data3;
  } else if (sensor->shtp_package.shtp_Data[5] ==
             SENSOR_REPORTID_LINEAR_ACCELERATION) {
    sensor->linear_acceleration_data.accelerometer_Accuracy = status_report;
    sensor->linear_acceleration_data.raw_Accel_X = data1;
    sensor->linear_acceleration_data.raw_Accel_Y = data2;
    sensor->linear_acceleration_data.raw_Accel_Z = data3;
  } else if (sensor->shtp_package.shtp_Data[5] == SENSOR_REPORTID_GRAVITY) {
    sensor->gravity_data.accelerometer_Accuracy = status_report;
    sensor->gravity_data.raw_Accel_X = data1;
    sensor->gravity_data.raw_Accel_Y = data2;
    sensor->gravity_data.raw_Accel_Z = data3;
  } else if (sensor->shtp_package.shtp_Data[5] ==
                 SENSOR_REPORTID_ROTATION_VECTOR ||
             sensor->shtp_package.shtp_Data[5] ==
                 SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
             sensor->shtp_package.shtp_Data[5] ==
                 SENSOR_REPORTID_ARVR_ROTATION_VECTOR ||
             sensor->shtp_package.shtp_Data[5] ==
                 SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR) {
    sensor->quaternions.quat_Accuracy = status_report;
    sensor->quaternions.raw_Quat_I = data1;
    sensor->quaternions.raw_Quat_J = data2;
    sensor->quaternions.raw_Quat_K = data3;
    sensor->quaternions.raw_Quat_Real = data4;

    // Only available on Rotation Vector and ARVR-Stabilized Rotation Vector
    sensor->quaternions.raw_Quat_Radian_Accuracy = data5;
  } else if (sensor->shtp_package.shtp_Data[5] ==
             SENSOR_REPORTID_STABILITY_CLASSIFIER) {
    sensor->additional_data.stability_Classifier =
        sensor->shtp_package.shtp_Data[5 + 4];  // Byte 4 only
  } else if (sensor->shtp_package.shtp_Data[5] ==
             SENSOR_REPORTID_TAP_DETECTOR) {
    sensor->additional_data.raw_tap_Detector =
        sensor->shtp_package.shtp_Data[5 + 4];  // Byte 4 only
  } else if (sensor->shtp_package.shtp_Data[5] ==
             SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO085 responds with this report to command requests. It's up to use
    // to remember which command we issued. If needed: store shtp_Data[5 + 2]
    // which is the command byte of the response uint8_t command =
    // sensor->shtp_package.shtp_Data[5 + 2];
  } else {
    // This sensor report ID is unhandled.
    // See reference manual (cf. [2]) to add additional feature reports as
    // needed.
    return 0;
  }

  // Notice: Additional feature reports may be strung together. Not relevant for
  // this application. If needed: Parse them all.

  return sensor->shtp_package.shtp_Data[5];
}

/**
 * @brief Pulls the data from the command response report and stores the data
 * (if necessary) in the corresponding struct of the sensor.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Returns the report ID or 0 if the report ID is not handled
 */
// Structure of responded packet (cf. [2], p. 71 f.):
// shtp_Header[0:3]: 4 byte header
// shtp_Data[0]: Report ID
// shtp_Data[1]: Sequence number
// shtp_Data[2]: Command
// shtp_Data[3]: Command Sequence Number
// shtp_Data[4]: Response Sequence Number
// shtp_Data[5 + 0]: R0
// shtp_Data[5 + 1]: R1
// shtp_Data[5 + 2]: R2
// shtp_Data[5 + 3]: R3
// shtp_Data[5 + 4]: R4
// shtp_Data[5 + 5]: R5
// shtp_Data[5 + 6]: R6
// shtp_Data[5 + 7]: R7
// shtp_Data[5 + 8]: R8
// shtp_Data[5 + 9]: R9
// shtp_Data[5 + 10]: R10
static uint16_t parse_CommandReport(sensor_meta *sensor) {
  // Command Responses
  if (sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO085 responds with this report to command requests.
    // Store shtp_Data[2] which is the command byte of the response
    uint8_t command = sensor->shtp_package.shtp_Data[2];
    // Store shtp_Data[5] which is in some responses the status / success byte
    // of the command
    uint8_t status_command = sensor->shtp_package.shtp_Data[5];

    // Response if Initialize Command or Clear DCD Reset Command
    if (command == SENSOR_COMMAND_INITIALIZE_RESPONSE ||
        command == SENSOR_COMMAND_INITIALIZE_RESPONSE_UNSOLICITED ||
        command == SENSOR_COMMAND_SAVE_DCD ||
        command == SENSOR_COMMAND_ME_CAL) {
      if (status_command == 0) {
        // Success
        return sensor->shtp_package.shtp_Data[0];
      }
      return D_ERR;
    } else {  // Add here another else if, if you implement Simple Calibration
              // Commands 0x0C
      // This sensor report ID is unhandled.
      return D_ERR;
    }

    // Product ID Response
  } else if (sensor->shtp_package.shtp_Data[0] ==
             SHTP_REPORT_PRODUCT_ID_RESPONSE) {
    return sensor->shtp_package.shtp_Data[0];
    // Get Feature Response
  } else if (sensor->shtp_package.shtp_Data[0] ==
             SHTP_REPORT_GET_FEATURE_RESPONSE) {
    return sensor->shtp_package.shtp_Data[0];
    // FRS Read Response
  } else if (sensor->shtp_package.shtp_Data[0] ==
             SHTP_REPORT_FRS_READ_RESPONSE) {
    return sensor->shtp_package.shtp_Data[0];
    // FRS Write Response
  } else if (sensor->shtp_package.shtp_Data[0] ==
             SHTP_REPORT_FRS_WRITE_RESPONSE) {
    return sensor->shtp_package.shtp_Data[0];
  } else {
    // This sensor report ID is unhandled.
    // See [2] to add additional feature reports as needed
  }

  // Notice: Additional feature reports may be strung together. Not relevant for
  // this application. If needed: Parse them all.

  return D_ERR;
}

/**
 * @brief Pulls the data from the executable response report and stores the data
 * (if necessary) in the corresponding struct of the sensor.
 * @note Assignment: 1 = reset complete, other values reserved
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Returns the report ID or 0 if the report ID is not handled
 */
// Structure of responded packet (cf. [1], p. 23):
// shtp_Data[0]: Response ID
static uint16_t parse_ExecutableReport(sensor_meta *sensor) {
  if (sensor->shtp_package.shtp_Data[0] == SHTP_RESET_COMMAND_RESPONSE) {
    // The BNO085 responds with this report to command request
    sensor->has_reset = true;
    return sensor->shtp_package.shtp_Data[0];
  } else {
    // This sensor report ID is unhandled.
  }

  return 0;
}

/*
 * Calls receive_Data(), decides between command report or channel report and
 * selects the corresponding parser.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Returns the report ID or D_ERR / 0 if the report ID is not handled
 */
static uint16_t get_Readings(sensor_meta *sensor) {
  // Check if data available
  // Notice: If reset, read reset message (if channel 1) or unsolicited
  // initialization message (if channel 2)
  if (receive_Data(sensor) == N_ERR) {
    // Check to see if this packet is a sensor reporting its data to us
    if (sensor->shtp_package.shtp_Header[2] == CHANNEL_REPORTS &&
        sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_BASE_TIMESTAMP) {
      return parse_InputReport(
          sensor);  // This will update the corresponding variables depending on
                    // which feature report is found
    } else if (sensor->shtp_package.shtp_Header[2] == CHANNEL_CONTROL) {
      return parse_CommandReport(
          sensor);  // This will update responses to commands (f.ex.
                    // softresetDCD_IMU(), calibration status, etc.) via the
                    // control channel
    } else if (sensor->shtp_package.shtp_Header[2] == CHANNEL_EXECUTABLE) {
      return parse_ExecutableReport(
          sensor);  // This will update responses to commands (f.ex.
                    // softreset_IMU()) via the executable channel
    }
  }
  return D_ERR;
}

// --- Public methods -------------------------------------------------

/*
 * Register the corresponding sensor, store the sensor number, pins and ports in
 * the corresponding sensor struct.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param sensor_number: Manually assigned number of sensor, identifier of
 * sensor
 * @param sensor_CSN_Pin: Desired CSN Pin of sensor
 * @param sensor_CSN_Port: Pointer to desired CSN Port of sensor
 * @param sensor_INTN_Pin: Desired INTN Pin of sensor
 * @param sensor_INTN_Port: Pointer to desired INTN Port of sensor
 */
void register_Sensor(sensor_meta *sensor, uint8_t sensor_number,
                     uint16_t sensor_CSN_Pin, GPIO_TypeDef *sensor_CSN_Port,
                     uint16_t sensor_INTN_Pin, GPIO_TypeDef *sensor_INTN_Port,
                     uint16_t sensor_RSTN_Pin, GPIO_TypeDef *sensor_RSTN_Port) {
  // Fill addressed sensor struct of pins and ports
  sensor->ports_pins.CSN_Pin = sensor_CSN_Pin;
  sensor->ports_pins.CSN_Port = sensor_CSN_Port;
  sensor->ports_pins.INTN_Pin = sensor_INTN_Pin;
  sensor->ports_pins.INTN_Port = sensor_INTN_Port;
  sensor->ports_pins.RSTN_Pin = sensor_RSTN_Pin;
  sensor->ports_pins.RSTN_Port = sensor_RSTN_Port;
  sensor->number = sensor_number;
}

/**
 * @brief Reads any init message and throws it away.
 * @note The advertisement message is not needed.
 * @note Could also be used to delete a unsolicited timestamp response.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t clear_init_Message_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Deassert CSN pin -> high
  csn(sensor->ports_pins, true);

  // Read init messages (cf. [1], p. 43)
  // Read initial SHTP advertisement packet
  status &= wait_for_INTN(sensor);
  status &= receive_Data(sensor);

  if (status == N_ERR) {
    // Set reset flag
    // Notice: This flag is set here because the hardware reset reset_IMUs()
    // cannot set flags (the function resets all IMUs). This function must be
    // called after reset_IMUs(). The flag can be set here accordingly.
    sensor->has_reset = true;
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Hardware reset of IMUs (one reset line for all IMUs).
 * @note: Also necessary for booting in SPI mode.
 */
void hardreset_IMU(sensor_meta *sensor) {
  // Prepare BNO085 to boot in SPI mode
  // Reset sensor
  reset_Hardware(sensor->ports_pins);
}

/**
 * @brief Sends reset command. softreset_IMU() does not clear DCD (Dynamic
 * Calibration Data).
 * @note Software reset can be used instead of hardware reset reset_IMUs() to
 * keep DCD (cf. [1], p. 23; p. 39).
 * @note If DCD should also be reset, then use hardware reset reset_IMUs() or
 * softresetDCD_IMU().
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t softreset_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  sensor->shtp_package.shtp_Data[0] =
      SHTP_RESET_COMMAND_REQUEST;  // Software reset command (cf. [1], p. 23)

  // Send reset command via executable channel
  status &= send_Data(sensor, CHANNEL_EXECUTABLE,
                      1);  // Transmit packet on channel 1, 1 byte

  // Delete advertisement message
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    // Set reset flag
    sensor->has_reset = true;
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Sends reset command. softresetDCD_IMU() does clear DCD (Dynamic
 * Calibration Data).
 * @note Software reset can be used instead of hardware reset reset_IMUs() (cf.
 * [2], p. 55).
 * @note If DCD should not be reset, then use softreset_IMU().
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 55):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Clear DCD and Reset 0x0B
// shtp_Data[3]: P0 Reserved
// shtp_Data[4]: P1 Reserved
// shtp_Data[5]: P2 Reserved
// shtp_Data[6]: P3 Reserved
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t softresetDCD_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 55, figure 62)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0;
  }

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_CLEAR_DCD_RESET);

  // Delete advertisement message
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    // Set reset flag
    sensor->has_reset = true;
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Indicates a received reset complete packet. Once it's been read, the
 * state will reset to false until another reset complete packet is found.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return true if reset, false if not
 */
bool get_and_clear_Reset_Status(sensor_meta *sensor) {
  if (sensor->has_reset) {
    sensor->has_reset = false;
    return true;
  }
  return false;
}

/**
 * @brief Enables the report Accelerometer and sets the desired report delay
 * (frequency).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_Accelerometer(sensor_meta *sensor,
                             uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->accelerometer_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_ACCELEROMETER,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report Linear Acceleration and sets the desired report
 * delay (frequency).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_LinearAcceleration(sensor_meta *sensor,
                                  uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->linear_acceleration_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_LINEAR_ACCELERATION,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report Gravity and sets the desired report delay
 * (frequency).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_Gravity(sensor_meta *sensor, uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->gravity_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_GRAVITY,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report Rotation Vector and sets the desired report delay
 * (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_RotationVector(sensor_meta *sensor,
                              uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->rotation_vector_mode = SENSOR_REPORTID_ROTATION_VECTOR;
  sensor->rotation_vector_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_ROTATION_VECTOR,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report ARVR-Stabilized Rotation Vector and sets the
 * desired report delay (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_ARVR_stabilized_RotationVector(sensor_meta *sensor,
                                              uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->rotation_vector_mode = SENSOR_REPORTID_ARVR_ROTATION_VECTOR;
  sensor->rotation_vector_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_ARVR_ROTATION_VECTOR,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report Game Rotation Vector and sets the desired report
 * delay (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_GameRotationVector(sensor_meta *sensor,
                                  uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->rotation_vector_mode = SENSOR_REPORTID_GAME_ROTATION_VECTOR;
  sensor->rotation_vector_report_frequency = time_between_reports;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_GAME_ROTATION_VECTOR,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report ARVR-Stabilized Game Rotation Vector and sets the
 * desired report delay (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_ARVR_stabilized_GameRotationVector(
    sensor_meta *sensor, uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  sensor->rotation_vector_mode = SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR;
  sensor->rotation_vector_report_frequency = time_between_reports;
  status &=
      set_FeatureCommand(sensor, SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR,
                         time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report StabilityClassifier and sets the desired report
 * delay (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @note possible classifiers: 0 = unknown, 1 = on table, 2 = stationary, 3 =
 * stable, 4 = motion (cf. [2], p. 77 f.)
 * @note classifier 3 only available if gyroscope calibration enabled
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_StabilityClassifier(sensor_meta *sensor,
                                   uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_STABILITY_CLASSIFIER,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Enables the report Tap Detector and sets the desired report delay
 * (frequency).
 * @note Calls set_FeatureCommand with no specific_Config which sends the config
 * package.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param time_between_reports: Desired time in ms between two reports
 * @return status: 1 no error occurred, 0 an error occurred
 */
uint8_t enable_TapDetector(sensor_meta *sensor, uint16_t time_between_reports) {
  uint8_t status = N_ERR;
  status &= set_FeatureCommand(sensor, SENSOR_REPORTID_TAP_DETECTOR,
                               time_between_reports, 0);
  return status;
}

/**
 * @brief Executes the sensor routine (call it periodically / as often as
 * possible), updates variables if possible.
 * @note Calls get_Readings to store received packages.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: True if new readings available, false if not
 */
bool data_available(sensor_meta *sensor) {
  // check periodically INTN Pins
  check_INTN(sensor);
  return (get_Readings(sensor) != 0);
}

/**
 * Calculate the acceleration component X with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: X component as a float
 */
float get_Accelerometer_X(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->accelerometer_data.raw_Accel_X,
                              accelerometer_Q1);
  sensor->accelerometer_data.Accel_X = a;
  return a;
}

/**
 * Calculate the acceleration component Y with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Y component as a float
 */
float get_Accelerometer_Y(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->accelerometer_data.raw_Accel_Y,
                              accelerometer_Q1);
  sensor->accelerometer_data.Accel_Y = a;
  return a;
}

/**
 * Calculate the acceleration component Z with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Z component as a float.
 */
float get_Accelerometer_Z(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->accelerometer_data.raw_Accel_Z,
                              accelerometer_Q1);
  sensor->accelerometer_data.Accel_Z = a;
  return a;
}

/**
 * @brief Return the acceleration accuracy.
 * @note: Assignment: 0 = Unreliable, 1 = Accuracy Low, 2 = Accuracy Medium, 3 =
 * Accuracy High
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Acceleration accuracy
 */
uint8_t get_Accelerometer_Accuracy(sensor_meta *sensor) {
  return (sensor->accelerometer_data.accelerometer_Accuracy);
}

/**
 * Calculate the acceleration component X with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: X component as a float
 */
float get_LinearAcceleration_X(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_X,
                              accelerometer_Q1);
  sensor->linear_acceleration_data.Accel_X = a;
  return a;
}

/**
 * Calculate the acceleration component Y with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Y component as a float
 */
float get_LinearAcceleration_Y(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_Y,
                              accelerometer_Q1);
  sensor->linear_acceleration_data.Accel_Y = a;
  return a;
}

/**
 * Calculate the acceleration component Z with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Z component as a float
 */
float get_LinearAcceleration_Z(sensor_meta *sensor) {
  float a = fixpoint_to_float(sensor->linear_acceleration_data.raw_Accel_Z,
                              accelerometer_Q1);
  sensor->linear_acceleration_data.Accel_Z = a;
  return a;
}

/**
 * @brief Return the acceleration accuracy.
 * @note: Assignment: 0 = Unreliable, 1 = Accuracy Low, 2 = Accuracy Medium, 3 =
 * Accuracy High
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Acceleration accuracy
 */
uint8_t get_LinearAcceleration_Accuracy(sensor_meta *sensor) {
  return (sensor->linear_acceleration_data.accelerometer_Accuracy);
}

/**
 * Calculate the acceleration component X with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: X component as a float
 */
float get_Gravity_X(sensor_meta *sensor) {
  float a =
      fixpoint_to_float(sensor->gravity_data.raw_Accel_X, accelerometer_Q1);
  sensor->gravity_data.Accel_X = a;
  return a;
}

/**
 * Calculate the acceleration component Y with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Y component as a float
 */
float get_Gravity_Y(sensor_meta *sensor) {
  float a =
      fixpoint_to_float(sensor->gravity_data.raw_Accel_Y, accelerometer_Q1);
  sensor->gravity_data.Accel_Y = a;
  return a;
}

/**
 * Calculate the acceleration component Z with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Z component as a float
 */
float get_Gravity_Z(sensor_meta *sensor) {
  float a =
      fixpoint_to_float(sensor->gravity_data.raw_Accel_Z, accelerometer_Q1);
  sensor->gravity_data.Accel_Z = a;
  return a;
}

/**
 * @brief Return the acceleration accuracy.
 * @note: Assignment: 0 = Unreliable, 1 = Accuracy Low, 2 = Accuracy Medium, 3 =
 * Accuracy High
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Acceleration accuracy
 */
uint8_t get_Gravity_Accuracy(sensor_meta *sensor) {
  return (sensor->gravity_data.accelerometer_Accuracy);
}

/**
 * @brief Calculate the unit quaternion i component with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return quat: Unit quaternion component i
 */
float get_Quat_I(sensor_meta *sensor) {
  float quat =
      fixpoint_to_float(sensor->quaternions.raw_Quat_I, rotationVector_Q1);
  // store the quaternion component in sensor struct
  sensor->quaternions.Quat_I = quat;
  return (quat);
}

/**
 * @brief Calculate the unit quaternion j component with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return quat: Unit quaternion component j
 */
float get_Quat_J(sensor_meta *sensor) {
  float quat =
      fixpoint_to_float(sensor->quaternions.raw_Quat_J, rotationVector_Q1);
  // store the quaternion component in sensor struct
  sensor->quaternions.Quat_J = quat;
  return (quat);
}

/**
 * @brief Calculate the unit quaternion k component with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return quat: Unit quaternion component k
 */
float get_Quat_K(sensor_meta *sensor) {
  float quat =
      fixpoint_to_float(sensor->quaternions.raw_Quat_K, rotationVector_Q1);
  // store the quaternion component in sensor struct
  sensor->quaternions.Quat_K = quat;
  return (quat);
}

/**
 * Calculate the unit quaternion real component with the specific Q point.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return quat: Unit quaternion component real
 */
float get_Quat_Real(sensor_meta *sensor) {
  float quat =
      fixpoint_to_float(sensor->quaternions.raw_Quat_Real, rotationVector_Q1);
  // store the quaternion component in sensor struct
  sensor->quaternions.Quat_Real = quat;
  return (quat);
}

/**
 * @brief Calculate the estimated rotation vector accuracy in radian with the
 * specific Q point (only for report Rotation Vector and ARVR-Stabilized
 * Rotation Vector).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return acc: Estimated (ARVR-Stabilized) Rotation Vector accuracy in radian
 */
float get_Quat_Radian_Accuracy(sensor_meta *sensor) {
  float acc = fixpoint_to_float(sensor->quaternions.raw_Quat_Radian_Accuracy,
                                rotationVectorAccuracy_Q1);
  // store the quaternion component in sensor struct
  sensor->quaternions.Quat_Radian_Accuracy = acc;
  return (acc);
}

/**
 * @brief Return the (ARVR-Stabilized / Game) Rotation Vector accuracy.
 * @note: Assignment: 0 = Unreliable, 1 = Accuracy Low, 2 = Accuracy Medium, 3 =
 * Accuracy High (cf. [1], p. 38)
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: (ARVR-Stabilized / Game) Rotation Vector accuracy
 */
uint8_t get_Quat_Accuracy(sensor_meta *sensor) {
  return (sensor->quaternions.quat_Accuracy);
}

/**
 * @brief Return the Stability Classifier.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: Stability Classifier
 */
uint8_t get_StabilityClassifier(sensor_meta *sensor) {
  return (sensor->additional_data.stability_Classifier);
}

/**
 * @brief Update the Tap Detector, checks if the tap is a single or a double
 * tap, sets the specific flag and counts the taps.
 * @note Taps in all spatial directions are classified as taps in this case (cf.
 * [2], p. 75).
 * @param *sensor: Pointer to corresponding sensor meta data
 */
void update_TapDetector(sensor_meta *sensor) {
  uint8_t raw_tap_Detector = sensor->additional_data.raw_tap_Detector;
  uint8_t bit_mask_tap = 0b00111111;
  uint8_t bit_mask_double_tap = 0b01000000;
  if ((raw_tap_Detector & bit_mask_tap) != 0) {
    // Check if the tap was a double tap (check double tap flag)
    if ((raw_tap_Detector & bit_mask_double_tap) != 0) {
      ++sensor->additional_data.counter_double_tap;
      sensor->additional_data.has_double_tap = true;
    } else {
      ++sensor->additional_data.counter_single_tap;
      sensor->additional_data.has_single_tap = true;
    }
  }
  // Clear raw_tap_Detector value
  sensor->additional_data.raw_tap_Detector = 0;
}

/**
 * @brief Gets the Product ID via Product ID Request, waits for the response and
 * stores data.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t get_ProductID(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Product ID request (cf. [1], p. 23 f.)
  sensor->shtp_package.shtp_Data[0] =
      SHTP_REPORT_PRODUCT_ID_REQUEST;  // Request the product ID and reset info
  sensor->shtp_package.shtp_Data[1] = 0;  // Reserved

  // Transmit packet on channel 2, 2 bytes
  status &= send_Data(sensor, CHANNEL_CONTROL, 2);

  // Wait for response
  status &= check_Command_Success(sensor, status);

  if (status == N_ERR) {
    if (sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
      // Store product id response (cf. [2], p. 39 f.)
      // Get SW Version Major (byte 2)
      sensor->info.SW_Version_Major = sensor->shtp_package.shtp_Data[2];
      // Get SW Version Minor (byte 3)
      sensor->info.SW_Version_Minor = sensor->shtp_package.shtp_Data[3];
      // Get SW Part Number, combine four bytes in one uint32_t value (byte 4:7)
      sensor->info.SW_Part_Number =
          ((uint32_t)sensor->shtp_package.shtp_Data[7] << 24) |
          ((uint32_t)sensor->shtp_package.shtp_Data[6] << 16) |
          ((uint32_t)sensor->shtp_package.shtp_Data[5] << 8) |
          ((uint32_t)sensor->shtp_package.shtp_Data[4]);
      // Get SW Build Number, combine four bytes in one uint32_t value (byte
      // 8:11)
      sensor->info.SW_Build_Number =
          ((uint32_t)sensor->shtp_package.shtp_Data[11] << 24) |
          ((uint32_t)sensor->shtp_package.shtp_Data[10] << 16) |
          ((uint32_t)sensor->shtp_package.shtp_Data[9] << 8) |
          ((uint32_t)sensor->shtp_package.shtp_Data[8]);
      // Get SW Version Patch, combine two bytes in one uint16_t value (byte
      // 12:13)
      sensor->info.SW_Version_Patch =
          ((uint16_t)sensor->shtp_package.shtp_Data[13] << 8) |
          ((uint16_t)sensor->shtp_package.shtp_Data[12]);

      // Product ID Response complete
      return status;
    }
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Reads and stores the reset reason for the last reset in sensor struct.
 * @note: Not working for softreset_IMU(), because softreset_IMU() is executed
 * on an other channel.
 * @note: Assignment: 0 = Not applicable, 1 = Power on reset, 2 = Internal
 * System Reset, 3 = Watchdog Timeout, 4 = External reset, 5 = Other
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t get_Reset_Reason(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Check reset reason via Product ID Request and Response (cf. [2], p.39 f.)
  status &= get_ProductID(sensor);

  if (status == N_ERR) {
    if (sensor->shtp_package.shtp_Data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
      // Store product id response (cf. [2], p. 39 f.)
      // Get reset reason (byte 1)
      sensor->reset_reason = sensor->shtp_package.shtp_Data[1];

      // Product ID Response complete
      return status;
    }
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Checks the connection of IMU via Product ID Request and Response.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t check_Connection_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Check connection via Product ID Request and Response.
  status &= get_ProductID(sensor);

  return status;
}

/**
 * @brief Sends Tare Now command. User can decide between all axis Tare or z
 * axis Tare.
 * @note: If Rotation Vector, then reorientation of all motion outputs.
 * @note: If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be
 * found before Tare. The user can use a loop in main to repeat Tare until
 * accuracy is 3 (cf. [1], p. 42).
 * @note: The function detects if the magnetic north has not yet been found.
 * @note: If Game Rotation Vector, then only tare of Game Rotation Vector. Z
 * axis Tare cannot be persistent (no absolute reference for heading).
 * @note: If z axis Tare and (ARVR-Stabilized) Game Rotation Vector, the
 * function use softreset_IMU() to start with new config (cf. [1], p. 42).
 * @note: If ARVR-Stabilized modes, then tare of underlying modes.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param all_Axis: Boolean variable to choose Tare mode. True for all axis
 * Tare.
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 47 ff.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Tare Command 0x03
// shtp_Data[3]: P0 Tare Subcommand Tare Now 0x00
// shtp_Data[4]: P1 Tare Mode (all axis, z axis)
// shtp_Data[5]: P2 Rotation Vector Basis
// shtp_Data[6]: P3 Reserved
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t tare_IMU(sensor_meta *sensor, bool all_Axis) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 47 ff.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0;
  }
  sensor->shtp_package.shtp_Data[3] = SENSOR_COMMAND_TARE_NOW;
  sensor->shtp_package.shtp_Data[4] =
      all_Axis ? SENSOR_COMMAND_TARE_AXIS_ALL : SENSOR_COMMAND_TARE_AXIS_Z;
  uint8_t rotation_vector_mode_map = 0;
  switch (sensor->rotation_vector_mode) {
    case SENSOR_REPORTID_ROTATION_VECTOR:
      rotation_vector_mode_map = 0;
      // Check if IMU has found magnetic north
      if (sensor->quaternions.quat_Accuracy != 3) {
        return D_ERR;
      }
      break;
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
      rotation_vector_mode_map = 1;
      break;
    // cases for Geomagnetic and Gyro-Integrated Rotation Vector not needed
    case SENSOR_REPORTID_ARVR_ROTATION_VECTOR:
      rotation_vector_mode_map = 4;
      // Check if IMU has found magnetic north
      if (sensor->quaternions.quat_Accuracy != 3) {
        return D_ERR;
      }
      break;
    case SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR:
      rotation_vector_mode_map = 5;
      break;
    default:
      // Error
      status = D_ERR;
      return status;
  }
  sensor->shtp_package.shtp_Data[5] = rotation_vector_mode_map;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_TARE);

  // If z axis Tare and (ARVR-Stabilized) Game Rotation Vector, use reset to
  // start with new config (cf. [1], p. 42).
  if ((sensor->rotation_vector_mode == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
       sensor->rotation_vector_mode ==
           SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR) &&
      (all_Axis == false)) {
    // Softreset without clearing DCD, check success
    status &= softreset_IMU(sensor);
    status &= check_Command_Success(sensor, status);
    // Enable again specific mode
    if (sensor->rotation_vector_mode == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      status &= enable_GameRotationVector(
          sensor, sensor->rotation_vector_report_frequency);
      status &= check_Command_Success(sensor, status);
    } else if (sensor->rotation_vector_mode ==
               SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR) {
      status &= enable_ARVR_stabilized_GameRotationVector(
          sensor, sensor->rotation_vector_report_frequency);
      status &= check_Command_Success(sensor, status);
    } else {
      return D_ERR;
    }
  }

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Store the last Tare command to flash.
 * @note: Only persists if mode is (ARVR-Stabilized) Rotation Vector.
 * @note: If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be
 * found before Tare. The user can use a loop in main to repeat Tare until
 * accuracy is 3 (cf. [1], p. 42).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 47 ff.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Tare Command 0x03
// shtp_Data[3]: P0 Tare Subcommand Persist Tare 0x01
// shtp_Data[4]: P1 Reserved
// shtp_Data[5]: P2 Reserved
// shtp_Data[6]: P3 Reserved
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t tare_persist_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Check if not (ARVR-Stabilized) Game Rotation Vector
  if (sensor->rotation_vector_mode == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
      sensor->rotation_vector_mode ==
          SENSOR_REPORTID_ARVR_GAME_ROTATION_VECTOR) {
    return D_ERR;
  }

  // Check if IMU has found magnetic north in case of (ARVR-Stabilized) Rotation
  // Vector
  if (sensor->rotation_vector_mode == SENSOR_REPORTID_ROTATION_VECTOR ||
      sensor->rotation_vector_mode == SENSOR_REPORTID_ARVR_ROTATION_VECTOR) {
    if (sensor->quaternions.quat_Accuracy != 3) {
      return D_ERR;
    }
  }

  // Fill shtp_data with the values for this command (cf. [2], p. 47 ff.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0;
  }
  sensor->shtp_package.shtp_Data[3] = SENSOR_COMMAND_TARE_PERSIST;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_TARE);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Set Reorientation of Tare, set current run-time sensor orientation.
 * @note: Does not replace any persistent Tare settings. To clear Tare settings
 * of flash by calling tare_clear_IMU().
 * @note: If (ARVR-Stabilized) Rotation Vector, the magnetic north has to be
 * found before Tare. The user can use a loop in main to repeat Tare until
 * accuracy is 3 (cf. [1], p. 42).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 47 ff.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Tare Command 0x03
// shtp_Data[3]: P0 Tare Subcommand Set Reorientation 0x02
// shtp_Data[4]: P1 Rotation quaternion X LSB
// shtp_Data[5]: P2 Rotation quaternion X MSB
// shtp_Data[6]: P3 Rotation quaternion Y LSB
// shtp_Data[7]: P4 Rotation quaternion Y MSB
// shtp_Data[8]: P5 Rotation quaternion Z LSB
// shtp_Data[9]: P6 Rotation quaternion Z MSB
// shtp_Data[10]: P7 Rotation quaternion W LSB
// shtp_Data[11]: P8 Rotation quaternion W MSB
uint8_t tare_set_reorientation_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Check if IMU has found magnetic north in case of (ARVR-Stabilized) Rotation
  // Vector
  if (sensor->rotation_vector_mode == SENSOR_REPORTID_ROTATION_VECTOR ||
      sensor->rotation_vector_mode == SENSOR_REPORTID_ARVR_ROTATION_VECTOR) {
    if (sensor->quaternions.quat_Accuracy != 3) {
      return D_ERR;
    }
  }

  // Fill shtp_data with the values for this command (cf. [2], p. 47 ff.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0;
  }
  sensor->shtp_package.shtp_Data[3] = SENSOR_COMMAND_TARE_SET_REORIENTATION;
  sensor->shtp_package.shtp_Data[4] =
      (byte)(sensor->quaternions.raw_Quat_I &
             0xFF);  // Extract the Rotation Vector X LSB
  sensor->shtp_package.shtp_Data[5] =
      (byte)((sensor->quaternions.raw_Quat_I >> 8) &
             0xFF);  // Extract the Rotation Vector X MSB
  sensor->shtp_package.shtp_Data[6] =
      (byte)(sensor->quaternions.raw_Quat_J &
             0xFF);  // Extract the Rotation Vector Y LSB
  sensor->shtp_package.shtp_Data[7] =
      (byte)((sensor->quaternions.raw_Quat_J >> 8) &
             0xFF);  // Extract the Rotation Vector Y MSB
  sensor->shtp_package.shtp_Data[8] =
      (byte)(sensor->quaternions.raw_Quat_K &
             0xFF);  // Extract the Rotation Vector Z LSB
  sensor->shtp_package.shtp_Data[9] =
      (byte)((sensor->quaternions.raw_Quat_K >> 8) &
             0xFF);  // Extract the Rotation Vector Z MSB
  sensor->shtp_package.shtp_Data[10] =
      (byte)(sensor->quaternions.raw_Quat_Real &
             0xFF);  // Extract the Rotation Vector W LSB
  sensor->shtp_package.shtp_Data[11] =
      (byte)((sensor->quaternions.raw_Quat_Real >> 8) &
             0xFF);  // Extract the Rotation Vector W MSB

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_TARE);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Clears current Tare by setting current run-time sensor orientation to
 * zero.
 * @note: Clears persistent Tare settings.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 47 ff.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Tare Command 0x03
// shtp_Data[3]: P0 Tare Subcommand Set Reorientation 0x02
// shtp_Data[4]: P1 Rotation quaternion X LSB 0x00
// shtp_Data[5]: P2 Rotation quaternion X MSB 0x00
// shtp_Data[6]: P3 Rotation quaternion Y LSB 0x00
// shtp_Data[7]: P4 Rotation quaternion Y MSB 0x00
// shtp_Data[8]: P5 Rotation quaternion Z LSB 0x00
// shtp_Data[9]: P6 Rotation quaternion Z MSB 0x00
// shtp_Data[10]: P7 Rotation quaternion W LSB 0x00
// shtp_Data[11]: P8 Rotation quaternion W MSB 0x00
uint8_t tare_clear_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 47 ff.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }
  sensor->shtp_package.shtp_Data[3] = SENSOR_COMMAND_TARE_SET_REORIENTATION;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_TARE);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Reinitializes the IMU.
 * @note: The initialize response is not unsolicited. The advertisement message
 * is cleared.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 49 f.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Initialize Command 0x04
// shtp_Data[3]: P0 Subsystem
// shtp_Data[4]: P1 Reserved
// shtp_Data[5]: P2 Reserved
// shtp_Data[6]: P3 Reserved
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t reinitialize_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 49 f.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }
  sensor->shtp_package.shtp_Data[3] = 1;  // 0 would mean "no operation"

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_INITIALIZE);

  // Interception of a normal sensor report, throw it away (happens if command
  // config takes too long).
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Saves Dynamic Calibration Data (DCD).
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 49 f.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Save DCD Command 0x06
// shtp_Data[3]: P0 Reserved
// shtp_Data[4]: P1 Reserved
// shtp_Data[5]: P2 Reserved
// shtp_Data[6]: P3 Reserved
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t save_DCD_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 49 f.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_SAVE_DCD);

  // Interception of a normal sensor report, throw it away (happens if command
  // config takes too long).
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Configure calibration routines, enable / disable accelerometer, gyro
 * or magnetometer calibration routine.
 * @note: Calibration settings do not persist across reset.
 * @note: General recommendations (cf. [1], p. 38):
 * @note: Enable Accel Cal for reducing zero-g offset (ZGO), enable Planar Accel
 * Cal if calibration is only in 2D possible.
 * @note: Enable Gyro Cal for reducing zero rate offset (ZRO) / drift. For
 * successful calibration the device to which the IMU is mounted must have
 * sufficient tremor (f.ex. tremor of human hand).
 * @note: If you perform slow horizontal rotations around the gravity vector,
 * the calibration can be fooled. Disable the gyro cal.
 * @note: Enable Mag Cal for compensation of hard-iron effects (magnets) or
 * soft-iron (ferrous materials) effects. Dynamic cal is recommended.
 * @note: Default: Acc and Mag Cal enabled.
 * @note: Motion Tracking in relatively stable magnetic field and sufficient
 * tremor: Enable Gyro Cal.
 * @note: VR applications: Only enable Accel Cal to prevent jumps.
 * @note: Calibration Steps: see [1], p. 39
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param enable_Accel_Cal: Boolean to enable (true) or disable (false)
 * accelerometer calibration
 * @param enable_Gyro_Cal: Boolean to enable (true) or disable (false) gyro
 * calibration
 * @param enable_Mag_Cal: Boolean to enable (true) or disable (false)
 * magnetometer calibration
 * @param enable_Planar_Accel_Cal: Boolean to enable (true) or disable (false)
 * planar accelerometer calibration. Use the planar one if calibration is only
 * in 2D possible (f.ex. robot)
 * @param enable_On_Table_Cal: Boolean to enable (true) or disable (false) on
 * Table calibration
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 52 f.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: ME Calibration Command 0x07
// shtp_Data[3]: P0 Accel Cal Enable (1 - enabled, 2 - disabled)
// shtp_Data[4]: P1 Gyro Cal Enable (1 - enabled, 2 - disabled)
// shtp_Data[5]: P2 Mag Cal Enable (1 - enabled, 2 - disabled)
// shtp_Data[6]: P3 Subcommand: Configure ME Calibration 0x00
// shtp_Data[7]: P4 Planar Accel Cal Enable (1 - enabled, 2 - disabled)
// shtp_Data[8]: P5 On Table Cal Enable (1 - enabled, 2 - disabled)
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t configure_ME_Calibration_IMU(sensor_meta *sensor, bool enable_Accel_Cal,
                                     bool enable_Gyro_Cal, bool enable_Mag_Cal,
                                     bool enable_Planar_Accel_Cal,
                                     bool enable_On_Table_Cal) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 52 f.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }
  sensor->shtp_package.shtp_Data[3] = enable_Accel_Cal ? 1 : 0;
  sensor->shtp_package.shtp_Data[4] = enable_Gyro_Cal ? 1 : 0;
  sensor->shtp_package.shtp_Data[5] = enable_Mag_Cal ? 1 : 0;
  sensor->shtp_package.shtp_Data[6] = SENSOR_COMMAND_ME_CAL_CONFIG;
  sensor->shtp_package.shtp_Data[7] = enable_Planar_Accel_Cal ? 1 : 0;
  sensor->shtp_package.shtp_Data[8] = enable_On_Table_Cal ? 1 : 0;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_ME_CAL);

  // Interception of a normal sensor report, throw it away (happens if command
  // config takes too long).
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Get ME calibration config, get enabled / disabled calibration
 * routines.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 52 f.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: ME Calibration Command 0x07
// shtp_Data[3]: P0 Reserved
// shtp_Data[4]: P1 Reserved
// shtp_Data[5]: P2 Reserved
// shtp_Data[6]: P3 Subcommand: Configure ME Calibration 0x00
// shtp_Data[7]: P4 Reserved
// shtp_Data[8]: P5 Reserved
// shtp_Data[9]: P6 Reserved
// shtp_Data[10]: P7 Reserved
// shtp_Data[11]: P8 Reserved
uint8_t get_ME_Calibration_Config_IMU(sensor_meta *sensor) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 52 f.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }
  sensor->shtp_package.shtp_Data[6] = SENSOR_COMMAND_ME_CAL_GET;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_ME_CAL);

  // Interception of a normal sensor report, throw it away (happens if command
  // config takes too long).
  status &= clear_init_Message_IMU(sensor);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Enable / disable the periodic DCD routines.
 * @note: This command does not have a response! The function
 * check_Command_Success() does not work!
 * @note: This command does not inhibit the Save DCD command save_DCD_IMU().
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param enable_periodic_DCD: Boolean variable to enable (true) or disable
 * (false) periodic DCD routine
 * @return: status, 1 no error occurred, 0 error occurred
 */
// Structure of request packet (cf. [2], p. 52 f.):
// shtp_Data[0]: Command Request 0xF2
// shtp_Data[1]: Command sequence number
// shtp_Data[2]: Configure DCD Save Command 0x09
// shtp_Data[3]: P0 Enable Periodic DCD Save 0x00 / Disable Periodic DCD Save
// 0x01 shtp_Data[4]: P1 Reserved shtp_Data[5]: P2 Reserved shtp_Data[6]: P3
// Reserved shtp_Data[7]: P4 Reserved shtp_Data[8]: P5 Reserved shtp_Data[9]: P6
// Reserved shtp_Data[10]: P7 Reserved shtp_Data[11]: P8 Reserved
uint8_t config_periodic_DCD_IMU(sensor_meta *sensor, bool enable_periodic_DCD) {
  uint8_t status = N_ERR;

  // Fill shtp_data with the values for this command (cf. [2], p. 52 f.)
  for (uint8_t shtp_index_temp = 3; shtp_index_temp < 12; shtp_index_temp++) {
    sensor->shtp_package.shtp_Data[shtp_index_temp] = 0x00;
  }
  sensor->shtp_package.shtp_Data[3] = enable_periodic_DCD ? 0x00 : 0x01;

  // Send command
  status &= send_Command(sensor, SENSOR_COMMAND_CONFIG_DCD);

  if (status == N_ERR) {
    // command successful
    return status;
  }

  // Error
  status = D_ERR;
  return status;
}

/**
 * @brief Checks if the executed command was successful via the command
 * response.
 * @note: Only if the passed status is not an error, the function will be
 * executed. This prevents unnecessary blocking.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param status_command: Status of executed command (f.ex. startup_status),
 * only check success if status_command is not an error
 * @return: status, 1 no error occurred, 0 error occurred
 */
uint8_t check_Command_Success(sensor_meta *sensor, uint8_t status_command) {
  uint8_t status = N_ERR;
  uint16_t timeout_counter = 0;
  if (status_command) {
    while (data_available(sensor) != true) {
      ++timeout_counter;
      if (timeout_counter >=
          2000) {  // Wait max 2000 * 100 us = 200 ms; experimental value
        status = D_ERR;
        break;
      }
      delay_Us(100);  // experimental value
    }
  } else {
    // Error
    status = D_ERR;
  }

  return status;
}

/**
 * @brief Deasserts the corresponding csn pin of the sensor
 * @param *sensor: Pointer to corresponding sensor meta data
 */
void deassert_csn(sensor_meta *sensor) { csn(sensor->ports_pins, true); }

/**
 * @brief Read an FRS record from the sensor.
 * @param *sensor: Pointer to corresponding sensor meta data
 * @param frs_type: FRS record type to read (SH-2 Reference figure 26)
 * @param buffer: Pointer to pre-allocated buffer to store 32-bit words
 * @param max_words: Maximum number of words the buffer can hold
 * @param *words_read: Pointer to variable that will contain the number of words
 * read
 * @return N_ERR if successful, D_ERR on error
 */
uint8_t read_FRS(sensor_meta *sensor, uint16_t frs_type, uint32_t *buffer,
                 uint16_t max_words, uint16_t *words_read) {
  uint8_t status = N_ERR;
  uint16_t timeout_counter = 0;
  *words_read = 0;
  int16_t last_offset = -2;

  // Send FRS Read Request
  sensor->shtp_package.shtp_Data[0] = SHTP_REPORT_FRS_READ_REQUEST;  // 0xF4
  sensor->shtp_package.shtp_Data[1] = 0;                             // Reserved
  sensor->shtp_package.shtp_Data[2] = 0;                             // Reserved
  sensor->shtp_package.shtp_Data[3] = 0;                             // Reserved
  sensor->shtp_package.shtp_Data[4] = frs_type & 0xFF;         // FRS Type LSB
  sensor->shtp_package.shtp_Data[5] = (frs_type >> 8) & 0xFF;  // FRS Type MSB
  sensor->shtp_package.shtp_Data[6] = 0;                       // Reserved
  sensor->shtp_package.shtp_Data[7] = 0;                       // Reserved

  status &= send_Data(sensor, CHANNEL_CONTROL, 8);
  if (status == D_ERR) return D_ERR;

  // Wait for Read Response(s)
  while (1) {
    status &= check_Command_Success(sensor, status);
    if (status == D_ERR) return D_ERR;

    // Timeout protection
    if (++timeout_counter >= 1000) {
      return D_ERR;  // timed out waiting for response
    }

    // Check if this is a read response
    if (sensor->shtp_package.shtp_Data[0] != SHTP_REPORT_FRS_READ_RESPONSE) {
      continue;  // ignore unrelated packets
    }

    // Extract status, data length and offset
    uint8_t num_words = (sensor->shtp_package.shtp_Data[1] >> 4) & 0x0F;
    uint8_t frs_status = sensor->shtp_package.shtp_Data[1] & 0x0F;
    uint16_t offset = sensor->shtp_package.shtp_Data[2] |
                      (sensor->shtp_package.shtp_Data[3] << 8);

    if (frs_status == 5) {
      // Record empty → valid, but no data
      *words_read = 0;
      break;
    }

    if (offset != (last_offset + 2)) {
      // Missed packet
      return D_ERR;
    }

    if (frs_status == 0 || frs_status == 3) {
      // Copy words into buffer
      for (uint8_t i = 0; i < num_words; i++) {
        uint8_t data_index = 4 + i * 4;
        buffer[offset + i] =
            (sensor->shtp_package.shtp_Data[data_index]) |
            (sensor->shtp_package.shtp_Data[data_index + 1] << 8) |
            (sensor->shtp_package.shtp_Data[data_index + 2] << 16) |
            (sensor->shtp_package.shtp_Data[data_index + 3] << 24);
      }
      *words_read += num_words;
      last_offset = offset;

      if (frs_status == 3) {
        // Completed
        break;
      }
    } else {
      // Any other status = error
      return D_ERR;
    }
  }
  return N_ERR;
}

/**
 * @brief Write a block of words into an FRS record.
 * @param sensor: Pointer to sensor metadata
 * @param frs_type: FRS record type
 * @param words: Pointer to array of 32-bit words to be written
 * @param num_words: Number of 32-bit words in the array
 * @return status: N_ERR if success, D_ERR otherwise
 */
uint8_t write_FRS(sensor_meta *sensor, uint16_t frs_type, uint32_t *words,
                  uint16_t num_words) {
  uint8_t status = N_ERR;

  // Send FRS Write Request
  sensor->shtp_package.shtp_Data[0] =
      SHTP_REPORT_FRS_WRITE_REQUEST;                     // Report ID 0xF7
  sensor->shtp_package.shtp_Data[1] = 0;                 // Reserved
  sensor->shtp_package.shtp_Data[2] = num_words & 0xFF;  // Word count LSB
  sensor->shtp_package.shtp_Data[3] =
      (num_words >> 8) & 0xFF;                                 // Word count MSB
  sensor->shtp_package.shtp_Data[4] = frs_type & 0xFF;         // FRS type LSB
  sensor->shtp_package.shtp_Data[5] = (frs_type >> 8) & 0xFF;  // FRS type MSB

  status &= send_Data(sensor, CHANNEL_CONTROL, 6);

  // Wait for Write Response
  status &= check_Command_Success(sensor, status);
  if (status == D_ERR ||
      sensor->shtp_package.shtp_Data[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {
    return D_ERR;
  }

  uint8_t frs_status = sensor->shtp_package.shtp_Data[1];
  if (frs_status != 4) {
    // Sensor not ready for write mode
    return D_ERR;
  }
  delay_Us(1000);

  // Send FRS Write Data Packets
  uint16_t offset = 0;
  while (offset < num_words) {
    sensor->shtp_package.shtp_Data[0] = SHTP_REPORT_FRS_WRITE_DATA_REQUEST;
    sensor->shtp_package.shtp_Data[1] = 0;                     // Reserved
    sensor->shtp_package.shtp_Data[2] = offset & 0xFF;         // Offset LSB
    sensor->shtp_package.shtp_Data[3] = (offset >> 8) & 0xFF;  // Offset MSB

    // Pack first word
    if (offset < num_words) {
      sensor->shtp_package.shtp_Data[4] = words[offset] & 0xFF;
      sensor->shtp_package.shtp_Data[5] = (words[offset] >> 8) & 0xFF;
      sensor->shtp_package.shtp_Data[6] = (words[offset] >> 16) & 0xFF;
      sensor->shtp_package.shtp_Data[7] = (words[offset] >> 24) & 0xFF;
      offset++;
    } else {
      sensor->shtp_package.shtp_Data[4] = 0;
      sensor->shtp_package.shtp_Data[5] = 0;
      sensor->shtp_package.shtp_Data[6] = 0;
      sensor->shtp_package.shtp_Data[7] = 0;
    }

    // Pack second word if available
    if (offset < num_words) {
      sensor->shtp_package.shtp_Data[8] = words[offset] & 0xFF;
      sensor->shtp_package.shtp_Data[9] = (words[offset] >> 8) & 0xFF;
      sensor->shtp_package.shtp_Data[10] = (words[offset] >> 16) & 0xFF;
      sensor->shtp_package.shtp_Data[11] = (words[offset] >> 24) & 0xFF;
      offset++;
    } else {
      sensor->shtp_package.shtp_Data[8] = 0;
      sensor->shtp_package.shtp_Data[9] = 0;
      sensor->shtp_package.shtp_Data[10] = 0;
      sensor->shtp_package.shtp_Data[11] = 0;
    }

    status &= send_Data(sensor, CHANNEL_CONTROL, 12);
    if (status == D_ERR) return D_ERR;

    // Wait for Write Response after each data packet
    /* TODO: Sensor responds with "4 - write mode entered or ready" after the
     * write request but with "6 – data received while not in write mode"
     * after the data request. We are looking for "0 – word(s) received".The
     * write operation therefore fails. Problem so far unclear.*/
    status &= check_Command_Success(sensor, status);
    if (status == D_ERR ||
        sensor->shtp_package.shtp_Data[0] != SHTP_REPORT_FRS_WRITE_RESPONSE) {
      return D_ERR;
    }
    frs_status = sensor->shtp_package.shtp_Data[1];
    uint16_t response_offset = sensor->shtp_package.shtp_Data[2] |
                               (sensor->shtp_package.shtp_Data[3] << 8);

    // Offset consistency check
    uint16_t expected_offset = offset - 1;  // last written word
    if (response_offset != expected_offset) {
      return D_ERR;  // mismatch -> error
    }

    // Check if data is received or write is completed
    switch (frs_status) {
      case 0:
        break;
      case 3:
        offset = num_words;
        break;
      default:
        return D_ERR;
    }
  }

  if (frs_status == 3) {
    return N_ERR;  // Success
  }

  return D_ERR;
}

/**
 * @brief Erases an FRS record on the target sensor.
 *
 * @note This function sends an FRS Write Request (0xF7) with the specified
 *       record type and a length of 0 words, which instructs the device to
 *       erase the record.
 *
 * @param *sensor   Pointer to the sensor meta data structure, which contains
 *                  the SHTP packet buffer and communication state.
 * @param frs_type  The FRS record type to erase (see datasheet, Figure 26).
 *
 * @return Status code:
 *         - @c N_ERR if the erase completed successfully
 *         - @c D_ERR if an error occurred (communication failure, invalid
 * response, or device reported an error status).
 */

uint8_t erase_FRS(sensor_meta *sensor, uint16_t frs_type) {
  // No data pointer, length_words = 0 → erase mode
  return write_FRS(sensor, frs_type, NULL, 0);
}
