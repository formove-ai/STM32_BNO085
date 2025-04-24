/*
 * @file Sensor_Struct.h
 * @brief Structs of sensor meta data: Pin, ports, quaternions, info, shtp_data.
 *
 * See sensorsuit/experimental/imu_readout_library/README.md for more
 * information.
 */

#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_SENSOR_STRUCT_H_
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_SENSOR_STRUCT_H_

// --- Includes ------------------------------------------------------
// Includes of standard libraries
#include <stdbool.h>
#include <stdint.h>

// Private includes
#include "stm32_hal.h"

// --- Macros ---------------------------------------------------------

// Limit sensor reports to 128 bytes (RAM restriction)
#define MAX_PACKET_SIZE 128
// Each packet has a header of 4 bytes (cf. [1], p. 22, figure 1-26)
#define HEADER_PACKET_SIZE 4

// There are 6 command channels.
#define CHANNEL_MAX_NUMBER 6

// --- Sensor structs -------------------------------------------------
// Struct for ports and pins of sensor
typedef struct ports_pins {
  GPIO_TypeDef *INTN_Port;
  uint16_t INTN_Pin;
  GPIO_TypeDef *CSN_Port;
  uint16_t CSN_Pin;
  GPIO_TypeDef *RSTN_Port;
  uint16_t RSTN_Pin;
} ports_pins;

// Struct for software info of the sensor
typedef struct sensor_info {
  uint8_t SW_Version_Major;
  uint8_t SW_Version_Minor;
  uint8_t SW_Part_Number;
  uint8_t SW_Build_Number;
  uint8_t SW_Version_Patch;
} sensor_info;

// Struct for shtp data
typedef struct shtp_package {
  uint8_t shtp_Header[HEADER_PACKET_SIZE];  // Data storage for header of each
                                            // data packet
  uint8_t shtp_Data[MAX_PACKET_SIZE];       // Data storage for report
  volatile uint8_t sequence_Number[CHANNEL_MAX_NUMBER];  // Each channel has its
                                                         // own sequence number.
  volatile uint8_t
      command_Sequence_Number;  // Commands have a sequence number as well.
                                // These are inside command packet, the header
                                // uses its own sequence number per channel
} shtp_package;

// Struct for quaternion data
typedef struct quaternion_data {
  uint16_t raw_Quat_I;
  uint16_t raw_Quat_J;
  uint16_t raw_Quat_K;
  uint16_t raw_Quat_Real;
  uint16_t raw_Quat_Radian_Accuracy;
  float Quat_I;
  float Quat_J;
  float Quat_K;
  float Quat_Real;
  float Quat_Radian_Accuracy;
  uint16_t quat_Accuracy;
} quaternion_data;

// Struct for accelerometer data
typedef struct accelerometer_data {
  uint16_t raw_Accel_X;
  uint16_t raw_Accel_Y;
  uint16_t raw_Accel_Z;
  float Accel_X;
  float Accel_Y;
  float Accel_Z;
  uint16_t accelerometer_Accuracy;
} accelerometer_data;

typedef accelerometer_data accelerometer_data;
typedef accelerometer_data linear_acceleration_data;
typedef accelerometer_data gravity_data;

// Struct for additional data
typedef struct additional_data {
  volatile uint8_t stability_Classifier;
  volatile uint8_t raw_tap_Detector;
  volatile uint32_t counter_single_tap;
  volatile uint32_t counter_double_tap;
  volatile bool has_single_tap;  // Flag has to be cleared somewhere after usage
  volatile bool has_double_tap;  // Flag has to be cleared somewhere after usage
} additional_data;

// Struct for meta data of the sensor
typedef struct sensor_meta {
  uint8_t number;
  volatile bool
      has_reset;  // Keeps track of any received reset complete packets
  volatile uint8_t reset_reason;  // Reset reason if reset occurred
  volatile uint8_t
      INTN_ready;  // flag for data operation, 1 when INTN PIN is LOW
  volatile uint8_t rotation_vector_mode;
  volatile uint16_t rotation_vector_report_frequency;
  volatile uint16_t accelerometer_report_frequency;
  volatile uint16_t linear_acceleration_report_frequency;
  volatile uint16_t gravity_report_frequency;
  ports_pins ports_pins;
  sensor_info info;
  shtp_package shtp_package;
  quaternion_data quaternions;
  accelerometer_data accelerometer_data;
  linear_acceleration_data linear_acceleration_data;
  gravity_data gravity_data;
  additional_data additional_data;
} sensor_meta;

#endif  // SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_SENSOR_STRUCT_H_
