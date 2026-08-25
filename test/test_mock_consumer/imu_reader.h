/*
 * @file imu_reader.h
 * @brief Stand-in for application code that consumes this library.
 *
 * Exists so that the generated mock is exercised the same way a downstream
 * project exercises it: application code calls the library, and the test
 * replaces the library with MockBNO085_SPI_Library.
 */

#ifndef TEST_TEST_MOCK_CONSUMER_IMU_READER_H_
#define TEST_TEST_MOCK_CONSUMER_IMU_READER_H_

#include <stdbool.h>
#include <stdint.h>

#include "BNO085_SPI_Library.h"

typedef struct quaternion_sample {
  float i;
  float j;
  float k;
  float real;
  uint8_t accuracy;
} quaternion_sample;

// Resets the IMU, drains its init message and enables the game rotation
// vector. Returns N_ERR when the IMU is streaming, D_ERR otherwise.
uint8_t imu_reader_start(sensor_meta *sensor, uint16_t report_interval_ms);

// Reads one sample. Returns false when the IMU has no data pending, leaving
// sample untouched.
bool imu_reader_read(sensor_meta *sensor, quaternion_sample *sample);

#endif  // TEST_TEST_MOCK_CONSUMER_IMU_READER_H_
