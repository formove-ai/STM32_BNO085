/*
 * @file BNO085_SPI_Library.h
 * @brief SPI Interface and service routine functions for BNO085 via SHTP
 * protocol used by the manufacturer CEVA.
 *
 * See sensorsuit/experimental/imu_readout_library/README.md for more
 * information.
 */

#ifndef SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_
#define SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_

// --- Includes -------------------------------------------------------
// Includes of standard libraries
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Private includes
#include "BNO085_SPI_Error_Flags.h"
#include "BNO085_SPI_Shims.h"
#include "Hardware_Init.h"
#include "Sensor_Struct.h"
#include "stm32_hal.h"
#include "stm32_hal_spi.h"

// --- Public functions -----------------------------------------------
void register_Sensor(sensor_meta *sensor, uint8_t sensor_number,
                     uint16_t sensor_CSN_Pin, GPIO_TypeDef *sensor_CSN_Port,
                     uint16_t sensor_INTN_Pin, GPIO_TypeDef *sensor_INTN_Port,
                     uint16_t sensor_RSTN_Pin, GPIO_TypeDef *sensor_RSTN_Port);
uint8_t clear_init_Message_IMU(sensor_meta *sensor);
void hardreset_IMU(sensor_meta *sensor);
uint8_t softreset_IMU(sensor_meta *sensor);
uint8_t softresetDCD_IMU(sensor_meta *sensor);
bool get_and_clear_Reset_Status(sensor_meta *sensor);
uint8_t enable_Accelerometer(sensor_meta *sensor,
                             uint16_t time_between_reports);
uint8_t enable_LinearAcceleration(sensor_meta *sensor,
                                  uint16_t time_between_reports);
uint8_t enable_Gravity(sensor_meta *sensor, uint16_t time_between_reports);
uint8_t enable_RotationVector(sensor_meta *sensor,
                              uint16_t time_between_reports);
uint8_t enable_ARVR_stabilized_RotationVector(sensor_meta *sensor,
                                              uint16_t time_between_reports);
uint8_t enable_GameRotationVector(sensor_meta *sensor,
                                  uint16_t time_between_reports);
uint8_t enable_ARVR_stabilized_GameRotationVector(
    sensor_meta *sensor, uint16_t time_between_reports);
uint8_t enable_StabilityClassifier(sensor_meta *sensor,
                                   uint16_t time_between_reports);
uint8_t enable_TapDetector(sensor_meta *sensor, uint16_t time_between_reports);
bool data_available(sensor_meta *sensor);
float get_Accelerometer_X(sensor_meta *sensor);
float get_Accelerometer_Y(sensor_meta *sensor);
float get_Accelerometer_Z(sensor_meta *sensor);
uint8_t get_Accelerometer_Accuracy(sensor_meta *sensor);
float get_LinearAcceleration_X(sensor_meta *sensor);
float get_LinearAcceleration_Y(sensor_meta *sensor);
float get_LinearAcceleration_Z(sensor_meta *sensor);
uint8_t get_LinearAcceleration_Accuracy(sensor_meta *sensor);
float get_Gravity_X(sensor_meta *sensor);
float get_Gravity_Y(sensor_meta *sensor);
float get_Gravity_Z(sensor_meta *sensor);
uint8_t get_Gravity_Accuracy(sensor_meta *sensor);
float get_Quat_I(sensor_meta *sensor);
float get_Quat_J(sensor_meta *sensor);
float get_Quat_K(sensor_meta *sensor);
float get_Quat_Real(sensor_meta *sensor);
float get_Quat_Radian_Accuracy(sensor_meta *sensor);
uint8_t get_Quat_Accuracy(sensor_meta *sensor);
uint8_t get_StabilityClassifier(sensor_meta *sensor);
void update_TapDetector(sensor_meta *sensor);
uint8_t get_ProductID(sensor_meta *sensor);
uint8_t get_Reset_Reason(sensor_meta *sensor);
uint8_t check_Connection_IMU(sensor_meta *sensor);
uint8_t tare_IMU(sensor_meta *sensor, bool all_Axis);
uint8_t tare_persist_IMU(sensor_meta *sensor);
uint8_t tare_set_reorientation_IMU(sensor_meta *sensor);
uint8_t tare_clear_IMU(sensor_meta *sensor);
uint8_t reinitialize_IMU(sensor_meta *sensor);
uint8_t save_DCD_IMU(sensor_meta *sensor);
uint8_t configure_ME_Calibration_IMU(sensor_meta *sensor, bool enable_Accel_Cal,
                                     bool enable_Gyro_Cal, bool enable_Mag_Cal,
                                     bool enable_Planar_Accel_Cal,
                                     bool enable_On_Table_Cal);
uint8_t get_ME_Calibration_Config_IMU(sensor_meta *sensor);
uint8_t config_periodic_DCD_IMU(sensor_meta *sensor, bool enable_periodic_DCD);
uint8_t check_Command_Success(sensor_meta *sensor, uint8_t status_command);
void deassert_csn(sensor_meta *sensor);
uint8_t read_FRS(sensor_meta *sensor, uint16_t frs_type, uint32_t *buffer,
                 uint16_t max_words, uint16_t *words_read);
uint8_t write_FRS(sensor_meta *sensor, uint16_t frs_type, uint32_t *words,
                  uint16_t num_words);
uint8_t erase_FRS(sensor_meta *sensor, uint16_t frs_type);

// Private function exposed for unit testing:
#ifdef BNO085_BUILD_FOR_INTERNAL_UNIT_TESTS
uint16_t parse_InputReport(sensor_meta *sensor);
#endif  // BNO085_BUILD_FOR_INTERNAL_UNIT_TESTS

#endif  // SENSORSUIT_PROD_BNO085_SPI_LIB_BNO085_SPI_INCLUDE_BNO085_SPI_LIBRARY_H_
