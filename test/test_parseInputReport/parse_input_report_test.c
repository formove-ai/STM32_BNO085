#include <BNO085_SPI_Library.h>
#include <unity.h>

// Implement the shims
void delay_Us(uint32_t delay) {}

void test_parse_InputReport_checks_for_null() {
  sensor_meta sensor;
  TEST_ASSERT_EQUAL(0, parse_InputReport(NULL));
}

void test_parse_InputReport_fills_sensor_struct_for_GameRotationVector() {
  shtp_package shtp = {
      .shtp_Header = {17 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
              0x02,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06,        // Z
              0x07, 0x08,        // W
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .rotation_vector_mode = 0x08,
      .rotation_vector_report_frequency = 10,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x08, parse_InputReport(&sensor));
  TEST_ASSERT_EQUAL(0x0201, sensor.quaternions.raw_Quat_I);
  TEST_ASSERT_EQUAL(0x0403, sensor.quaternions.raw_Quat_J);
  TEST_ASSERT_EQUAL(0x0605, sensor.quaternions.raw_Quat_K);
  TEST_ASSERT_EQUAL(0x0807, sensor.quaternions.raw_Quat_Real);
}

void test_parse_InputReport_fills_sensor_struct_for_LinearAcceleration() {
  shtp_package shtp = {
      .shtp_Header = {15 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
              0x02,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06         // Z
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .rotation_vector_mode = 0x08,
      .rotation_vector_report_frequency = 10,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x04, parse_InputReport(&sensor));
  // These should be unchanged:
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_I);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_J);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_K);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_Real);
  // These should contain the values from the data package:
  TEST_ASSERT_EQUAL(0x0201, sensor.linear_acceleration_data.raw_Accel_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.linear_acceleration_data.raw_Accel_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.linear_acceleration_data.raw_Accel_Z);
  TEST_ASSERT_EQUAL(0x02,
                    sensor.linear_acceleration_data.accelerometer_Accuracy);
}

void test_parse_InputReport_fills_sensor_struct_for_CalibratedGyroscope() {
  shtp_package shtp = {
      .shtp_Header = {15 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
              0x03,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06         // Z
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x02, parse_InputReport(&sensor));
  TEST_ASSERT_EQUAL(0x0201, sensor.gyroscope_data.raw_Gyro_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.gyroscope_data.raw_Gyro_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.gyroscope_data.raw_Gyro_Z);
  TEST_ASSERT_EQUAL(0x03, sensor.gyroscope_data.gyroscope_Accuracy);
}
void test_parse_InputReport_fills_all_acceleration_data() {
  shtp_package linear_acceleration_package = {
      .shtp_Header = {15 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,
              0x02,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06         // Z
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  shtp_package gravity_package = {
      .shtp_Header = {15 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
              0x02,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06         // Z
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  shtp_package accelerometer_package = {
      .shtp_Header = {15 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
              0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
              0x02,              // Status (Accuracy)
              0x00, 0x01, 0x02,  // X
              0x03, 0x04,        // Y
              0x05, 0x06         // Z
          },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .rotation_vector_mode = 0x08,
      .rotation_vector_report_frequency = 10,
      .shtp_package = linear_acceleration_package,
  };
  TEST_ASSERT_EQUAL(0x04, parse_InputReport(&sensor));
  sensor.shtp_package = gravity_package;
  TEST_ASSERT_EQUAL(0x06, parse_InputReport(&sensor));
  sensor.shtp_package = accelerometer_package;
  TEST_ASSERT_EQUAL(0x01, parse_InputReport(&sensor));
  // These should be unchanged:
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_I);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_J);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_K);
  TEST_ASSERT_EQUAL(0, sensor.quaternions.raw_Quat_Real);
  // These should contain the values from the data package:
  TEST_ASSERT_EQUAL(0x0201, sensor.accelerometer_data.raw_Accel_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.accelerometer_data.raw_Accel_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.accelerometer_data.raw_Accel_Z);
  TEST_ASSERT_EQUAL(0x02, sensor.accelerometer_data.accelerometer_Accuracy);
  TEST_ASSERT_EQUAL(0x0201, sensor.linear_acceleration_data.raw_Accel_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.linear_acceleration_data.raw_Accel_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.linear_acceleration_data.raw_Accel_Z);
  TEST_ASSERT_EQUAL(0x02,
                    sensor.linear_acceleration_data.accelerometer_Accuracy);
  TEST_ASSERT_EQUAL(0x0201, sensor.gravity_data.raw_Accel_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.gravity_data.raw_Accel_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.gravity_data.raw_Accel_Z);
  TEST_ASSERT_EQUAL(0x02, sensor.gravity_data.accelerometer_Accuracy);
}

void test_parse_InputReport_fills_sensor_struct_for_RawAccelerometer() {
  shtp_package shtp = {
      .shtp_Header = {21 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
          0x00, 0x00, 0x00, 0x00, 0x00,  // SHTP timestamp (ignored by parser)
          0x14, 0x00,                    // Report ID, sequence
          0x02,                          // Status (Accuracy)
          0x00,                          // Delay
          0x01, 0x02,                    // X (LSB, MSB)
          0x03, 0x04,                    // Y
          0x05, 0x06,                    // Z
          0xAA, 0xBB,                    // Reserved (2 bytes)
          0x05, 0x06, 0x07, 0x08,        // Raw report timestamp (uint32 LE)
      },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x14, parse_InputReport(&sensor));
  TEST_ASSERT_EQUAL(0x0201, sensor.raw_accelerometer_data.raw_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.raw_accelerometer_data.raw_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.raw_accelerometer_data.raw_Z);
  TEST_ASSERT_EQUAL(0x02, sensor.raw_accelerometer_data.accuracy);
  TEST_ASSERT_EQUAL(0x08070605, sensor.raw_accelerometer_data.timestamp);
}

void test_parse_InputReport_fills_sensor_struct_for_RawGyroscope() {
  shtp_package shtp = {
      .shtp_Header = {21 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
          0x00, 0x00, 0x00, 0x00, 0x00,  // SHTP timestamp (ignored by parser)
          0x15, 0x00,                    // Report ID, sequence
          0x02,                          // Status (Accuracy)
          0x00,                          // Delay
          0x01, 0x02,                    // X (LSB, MSB)
          0x03, 0x04,                    // Y
          0x05, 0x06,                    // Z
          0xAA, 0xBB,                    // Reserved (2 bytes)
          0x05, 0x06, 0x07, 0x08,        // Raw report timestamp (uint32 LE)
      },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x15, parse_InputReport(&sensor));
  TEST_ASSERT_EQUAL(0x0201, sensor.raw_gyroscope_data.raw_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.raw_gyroscope_data.raw_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.raw_gyroscope_data.raw_Z);
  TEST_ASSERT_EQUAL(0x02, sensor.raw_gyroscope_data.accuracy);
  TEST_ASSERT_EQUAL(0x08070605, sensor.raw_gyroscope_data.timestamp);
}

void test_parse_InputReport_fills_sensor_struct_for_RawMagnetometer() {
  shtp_package shtp = {
      .shtp_Header = {21 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
          0x00, 0x00, 0x00, 0x00, 0x00,  // SHTP timestamp (ignored by parser)
          0x16, 0x00,                    // Report ID, sequence
          0x02,                          // Status (Accuracy)
          0x00,                          // Delay
          0x01, 0x02,                    // X (LSB, MSB)
          0x03, 0x04,                    // Y
          0x05, 0x06,                    // Z
          0xAA, 0xBB,                    // Reserved (2 bytes)
          0x05, 0x06, 0x07, 0x08,        // Raw report timestamp (uint32 LE)
      },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  sensor_meta sensor = {
      .number = 1,
      .shtp_package = shtp,
  };
  TEST_ASSERT_EQUAL(0x16, parse_InputReport(&sensor));
  TEST_ASSERT_EQUAL(0x0201, sensor.raw_magnetometer_data.raw_X);
  TEST_ASSERT_EQUAL(0x0403, sensor.raw_magnetometer_data.raw_Y);
  TEST_ASSERT_EQUAL(0x0605, sensor.raw_magnetometer_data.raw_Z);
  TEST_ASSERT_EQUAL(0x02, sensor.raw_magnetometer_data.accuracy);
  TEST_ASSERT_EQUAL(0x08070605, sensor.raw_magnetometer_data.timestamp);
}

void test_parse_InputReport_raw_accel_timestamp_is_monotonic() {
  shtp_package shtp1 = {
      .shtp_Header = {21 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
          0x00, 0x00, 0x00, 0x00, 0x00,  // SHTP timestamp (ignored by parser)
          0x14, 0x00,                    // Report ID, sequence
          0x02,                          // Status (Accuracy)
          0x00,                          // Delay
          0x01, 0x02,                    // X (LSB, MSB)
          0x03, 0x04,                    // Y
          0x05, 0x06,                    // Z
          0xAA, 0xBB,                    // Reserved (2 bytes)
          0x04, 0x03, 0x02, 0x01,        // Timestamp = 0x01020304 (LE)
      },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };
  shtp_package shtp2 = {
      .shtp_Header = {21 + 4, 0x00,  // Data length, data + header
                      0x00, 0x00},
      .shtp_Data = {
          0x00, 0x00, 0x00, 0x00, 0x00,  // SHTP timestamp (ignored by parser)
          0x14, 0x01,                    // Report ID, sequence
          0x02,                          // Status (Accuracy)
          0x00,                          // Delay
          0x01, 0x02,                    // X (LSB, MSB)
          0x03, 0x04,                    // Y
          0x05, 0x06,                    // Z
          0xAA, 0xBB,                    // Reserved (2 bytes)
          0x05, 0x03, 0x02, 0x01,        // Timestamp = 0x01020305 (LE)
      },
      .sequence_Number = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .command_Sequence_Number = 0x00,
  };

  sensor_meta sensor = {
      .number = 1,
      .shtp_package = shtp1,
  };
  TEST_ASSERT_EQUAL(0x14, parse_InputReport(&sensor));
  uint32_t t1 = sensor.raw_accelerometer_data.timestamp;
  TEST_ASSERT_EQUAL_UINT32(0x01020304, t1);

  sensor.shtp_package = shtp2;
  TEST_ASSERT_EQUAL(0x14, parse_InputReport(&sensor));
  uint32_t t2 = sensor.raw_accelerometer_data.timestamp;
  TEST_ASSERT_EQUAL_UINT32(0x01020305, t2);

  TEST_ASSERT_TRUE(t2 > t1);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_InputReport_checks_for_null);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_GameRotationVector);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_LinearAcceleration);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_CalibratedGyroscope);
  RUN_TEST(test_parse_InputReport_fills_all_acceleration_data);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_RawAccelerometer);
  RUN_TEST(test_parse_InputReport_raw_accel_timestamp_is_monotonic);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_RawGyroscope);
  RUN_TEST(test_parse_InputReport_fills_sensor_struct_for_RawMagnetometer);
  UNITY_END();
}
