// Verifies that the generated mock in mock/ is usable by a downstream project:
// application code (imu_reader) is linked against MockBNO085_SPI_Library
// instead of the real library, with no HAL and no Ruby involved.

#include <unity.h>

#include "MockBNO085_SPI_Library.h"
#include "imu_reader.h"

static sensor_meta sensor;

void setUp(void) { MockBNO085_SPI_Library_Init(); }

void tearDown(void) {
  MockBNO085_SPI_Library_Verify();
  MockBNO085_SPI_Library_Destroy();
}

void test_start_resets_then_enables_game_rotation_vector(void) {
  hardreset_IMU_Expect(&sensor);
  clear_init_Message_IMU_ExpectAndReturn(&sensor, N_ERR);
  enable_GameRotationVector_ExpectAndReturn(&sensor, 10, N_ERR);

  TEST_ASSERT_EQUAL_UINT8(N_ERR, imu_reader_start(&sensor, 10));
}

// Only the two calls that precede the failure are expected, so the mock fails
// the test if imu_reader_start keeps going after an error.
void test_start_stops_when_clearing_the_init_message_fails(void) {
  hardreset_IMU_Expect(&sensor);
  clear_init_Message_IMU_ExpectAndReturn(&sensor, D_ERR);

  TEST_ASSERT_EQUAL_UINT8(D_ERR, imu_reader_start(&sensor, 10));
}

void test_read_reports_no_sample_while_no_data_is_available(void) {
  quaternion_sample sample = {0};

  data_available_ExpectAndReturn(&sensor, false);

  TEST_ASSERT_FALSE(imu_reader_read(&sensor, &sample));
}

void test_read_copies_the_quaternion_out_of_the_library(void) {
  quaternion_sample sample = {0};

  data_available_ExpectAndReturn(&sensor, true);
  get_Quat_I_ExpectAndReturn(&sensor, 0.25f);
  get_Quat_J_ExpectAndReturn(&sensor, 0.5f);
  get_Quat_K_ExpectAndReturn(&sensor, 0.75f);
  get_Quat_Real_ExpectAndReturn(&sensor, 1.0f);
  get_Quat_Accuracy_ExpectAndReturn(&sensor, 3);

  TEST_ASSERT_TRUE(imu_reader_read(&sensor, &sample));
  TEST_ASSERT_EQUAL_FLOAT(0.25f, sample.i);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, sample.j);
  TEST_ASSERT_EQUAL_FLOAT(0.75f, sample.k);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, sample.real);
  TEST_ASSERT_EQUAL_UINT8(3, sample.accuracy);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_start_resets_then_enables_game_rotation_vector);
  RUN_TEST(test_start_stops_when_clearing_the_init_message_fails);
  RUN_TEST(test_read_reports_no_sample_while_no_data_is_available);
  RUN_TEST(test_read_copies_the_quaternion_out_of_the_library);
  return UNITY_END();
}
