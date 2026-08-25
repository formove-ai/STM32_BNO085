#include "imu_reader.h"

uint8_t imu_reader_start(sensor_meta *sensor, uint16_t report_interval_ms) {
  hardreset_IMU(sensor);

  if (clear_init_Message_IMU(sensor) == D_ERR) {
    return D_ERR;
  }

  return enable_GameRotationVector(sensor, report_interval_ms);
}

bool imu_reader_read(sensor_meta *sensor, quaternion_sample *sample) {
  if (!data_available(sensor)) {
    return false;
  }

  sample->i = get_Quat_I(sensor);
  sample->j = get_Quat_J(sensor);
  sample->k = get_Quat_K(sensor);
  sample->real = get_Quat_Real(sensor);
  sample->accuracy = get_Quat_Accuracy(sensor);
  return true;
}
