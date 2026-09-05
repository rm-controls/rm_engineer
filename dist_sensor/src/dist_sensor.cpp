//
// Created by ch on 25-6-4.
//

#include "dist_sensor/dist_sensor.h"

namespace dist_sensor
{
void DistSensor::read(SensorData& sensor)
{
  if (sensor.base_.serial_.available())
  {
    rx_len_ = static_cast<int>(sensor.base_.serial_.available());
    sensor.base_.serial_.read(rx_buffer_, rx_len_);
  }
  else
    return;
  uint8_t temp_buffer[256] = { 0 };
  if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
    sensor.base_.sensor_is_online_ = false;
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = sensor.unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      sensor.unpack_buffer_[k_i] = temp_buffer[k_i];
  }
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (sensor.unpack_buffer_[k_i] == 0x59 && sensor.unpack_buffer_[k_i + 1] == 0x59)
    {
      int frame_len = unpack(&sensor.unpack_buffer_[k_i], sensor);
      if (frame_len != -1)
        k_i += frame_len;
    }
  }
  clearRxBuffer();
}

int DistSensor::unpack(uint8_t* rx_data, SensorData& sensor)
{
  int frame_len = k_header_length_ + k_data_length_ + k_tail_length_;
  if (sensor.base_.verifyCheckSum(rx_data, frame_len) == 1)
  {
    dist_sensor::DistanceSensorData dist_sensor_data_ref;
    memcpy(&dist_sensor_data_ref, rx_data + 2, sizeof(dist_sensor::DistanceSensorData));
    sensor.setDistance((dist_sensor_data_ref.dist[1] << 8 | dist_sensor_data_ref.dist[0]) / 1000.0);
    sensor.setAmp(dist_sensor_data_ref.amp[1] << 8 | dist_sensor_data_ref.amp[0]);
    sensor.setTemperature((dist_sensor_data_ref.temp[1] << 8 | dist_sensor_data_ref.temp[0]) / 8.0 - 256);
    sensor.publish();

    sensor.base_.sensor_is_online_ = true;
    last_get_data_time_ = ros::Time::now();
    return frame_len;
  }
  return -1;
}

}  // namespace dist_sensor