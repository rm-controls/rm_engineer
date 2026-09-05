//
// Created by ch on 24-11-23.
//
#include "engineer_custom_controller/engineer_custom_controller.h"

namespace engineer_custom_controller
{
void EngineerCustomController::read()
{
  if (base_.serial_.available())
  {
    rx_len_ = static_cast<int>(base_.serial_.available());
    base_.serial_.read(rx_buffer_, rx_len_);
  }
  else
    return;
  uint8_t temp_buffer[128] = { 0 };
  int frame_len;
  if (ros::Time::now() - last_get_data_time_ > ros::Duration(0.1))
    base_.engineer_custom_controller_is_online_ = false;
  // for (int k_i = 0; k_i < rx_len_; ++k_i)
  //   ROS_INFO("%02X", rx_buffer_[k_i]);
  if (rx_len_ < k_unpack_buffer_length_)
  {
    for (int k_i = 0; k_i < k_unpack_buffer_length_ - rx_len_; ++k_i)
      temp_buffer[k_i] = unpack_buffer_[k_i + rx_len_];
    for (int k_i = 0; k_i < rx_len_; ++k_i)
      temp_buffer[k_i + k_unpack_buffer_length_ - rx_len_] = rx_buffer_[k_i];
    for (int k_i = 0; k_i < k_unpack_buffer_length_; ++k_i)
      unpack_buffer_[k_i] = temp_buffer[k_i];
  }
  for (int k_i = 0; k_i < k_unpack_buffer_length_ - k_frame_length_; ++k_i)
  {
    if (unpack_buffer_[k_i] == 0xA5)
    {
      frame_len = unpack(&unpack_buffer_[k_i]);
      if (frame_len != -1)
        k_i += frame_len;
    }
  }
  clearRxBuffer();
}

int EngineerCustomController::unpack(uint8_t* rx_data)
{
  uint16_t cmd_id;
  int frame_len;
  engineer_custom_controller::FrameHeader frame_header;

  memcpy(&frame_header, rx_data, k_header_length_);
  if (static_cast<bool>(base_.verifyCRC8CheckSum(rx_data, k_header_length_)))
  {
    if (frame_header.data_length > 128)  // temporary and inaccurate value
    {
      ROS_INFO("discard possible wrong frames, data length: %d", frame_header.data_length);
      return 0;
    }
    frame_len = frame_header.data_length + k_header_length_ + k_cmd_id_length_ + k_tail_length_;
    if (base_.verifyCRC16CheckSum(rx_data, frame_len) == 1)
    {
      cmd_id = (rx_data[6] << 8 | rx_data[5]);
      switch (cmd_id)
      {
        case engineer_custom_controller::ROBOT_TO_CUSTOM_CONTROLLER_CMD:
        {
          engineer_custom_controller::RobotToCustomData robot_custom_ref;
          rm_msgs::RobotCustomData robot_custom_data;
          memcpy(&robot_custom_ref, rx_data + 7, sizeof(engineer_custom_controller::RobotToCustomData));
          for (int i = 0; i < 30; ++i)
            robot_custom_data.data[i] = robot_custom_ref.data[i];
          robot_custom_data.stamp = last_get_data_time_;
          robot_custom_data_pub_.publish(robot_custom_data);
          break;
        }
        default:
          ROS_WARN("Referee command ID 0x%02X not found.", cmd_id);
          break;
      }
      base_.engineer_custom_controller_is_online_ = true;
      last_get_data_time_ = ros::Time::now();
      return frame_len;
    }
  }
  return -1;
}

}  // namespace engineer_custom_controller
