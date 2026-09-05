//
// Created by ch on 24-11-23.
//
#pragma once

#include <ros/ros.h>

#include "engineer_custom_controller/common/data.h"
#include "engineer_custom_controller/custom_controller_data_sender.h"

namespace engineer_custom_controller
{
class EngineerCustomController
{
public:
  explicit EngineerCustomController(ros::NodeHandle& nh) : custom_controller_data_sender(nh, base_), last_get_data_time_(ros::Time::now())
  {
    ROS_INFO("Engineer custom controller load.");
    robot_custom_data_pub_ = nh.advertise<rm_msgs::RobotCustomData>("robot_custom_data", 1);

    base_.initSerial();
  }
  void read();
  void clearRxBuffer()
  {
    rx_buffer_.clear();
    rx_len_ = 0;
  }

  ros::Publisher robot_custom_data_pub_;

  Base base_;
  CustomControllerDataSender custom_controller_data_sender;
  std::vector<uint8_t> rx_buffer_;
  int rx_len_{};

private:
  int unpack(uint8_t* rx_data);
  ros::Time last_get_data_time_;
  const int k_frame_length_ = 64, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
  const int k_unpack_buffer_length_ = 128;
  uint8_t unpack_buffer_[128]{};
};
}  // namespace engineer_custom_controller
