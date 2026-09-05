//
// Created by ljyi on 2026/4/6.
//

#pragma once

#include <ros/ros.h>
#include <cstring>
#include <realtime_tools/realtime_buffer.h>

#include "common/data.h"
#include "common/protocol.h"

namespace engineer_custom_controller
{
class CustomControllerDataSender
{
public:
  explicit CustomControllerDataSender(ros::NodeHandle& nh, Base& base) : base_(base)
  {
    ROS_INFO("Custom data sender load.");
    joint_names_ = nh.param("joint_names", std::vector<std::string>());
    maxVal = nh.param("maxVal", 12.56);
    minVal = nh.param("minVal", -12.56);
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CustomControllerDataSender::jointStateCallback, this);
    ecat_data_sub_ = nh.subscribe<rm_ecat_msgs::RmEcatMitSlaveReadings>("/rm_ecat_hw/mit_readings", 1, &CustomControllerDataSender::ecatDataCallback, this);
  }
  virtual ~CustomControllerDataSender() = default;

private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& data);
  void ecatDataCallback(const rm_ecat_msgs::RmEcatMitSlaveReadings::ConstPtr& data);
  void sendCustomControllerData(const sensor_msgs::JointState::ConstPtr& data);
  void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len);
  void clearTxBuffer()
  {
    for (int i = 0; i < k_frame_length_; i++)
      tx_buffer_[i] = 0;
    tx_len_ = 0;
  }

  int16_t rawData2Int16(double value);

  double maxVal, minVal;
  std::vector<std::string> joint_names_{};
  std::map<std::string, double> joint2position_{};
  ros::Subscriber joint_state_sub_, ecat_data_sub_;
  ros::Time last_send_time_{};
  Base& base_;
  uint8_t tx_buffer_[128]{};
  realtime_tools::RealtimeBuffer<int16_t> button_rt_buffer_{};
  int tx_len_{};
  bool r_pressed_ = false;

  const int k_frame_length_ = 128, k_header_length_ = 5, k_cmd_id_length_ = 2, k_tail_length_ = 2;
};

}  // namespace engineer_custom_controller
