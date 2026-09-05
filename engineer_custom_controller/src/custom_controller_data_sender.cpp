//
// Created by ljyi on 2026/4/6.
//

#include "engineer_custom_controller/custom_controller_data_sender.h"

namespace engineer_custom_controller
{
void CustomControllerDataSender::pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int data_len)
{
  memset(tx_buffer, 0, k_frame_length_);
  auto* frame_header = reinterpret_cast<FrameHeader*>(tx_buffer);

  frame_header->sof = 0xA5;
  frame_header->data_length = data_len;
  memcpy(&tx_buffer[k_header_length_], reinterpret_cast<uint8_t*>(&cmd_id), k_cmd_id_length_);
  base_.appendCRC8CheckSum(tx_buffer, k_header_length_);
  memcpy(&tx_buffer[k_header_length_ + k_cmd_id_length_], data, data_len);
  base_.appendCRC16CheckSum(tx_buffer, k_header_length_ + k_cmd_id_length_ + data_len + k_tail_length_);
}

void CustomControllerDataSender::sendCustomControllerData(const sensor_msgs::JointState::ConstPtr& data) {
  uint8_t tx_data[sizeof(CustomControllerData)] = { 0 };
  for (size_t i = 0; i < data->name.size(); ++i)
  {
    joint2position_[data->name[i]] = data->position[i];
  }
  std::vector<int16_t> positions;
  for (size_t i = 0; i < joint2position_.size(); ++i) {
    double position = joint2position_[joint_names_[i]];
    positions.push_back(rawData2Int16(position));
  }
  positions.push_back(*button_rt_buffer_.readFromRT());
  memcpy(tx_data, positions.data(), sizeof(int16_t) * positions.size());
  pack(tx_buffer_, tx_data, CUSTOM_CONTROLLER_CMD, sizeof(CustomControllerData));
  tx_len_ = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(CustomControllerData) + k_tail_length_);
  try {
    base_.serial_.write(tx_buffer_, tx_len_);
  } catch (serial::PortNotOpenedException& e) {
    ROS_ERROR_STREAM(e.what());
  }
  if ((*button_rt_buffer_.readFromRT() >> 9) & 0x01 || (*button_rt_buffer_.readFromRT() >> 1) & 0x01) {
    clearTxBuffer();
    SimKeyboardMouseData keyboard_mouse_data;
    int len = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(SimKeyboardMouseData) + k_tail_length_);;
    keyboard_mouse_data.key_value = 0x0059;
    pack(tx_buffer_, reinterpret_cast<uint8_t *>(&keyboard_mouse_data), SIM_KEYBOARD_MOUSE_CMD, sizeof(SimKeyboardMouseData));
    try {
      base_.serial_.write(tx_buffer_, len);
      r_pressed_ = true;
    } catch (serial::PortNotOpenedException& e) {
      ROS_ERROR_STREAM(e.what());
    }
  }
  else if (r_pressed_) {
    clearTxBuffer();
    SimKeyboardMouseData keyboard_mouse_data;
    int len = k_header_length_ + k_cmd_id_length_ + static_cast<int>(sizeof(SimKeyboardMouseData) + k_tail_length_);
    keyboard_mouse_data.key_value = 0x0000;
    pack(tx_buffer_, reinterpret_cast<uint8_t *>(&keyboard_mouse_data), SIM_KEYBOARD_MOUSE_CMD, sizeof(SimKeyboardMouseData));
    try {
      for (int i = 0; i < 5; ++i) {
        base_.serial_.write(tx_buffer_, len);
      }
      r_pressed_ = false;
    } catch (serial::PortNotOpenedException& e) {
      ROS_ERROR_STREAM(e.what());
    }
  }
  clearTxBuffer();
}

int16_t CustomControllerDataSender::rawData2Int16(double value) {
  if (value > maxVal) {
    value = maxVal;
  } else if (value < minVal) {
    value = minVal;
  }
  double norm = (value - minVal) / (maxVal - minVal);
  return static_cast<int16_t>(norm * 65535.0 - 32768.0);
}

void CustomControllerDataSender::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data) {
  sendCustomControllerData(data);
  last_send_time_ = ros::Time::now();
}

void CustomControllerDataSender::ecatDataCallback(const rm_ecat_msgs::RmEcatMitSlaveReadings::ConstPtr &data) {
  double raw_button_l = (data->readings[1].velocity[0] + 45.0) / (45. * 2. / 4095) + 0.00001;
  double raw_button_r = (data->readings[1].velocity[1] + 45.0) / (45. * 2. / 4095) + 0.00001;
  int16_t button_data = (static_cast<uint8_t>(raw_button_l) << 8) | static_cast<uint8_t>(raw_button_r);
  button_rt_buffer_.writeFromNonRT(button_data);
}
}  // namespace engineer_custom_controller
