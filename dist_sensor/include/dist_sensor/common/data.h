//
// Created by ch on 25-6-4.
//

#pragma once

#include <ros/ros.h>
#include <unistd.h>
#include <serial/serial.h>

namespace dist_sensor
{
class Base
{
public:
  serial::Serial serial_;
  bool sensor_is_online_ = false;

  void initSerial(const std::string &port)
  {
    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    serial_.setPort(port);
    serial_.setBaudrate(115200);
    serial_.setTimeout(timeout);
    if (serial_.isOpen())
      return;
    try
    {
      serial_.open();
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR("Cannot open port  %s", port.c_str());
    }
  }

  // CheckSum
  uint8_t getChecksum(unsigned char* pch_message, unsigned int dw_length)
  {
    uint8_t checksum = 0;
    while (dw_length--)
    {
      checksum += *pch_message++;
    }
    return checksum;
  }

  uint32_t verifyCheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char expected_checksum;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return 0;
    expected_checksum = getChecksum(pch_message, dw_length - 1);
    return (expected_checksum == pch_message[dw_length - 1]);
  }

  void appendCheckSum(unsigned char* pch_message, unsigned int dw_length)
  {
    unsigned char checksum;
    if ((pch_message == nullptr) || (dw_length <= 2))
      return;
    checksum = getChecksum(pch_message, dw_length - 1);
    pch_message[dw_length - 1] = checksum;
  }
};

class SensorData
{
public:
  explicit SensorData(ros::NodeHandle& nh, const std::string& sensor_name, const std::string& port)
  : dist_lower_(nh.param("dist_lower", 0.2)),
    dist_upper_(nh.param("dist_upper", 2.0))
  {
    base_.initSerial(port);
    publisher_ = nh.advertise<rm_msgs::EngineerDistSensor>(sensor_name, 1);
  }
  void setDistance(const double distance)
  {
    if (getAmp() >= 200 && getAmp() <= 60000 && distance >= dist_lower_ && distance <= dist_upper_)
    {
      dist_sensor_data_.distance = distance;
      error_ = false;
    }
    else
      error_ = true;
  }
  void setAmp(const double amp)
  {
    dist_sensor_data_.amp = amp;
  }
  void setTemperature(const double temperature)
  {
    dist_sensor_data_.temperature = temperature;
  }
  void publish()
  {
    publisher_.publish(dist_sensor_data_);
  }
  double getDistance()
  {
    return dist_sensor_data_.distance;
  }
  double getAmp()
  {
      return dist_sensor_data_.amp;
  }
  bool getError()
  {
      return error_;
  }
  Base base_;
  uint8_t unpack_buffer_[256]{};
private:
  double dist_lower_, dist_upper_;
  ros::Publisher publisher_;
  rm_msgs::EngineerDistSensor dist_sensor_data_;
  bool error_ = true;
};
}  // namespace dist_sensor
