//
// Created by ch on 25-6-4.
//

#pragma once

#include <ros/ros.h>
#include <rm_msgs/EngineerDistSensor.h>
#include <rm_common/filters/lp_filter.h>
#include <std_msgs/Float64.h>

#include "dist_sensor/common/data.h"
#include "dist_sensor/common/protocol.h"

namespace dist_sensor
{
  class DistSensor
  {
  public:
    explicit DistSensor(ros::NodeHandle& nh) :
    sensor_l_(nh, "dist_sensor_l", "/dev/usbDistSensorLeft"),
    sensor_r_(nh, "dist_sensor_r", "/dev/usbDistSensorRight"),
    last_get_data_time_(ros::Time::now())
    {
      sensor_interval_ = nh.param("sensor_interval", 0.6);
      yaw_angle_pub_ = nh.advertise<std_msgs::Float64>("yaw_angle",1);
      ROS_INFO("Dist sensor load.");
    }

    void read(SensorData& sensor);
    void clearRxBuffer()
    {
      rx_buffer_.clear();
      rx_len_ = 0;
    }
    void calculateYawAngle()
    {
      std_msgs::Float64 yaw_angle;
      if (sensor_l_.getError() || sensor_r_.getError())
        yaw_angle.data = 0;
      else
        yaw_angle.data = atan((sensor_r_.getDistance() - sensor_l_.getDistance()) / sensor_interval_);
      yaw_angle_pub_.publish(yaw_angle);
    }
    SensorData sensor_l_, sensor_r_;
    std::vector<uint8_t> rx_buffer_;
    int rx_len_;

  private:
    int unpack(uint8_t* rx_data, SensorData& sensor);
    ros::Time last_get_data_time_;
    ros::Publisher yaw_angle_pub_;
    double sensor_interval_;
    const int k_frame_length_ = 128, k_header_length_ = 2, k_data_length_ = 6, k_tail_length_ = 1;
    const int k_unpack_buffer_length_ = 256;
  };
}  // namespace dist_sensor
