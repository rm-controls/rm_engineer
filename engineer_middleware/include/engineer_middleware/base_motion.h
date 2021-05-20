//
// Created by astro on 2021/4/24.
//

#ifndef ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_
#define ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_

#include <queue>
//ROS
#include <control_toolbox/pid.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <rm_common/ori_tool.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/EngineerAction.h>
#include <geometry_msgs/Twist.h>

namespace engineer_middleware {

class BaseMotion {
 public:
  explicit BaseMotion(ros::NodeHandle &nh);
  ros::NodeHandle nh_;
  void baseControllerThread();
  void execute(ros::Duration period);
  void setMiddlewareControl(bool is_middleware_control) { middleware_control_ = is_middleware_control; }
  bool chassisMoveTo(double x, double y, double z);
  bool gimbalMoveTo(double yaw, double pitch);
  void updateCurrent();
  void setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z);
  void setGimbal(uint8_t chassis_mode, double yaw, double pitch);
  void setChassisPosition(double x, double y, double z) {
    std::unique_lock<std::mutex> lock(data_buffer_mutex_);
    expect_x_ = x;
    expect_y_ = y;
    expect_z_ = z;
  }
  void setGimbalPosition(double yaw, double pitch) {
    std::unique_lock<std::mutex> lock(data_buffer_mutex_);
    expect_x_ = yaw;
    expect_pitch_ = pitch;
  }
 private:
  control_toolbox::Pid pid_x_, pid_y_, pid_z_, pid_yaw_, pid_pitch_;
  double chassis_error_, gimbal_error_;
  bool middleware_control_;
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  ros::Publisher gimbal_cmd_pub_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  double expect_x_, expect_y_, expect_z_, expect_yaw_, expect_pitch_;
  double current_x_, current_y_, current_z_, current_yaw_, current_pitch_;
  std::mutex data_buffer_mutex_;
};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_
