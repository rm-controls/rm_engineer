//
// Created by astro on 2021/4/24.
//

#ifndef SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_
#define SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_

#include <queue>
//ROS
#include <control_toolbox/pid.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/EngineerAction.h>
#include <geometry_msgs/Twist.h>

namespace engineer_middleware {
struct BaseMotionDate {
  double x;
  double y;
  double yaw;
  double pitch;
  bool MiddlewareControl;
};
class BaseController {
 public:
  explicit BaseController(ros::NodeHandle &nh);
  ros::NodeHandle nh_;

  void baseControllerThread();
  void sendDataToBaseController(BaseMotionDate &send_data);
  void move();
  void setChassis(uint8_t chassis_mode, double linear_x, double linear_y);
  void setGimbal(uint8_t chassis_mode, double yaw, double pitch);
 private:
  control_toolbox::Pid pid_x_, pid_y_;
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  BaseMotionDate chassis_data_{};
  std::queue<BaseMotionDate> data_buffer_;
  std::mutex data_buffer_mutex_;
};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_BASE_MOTION_H_
