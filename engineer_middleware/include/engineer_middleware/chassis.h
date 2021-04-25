//
// Created by astro on 2021/4/24.
//

#ifndef SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_CHASSIS_H_
#define SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_CHASSIS_H_

//ROS
#include <control_toolbox/pid.h>
#include <ros/ros.h>

#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/EngineerAction.h>
#include <geometry_msgs/Twist.h>
namespace engineer_middleware {
class Chassis {
 public:
  Chassis(ros::NodeHandle &nh);
  void move(double x, double y);
  ros::NodeHandle nh_;
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
 private:
  control_toolbox::Pid pid_x_, pid_y_;

};
}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_CHASSIS_H_
