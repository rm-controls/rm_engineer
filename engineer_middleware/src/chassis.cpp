//
// Created by astro on 2021/4/24.
//

#include "engineer_middleware/chassis.h"
namespace engineer_middleware {

Chassis::Chassis(ros::NodeHandle &nh) : nh_(nh) {
  pid_x_.init(ros::NodeHandle(nh_, "pid_x"));
  pid_y_.init(ros::NodeHandle(nh_, "pid_y"));

  ros::NodeHandle root_nh;
  vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
}
void Chassis::move(double x, double y) {

}
}
