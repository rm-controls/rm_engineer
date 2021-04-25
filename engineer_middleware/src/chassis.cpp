//
// Created by astro on 2021/4/24.
//

#include "engineer_middleware/chassis.h"
namespace engineer_middleware {

Chassis::Chassis(ros::NodeHandle &nh) : nh_(nh) {
  ros::NodeHandle nh_chassis = ros::NodeHandle(nh_, "chassis");
  ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_chassis, "x");
  ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_chassis, "y");
  pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
  pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));

  ros::NodeHandle root_nh;
  vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
}
void Chassis::move(double x, double y) {

}
}
