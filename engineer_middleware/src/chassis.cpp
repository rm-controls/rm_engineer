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

  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
}
void Chassis::move() {
  geometry_msgs::TransformStamped chassis_transformStamped;
  double current_x, current_y;
  try {
    chassis_transformStamped = this->tf_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_WARN("%s", ex.what());
  }
  current_x = chassis_transformStamped.transform.translation.x;
  current_y = chassis_transformStamped.transform.translation.y;
  pid_x_.computeCommand(expect_x_ - current_x, ros::Duration(0.01));
  pid_y_.computeCommand(expect_y_ - current_y, ros::Duration(0.01));;
  setChassis(chassis_cmd_.RAW, pid_x_.getCurrentCmd(), pid_y_.getCurrentCmd(), 0.0);
}
void Chassis::setExpectPosition(double expect_x, double expect_y) {
  expect_x_ = expect_x;
  expect_y_ = expect_y;
}
void Chassis::setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z) {
  chassis_cmd_.mode = chassis_mode;
  chassis_cmd_.accel.linear.x = 5;
  chassis_cmd_.accel.linear.y = 5;
  chassis_cmd_.accel.angular.z = 5;
  chassis_cmd_.effort_limit = 99;
  chassis_cmd_.accel.angular.x = 5;
  chassis_cmd_.accel.angular.y = 5;
  chassis_cmd_.accel.angular.z = 5;
  cmd_vel_.linear.x = linear_x;
  cmd_vel_.linear.y = linear_y;
  cmd_vel_.linear.z = angular_z;
  chassis_cmd_pub_.publish(chassis_cmd_);
  vel_cmd_pub_.publish(cmd_vel_);
}
}
