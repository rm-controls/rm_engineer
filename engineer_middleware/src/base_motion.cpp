//
// Created by astro on 2021/4/24.
//

#include "engineer_middleware/base_motion.h"
namespace engineer_middleware {

BaseController::BaseController(ros::NodeHandle &nh) : nh_(nh) {
  ros::NodeHandle nh_base_motion = ros::NodeHandle(nh_, "base_motion");
  ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_base_motion, "x");
  ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_base_motion, "y");
  ros::NodeHandle nh_pid_z = ros::NodeHandle(nh_base_motion, "z");
  ros::NodeHandle nh_pid_yaw = ros::NodeHandle(nh_base_motion, "yaw");
  ros::NodeHandle nh_pid_pitch = ros::NodeHandle(nh_base_motion, "pitch");
  pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
  pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
  pid_z_.init(ros::NodeHandle(nh_pid_z, "pid"));
  pid_yaw_.init(ros::NodeHandle(nh_pid_yaw, "pid"));
  pid_pitch_.init(ros::NodeHandle(nh_pid_pitch, "pid"));
  ros::NodeHandle root_nh;
  vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
  gimbal_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_gimbal", 1);
  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
}
void BaseController::move(ros::Duration period) {
  BaseMotionDate current_position{};
  current_position = getCurrent();
  pid_x_.computeCommand(chassis_data_.x - current_position.x, period);
  pid_y_.computeCommand(chassis_data_.y - current_position.y, period);
  pid_z_.computeCommand(chassis_data_.z - current_position.z, period);
  pid_yaw_.computeCommand(chassis_data_.yaw - current_position.yaw, period);
  pid_pitch_.computeCommand(chassis_data_.pitch - current_position.pitch, period);

  setChassis(chassis_cmd_.RAW, pid_x_.getCurrentCmd(), pid_y_.getCurrentCmd(), pid_z_.getCurrentCmd());
}
void BaseController::setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z) {
  chassis_cmd_.mode = chassis_mode;
  chassis_cmd_.accel.linear.x = 5;
  chassis_cmd_.accel.linear.y = 5;
  chassis_cmd_.accel.angular.z = 5;
  chassis_cmd_.power_limit = 99;
  chassis_cmd_.accel.angular.x = 5;
  chassis_cmd_.accel.angular.y = 5;
  chassis_cmd_.accel.angular.z = 5;
  cmd_vel_.linear.x = linear_x;
  cmd_vel_.linear.y = linear_y;
  cmd_vel_.angular.z = angular_z;
  chassis_cmd_pub_.publish(chassis_cmd_);
  vel_cmd_pub_.publish(cmd_vel_);
}
void BaseController::baseControllerThread() {
  ROS_INFO("start chassis thread");
  ros::Rate loop_rate(100);
  ros::Time last = ros::Time::now();
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(data_buffer_mutex_);
    if (data_buffer_.empty()) {
      if (chassis_data_.MiddlewareControl)
        move(ros::Time::now() - last);
    } else {
      chassis_data_ = data_buffer_.back();
      data_buffer_.pop();
      if (chassis_data_.MiddlewareControl)
        move(ros::Time::now() - last);
    }
    last = ros::Time::now();
    loop_rate.sleep();
  }
}
void BaseController::setGimbal(uint8_t chassis_mode, double yaw, double pitch) {

}
BaseMotionDate BaseController::getCurrent() {
  BaseMotionDate current_position{};
  geometry_msgs::TransformStamped chassis_transformStamped;
  geometry_msgs::TransformStamped gimbal_transformStamped;
  double roll{}, pitch{}, yaw{};
  try {
    chassis_transformStamped = this->tf_.lookupTransform("odom", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_WARN("%s", ex.what());
  }
  try {
    gimbal_transformStamped = this->tf_.lookupTransform("odom", "pitch", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    //ROS_WARN("%s", ex.what());
  }
  quatToRPY(gimbal_transformStamped.transform.rotation, roll, pitch, yaw);
  current_position.yaw = yaw;
  current_position.pitch = pitch;
  current_position.x = chassis_transformStamped.transform.translation.x;
  current_position.y = chassis_transformStamped.transform.translation.y;
  quatToRPY(chassis_transformStamped.transform.rotation, roll, pitch, yaw);
  current_position.z = yaw;
  return current_position;
}
}
