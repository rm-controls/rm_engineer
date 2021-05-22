//
// Created by astro on 2021/4/24.
//

#include "engineer_middleware/base_motion.h"
namespace engineer_middleware {

BaseMotion::BaseMotion(ros::NodeHandle &nh) : nh_(nh) {
  ros::NodeHandle nh_base_motion = ros::NodeHandle(nh_, "base_motion");
  ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_base_motion, "x");
  ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_base_motion, "y");
  ros::NodeHandle nh_pid_z = ros::NodeHandle(nh_base_motion, "z");
  ros::NodeHandle nh_pid_yaw = ros::NodeHandle(nh_base_motion, "yaw");
  ros::NodeHandle nh_pid_pitch = ros::NodeHandle(nh_base_motion, "pitch");
  pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
  pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
  pid_w_.init(ros::NodeHandle(nh_pid_z, "pid"));
  pid_yaw_.init(ros::NodeHandle(nh_pid_yaw, "pid"));
  pid_pitch_.init(ros::NodeHandle(nh_pid_pitch, "pid"));
  ros::NodeHandle root_nh;
  vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
  gimbal_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_gimbal", 1);
  this->tf_listener_ = new tf2_ros::TransformListener(this->tf_);
  middleware_control_ = false;
  chassis_error_ = getParam(nh_, "base_motion/chassis_error", 0.2);
  gimbal_error_ = getParam(nh_, "base_motion/gimbal_error", 0.2);
}

void BaseMotion::execute(ros::Duration period) {
  updateCurrent();
  pid_x_.computeCommand(expect_x_ - current_x_, period);
  pid_y_.computeCommand(expect_y_ - current_y_, period);
  pid_w_.computeCommand(expect_w_ - current_w_, period);
  pid_yaw_.computeCommand(expect_yaw_ - current_yaw_, period);
  pid_pitch_.computeCommand(expect_pitch_ - current_pitch_, period);
  setChassis(chassis_cmd_.RAW, pid_x_.getCurrentCmd(), pid_y_.getCurrentCmd(), pid_w_.getCurrentCmd());
}
bool BaseMotion::chassisMoveTo(double x, double y, double z) {
  setChassisPosition(x, y, z);
  while (std::abs(current_x_ - expect_x_) > chassis_error_
      && std::abs(current_y_ - expect_y_) > chassis_error_) {
    updateCurrent();
    ros::Duration(0.01).sleep();
  }
  return true;
}
bool BaseMotion::gimbalMoveTo(double yaw, double pitch) {
  setGimbalPosition(yaw, pitch);
  while (std::abs(current_x_ - expect_x_) > gimbal_error_
      && std::abs(current_y_ - expect_y_) > gimbal_error_) {
    updateCurrent();
    ros::Duration(0.01).sleep();
  }
  return true;
}
void BaseMotion::setChassis(uint8_t chassis_mode, double linear_x, double linear_y, double angular_z) {
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
void BaseMotion::baseControllerThread() {
  ROS_INFO("start chassis thread");
  ros::Rate loop_rate(100);
  ros::Time last = ros::Time::now();
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(data_buffer_mutex_);
    if (middleware_control_)
      execute(ros::Time::now() - last);
    last = ros::Time::now();
    loop_rate.sleep();
  }
}
void BaseMotion::setGimbal(uint8_t chassis_mode, double yaw, double pitch) {

}
void BaseMotion::updateCurrent() {
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
  current_yaw_ = yaw;
  current_pitch_ = pitch;
  current_x_ = chassis_transformStamped.transform.translation.x;
  current_y_ = chassis_transformStamped.transform.translation.y;
  quatToRPY(chassis_transformStamped.transform.rotation, roll, pitch, yaw);
  current_w_ = yaw;
}

}
