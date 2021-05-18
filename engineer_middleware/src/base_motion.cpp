//
// Created by astro on 2021/4/24.
//

#include "engineer_middleware/base_motion.h"
namespace engineer_middleware {

BaseController::BaseController(ros::NodeHandle &nh) : nh_(nh) {
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
void BaseController::move() {
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
  pid_x_.computeCommand(chassis_data_.x - current_x, ros::Duration(0.01));
  pid_y_.computeCommand(chassis_data_.y - current_y, ros::Duration(0.01));;
  setChassis(chassis_cmd_.RAW, pid_x_.getCurrentCmd(), pid_y_.getCurrentCmd());
}
void BaseController::setChassis(uint8_t chassis_mode, double linear_x, double linear_y) {
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
  chassis_cmd_pub_.publish(chassis_cmd_);
  vel_cmd_pub_.publish(cmd_vel_);
}
void BaseController::baseControllerThread() {
  ROS_INFO("start chassis thread");
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(data_buffer_mutex_);
    if (data_buffer_.empty()) {
      if (chassis_data_.MiddlewareControl)
        move();
    } else {
      chassis_data_ = data_buffer_.back();
      data_buffer_.pop();
    }
    loop_rate.sleep();
  }
}
void BaseController::sendDataToBaseController(BaseMotionDate &send_data) {
  std::unique_lock<std::mutex> lock(data_buffer_mutex_);
  data_buffer_.push(send_data);
}
void BaseController::setGimbal(uint8_t chassis_mode, double yaw, double pitch) {

}
}
