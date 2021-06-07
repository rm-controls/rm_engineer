//
// Created by qiayuan on 5/30/21.
//

#ifndef ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_
#define ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_

#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <rm_common/ori_tool.h>
namespace engineer_middleware {
class ChassisInterface {
 public:
  explicit ChassisInterface(ros::NodeHandle &nh) {
    tf_listener_ = new tf2_ros::TransformListener(this->tf_);
    ros::NodeHandle nh_base_motion = ros::NodeHandle(nh, "chassis");
    ros::NodeHandle nh_pid_x = ros::NodeHandle(nh_base_motion, "x");
    ros::NodeHandle nh_pid_y = ros::NodeHandle(nh_base_motion, "y");
    ros::NodeHandle nh_pid_w = ros::NodeHandle(nh_base_motion, "yaw");
    pid_x_.init(ros::NodeHandle(nh_pid_x, "pid"));
    pid_y_.init(ros::NodeHandle(nh_pid_y, "pid"));
    pid_yaw_.init(ros::NodeHandle(nh_pid_w, "pid"));
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }
  bool setGoal(const geometry_msgs::PoseStamped &pose) {
    goal_ = pose;
    return true;
  };
  bool setCurrent2Gola() {
    goal_ = current_;
    return true;
  }
  void update() {
    geometry_msgs::TransformStamped chassis_transformStamped;
    double roll{}, pitch{};
    try {
      chassis_transformStamped = tf_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
    }
    current_.pose.position.x = chassis_transformStamped.transform.translation.x;
    current_.pose.position.y = chassis_transformStamped.transform.translation.y;
    quatToRPY(chassis_transformStamped.transform.rotation, roll, pitch, current_yaw_);
  }
  double getPosError() const {
    return sqrt(
        std::pow((goal_.pose.position.x - current_.pose.position.x), 2) +
            std::pow((goal_.pose.position.y - current_.pose.position.y), 2));
  }
  double getYawError() const {
    return goal_yaw_ - current_yaw_;
  }
  void run(ros::Duration period) {
    geometry_msgs::Twist cmd_vel_{};
    double goal_roll, goal_pitch;
    quatToRPY(goal_.pose.orientation, goal_roll, goal_pitch, goal_yaw_);
    pid_x_.computeCommand(goal_.pose.position.x - current_.pose.position.x, period);
    pid_y_.computeCommand(goal_.pose.position.y - current_.pose.position.y, period);
    pid_yaw_.computeCommand(goal_yaw_ - current_yaw_, period);
    cmd_vel_.linear.x = pid_x_.getCurrentCmd();
    cmd_vel_.linear.y = pid_y_.getCurrentCmd();
    cmd_vel_.angular.z = pid_yaw_.getCurrentCmd();
    vel_pub_.publish(cmd_vel_);
  }
 private:
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;
  geometry_msgs::PoseStamped goal_{}, current_{};
  double current_yaw_, goal_yaw_;
  ros::Publisher vel_pub_;

};
}
#endif //ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_
