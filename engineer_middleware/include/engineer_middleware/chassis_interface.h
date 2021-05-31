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
  }
  bool setGoal(const geometry_msgs::PoseStamped &pose) {
    goal_ = pose;
    return true;
  };
  void update() {
    geometry_msgs::TransformStamped chassis_transformStamped;
    geometry_msgs::TransformStamped gimbal_transformStamped;
    double roll{}, pitch{};
    try {
      chassis_transformStamped = tf_.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      //ROS_WARN("%s", ex.what());
    }
    x_ = chassis_transformStamped.transform.translation.x;
    y_ = chassis_transformStamped.transform.translation.y;
    quatToRPY(chassis_transformStamped.transform.rotation, roll, pitch, yaw_);
  }
  double getPosError() const {
    return sqrt(x_ * x_ + y_ * y_);
  }
  double getYawError() const {
    return goal_.pose.orientation.w - yaw_;
  }
 private:
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener *tf_listener_;
  control_toolbox::Pid pid_x_, pid_y_, pid_yaw_;
  geometry_msgs::PoseStamped goal_{}, current_{};
  double x_{}, y_{}, yaw_{};
};
}
#endif //ENGINEER_MIDDLEWARE_CHASSIS_INTERFACE_H_
