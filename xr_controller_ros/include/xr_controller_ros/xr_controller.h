//
// Created by root on 2026/9/18.
//

#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <rm_msgs/XrControllerData.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>
#include <vector>

namespace xr_controller_ros
{
class XrController
{
public:
  explicit XrController(ros::NodeHandle& nh);
  void checkOnOff();
  bool update(const rm_msgs::XrControllerData& data);

private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& data);
  bool captureReference();
  bool solveIkAndPublish(const geometry_msgs::Point&, const geometry_msgs::Quaternion&, const geometry_msgs::Point&,
                         const geometry_msgs::Pose&, const robot_model::JointModelGroup*, const std::string&,
                         ros::Publisher&);
  bool setRobotStateFromJointState();
  bool is_start_ = false;
  bool mark_triggered_ = false, last_mark_ = false;
  bool has_joint_state_ = false, has_reference_ = false;
  ros::Time mark_timing_;
  rm_msgs::XrControllerData control_data_;
  sensor_msgs::JointState joint_state_;
  geometry_msgs::Point left_hand_reference_, right_hand_reference_;
  geometry_msgs::Pose left_ee_reference_, right_ee_reference_;
  ros::Publisher left_control_cmd_pub_, right_control_cmd_pub_;
  ros::Subscriber joint_state_sub_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_model::JointModelGroup* left_arm_group_ = nullptr;
  const robot_model::JointModelGroup* right_arm_group_ = nullptr;
  std::vector<double> last_left_solution_, last_right_solution_;
  bool has_last_left_solution_ = false;
  bool has_last_right_solution_ = false;
  double position_scale_x_ = 1.0, position_scale_y_ = 1.0, position_scale_z_ = 1.0;
  double trajectory_duration_ = 0.1;
  double max_joint_step_ = 0.3;
};
}  // namespace xr_controller_ros
