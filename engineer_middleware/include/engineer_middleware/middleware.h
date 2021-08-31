//
// Created by astro on 2021/4/25.
//

#ifndef ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

#include "engineer_middleware/step_queue.h"

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <rm_msgs/EngineerAction.h>

namespace engineer_middleware
{
class Middleware
{
public:
  explicit Middleware(ros::NodeHandle& nh);
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr& goal)
  {
    std::string name;
    name = goal->step_queue_name;
    is_middleware_control_ = true;
    ROS_INFO("Start step queue id %s", name.c_str());
    auto step_queue = step_queues_.find(name);
    if (step_queue != step_queues_.end())
      step_queue->second.run(as_);
    ROS_INFO("Finish step queue id %s", name.c_str());
    is_middleware_control_ = false;
  }
  void run(ros::Duration period)
  {
    if (is_middleware_control_)
      chassis_interface_.run(period);
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> as_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  ChassisInterface chassis_interface_;
  ros::Publisher hand_pub_, card_pub_, gimbal_pub_;
  std::unordered_map<std::string, StepQueue> step_queues_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tf_listener_;
  bool is_middleware_control_;
};

}  // namespace engineer_middleware

#endif  // SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
