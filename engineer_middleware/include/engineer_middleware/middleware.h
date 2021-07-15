//
// Created by astro on 2021/4/25.
//

#ifndef ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

#include "engineer_middleware/step_queue.h"

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <rm_msgs/EngineerAction.h>

namespace engineer_middleware {
class Middleware {
 public:
  explicit Middleware(ros::NodeHandle &nh);
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr &goal) {
    uint8_t id;
    id = goal->step_queue_id;
    is_middleware_control_ = true;
    ROS_INFO("Start step queue id %d", id);
    auto step = step_queues_.find(id);
    if (step != step_queues_.end())
      step->second.run();
    ROS_INFO("Finish step queue id %d", id);
    result_.finish = true;
    is_middleware_control_ = false;
    action_.setSucceeded(result_);
  }
  void run(ros::Duration period) {
    if (is_middleware_control_)
      chassis_interface_.run(period);
  }
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  moveit::planning_interface::MoveGroupInterface arm_group_;
  moveit::planning_interface::MoveGroupInterface hand_group_;
  ChassisInterface chassis_interface_;
  ros::Publisher card_pub_, gimbal_pub_;
  std::unordered_map<uint8_t, StepQueue> step_queues_;
  tf2_ros::Buffer tf_;
  bool is_middleware_control_{};
  rm_msgs::EngineerFeedback feedback_;
  rm_msgs::EngineerResult result_;

};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
