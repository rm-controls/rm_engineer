//
// Created by astro on 2021/4/25.
//

#ifndef ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
#define ENGINEER_MIDDLEWARE_MIDDLEWARE_H_

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/SwitchController.h>

#include <rm_msgs/EngineerAction.h>
#include "engineer_middleware/step_queue.h"
#include "engineer_middleware/chassis_interface.h"
#include "rm_msgs/GimbalCmd.h"
namespace engineer_middleware {
class Middleware {
 public:
  explicit Middleware(ros::NodeHandle &nh);
  ros::NodeHandle nh_;
  //action
  actionlib::SimpleActionServer<rm_msgs::EngineerAction> action_;
  rm_msgs::EngineerFeedback feedback;
  rm_msgs::EngineerResult result_;
  void switchController(const std::string &start_controller, const std::string &stop_controller);
 private:
  //server client
  ros::ServiceClient switch_controller_client_;
  //arm
  moveit::planning_interface::MoveGroupInterface *arm_group_;
  moveit::planning_interface::MoveGroupInterface *hand_group_;
  engineer_middleware::StepQueue *step_queue_;
  XmlRpc::XmlRpcValue *steps_params_;
  void executeCB(const actionlib::SimpleActionServer<rm_msgs::EngineerAction>::GoalConstPtr &goal) {
    std::string step_name;
    step_name = goal->step;

    result_.finish = true;
    action_.setSucceeded(result_);
  }
  ros::Publisher vel_cmd_pub_;
  ros::Publisher gimbal_cmd_pub_;
  tf2_ros::Buffer tf_;
};

}

#endif //SRC_RM_SOFTWARE_RM_ENGINEER_ENGINEER_MIDDLEWARE_INCLUDE_ENGINEER_MIDDLEWARE_MIDDLEWARE_H_
