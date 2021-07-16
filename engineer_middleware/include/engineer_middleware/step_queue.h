//
// Created by qiayuan on 4/3/21.
//

#ifndef ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#define ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#pragma once

#include "engineer_middleware/step.h"

// STL
#include <deque>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <rm_msgs/EngineerAction.h>

namespace engineer_middleware {
class StepQueue {
 public:
  StepQueue(const XmlRpc::XmlRpcValue &steps, tf2_ros::Buffer &tf,
            moveit::planning_interface::MoveGroupInterface &arm_group,
            moveit::planning_interface::MoveGroupInterface &hand_group,
            ChassisInterface &chassis_interface, ros::Publisher &card_pub, ros::Publisher &gimbal_pub) :
      chassis_interface_(chassis_interface) {
    ROS_ASSERT(steps.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < steps.size(); ++i)
      queue_.emplace_back(steps[i], tf, arm_group, hand_group, chassis_interface, card_pub, gimbal_pub);
  }
  bool run(actionlib::SimpleActionServer<rm_msgs::EngineerAction> &as) {
    if (queue_.empty()) {
      ROS_WARN("step is empty");
      return false;
    }
    geometry_msgs::PoseStamped current;
    current.header.frame_id = "base_link";
    current.pose.orientation.w = 1.;
    chassis_interface_.setGoal(current);

    rm_msgs::EngineerFeedback feedback;
    rm_msgs::EngineerResult result;

    feedback.total_steps = queue_.size();
    for (size_t i = 0; i < queue_.size(); ++i) {
      ros::Time start = ros::Time::now();
      if (!queue_[i].move())
        return false;
      ROS_INFO("Start step: %s", queue_[i].getName().c_str());
      while (!queue_[i].isFinish()) {
        if (!queue_[i].checkTimeout(ros::Time::now() - start)) {
          queue_[i].stop();
          return false;
        }
        if (as.isPreemptRequested() || !ros::ok()) {
          ROS_INFO("%s: Preempted", queue_[i].getName().c_str());
          as.setPreempted();
          return false;
        }
        feedback.finished_step = i;
        feedback.current_step = queue_[i].getName();
        as.publishFeedback(feedback);
        ros::WallDuration(0.2).sleep();
      }
      feedback.finished_step = queue_.size();
      as.publishFeedback(feedback);
      ROS_INFO("Finish step: %s", queue_[i].getName().c_str());
    }
    result.finish = true;
    as.setSucceeded(result);
    return true;
  }
  const std::deque<Step> &getQueue() const { return queue_; }
  std::deque<Step>::size_type size() const { return queue_.size(); }
 private:
  std::deque<Step> queue_;
  ChassisInterface &chassis_interface_;
};
}

#endif //ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
