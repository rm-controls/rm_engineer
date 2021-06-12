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
  bool run() {
    if (queue_.empty()) {
      ROS_WARN("step is empty");
      return false;
    }
    geometry_msgs::PoseStamped current;
    current.header.frame_id = "base_link";
    current.pose.orientation.w = 1.;
    chassis_interface_.setGoal(current);
    for (auto &step:queue_) {
      ros::Time start = ros::Time::now();
      if (!step.move())
        return false;
      while (!step.isFinish()) {
        if (!step.checkTimeout(ros::Time::now() - start)) {
          step.stop();
          return false;
        }
        ros::WallDuration(0.2).sleep();
      }

      ros::WallDuration(1.).sleep();
    }
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
