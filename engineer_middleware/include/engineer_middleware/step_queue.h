//
// Created by qiayuan on 4/3/21.
//

#ifndef ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#define ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#pragma once

// STL
#include <deque>
#include <string>

#include "engineer_middleware/step.h"

namespace engineer_middleware {
class StepQueue {
 public:
  StepQueue(const XmlRpc::XmlRpcValue &steps,
            moveit::planning_interface::MoveGroupInterface &arm_group,
            moveit::planning_interface::MoveGroupInterface &hand_group,
            BaseMotion *base_motion);

  bool move();

  const std::deque<Step> &getQueue() const;
  std::deque<Step>::size_type size() const { return queue_.size(); }
  void reload(const XmlRpc::XmlRpcValue &steps);
 private:
  std::deque<Step> queue_;
  moveit::planning_interface::MoveGroupInterface &arm_group_, &hand_group_;
  BaseMotion *base_motion_;
};
}

#endif //ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
