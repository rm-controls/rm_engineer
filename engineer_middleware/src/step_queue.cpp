//
// Created by qiayuan on 4/3/21.
//


#include "engineer_middleware/step_queue.h"

namespace engineer_middleware {

StepQueue::StepQueue(const XmlRpc::XmlRpcValue &steps,
                     moveit::planning_interface::MoveGroupInterface &arm_group,
                     moveit::planning_interface::MoveGroupInterface &hand_group) :
    arm_group_(arm_group), hand_group_(hand_group) {
  arm_group.getCurrentState();
  ROS_ASSERT(steps.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < steps.size(); ++i) {
    queue_.emplace_back(steps[i], arm_group, hand_group);
  }
}

bool StepQueue::move() {
  for (auto &step:queue_) {
    step.compute(*arm_group_.getCurrentState());
    if (!step.move())
      return false;
  }
  return true;
}

const std::deque<Step> &StepQueue::getQueue() const {
  return queue_;
}

}