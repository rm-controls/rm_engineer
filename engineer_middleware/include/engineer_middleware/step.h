#pragma once

#include "engineer_middleware/arm_motion.h"

// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include <rm_common/ros_utilities.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace engineer_middleware {

class Step {
 public:
  Step(const XmlRpc::XmlRpcValue &step, moveit::planning_interface::MoveGroupInterface &arm_group,
       moveit::planning_interface::MoveGroupInterface &hand_group);
  ~Step() = default;

  bool compute(const moveit::core::RobotState &current_state);
  bool move();

  bool hasArmMotion() const { return (bool) (arm_motion_); }
  const std::string &getId() const { return id_; }
  void setId(const std::string &id) { id_ = id; }

 protected:
  std::unique_ptr<ArmMotionBase> arm_motion_;

 private:
  std::string id_;
};

} /* namespace */
