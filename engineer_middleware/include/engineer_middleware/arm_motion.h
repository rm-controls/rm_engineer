#pragma once

#include "engineer_middleware/step.h"
#include "engineer_middleware/state.h"

// STD
#include <string>
#include <memory>
#include <geometry_msgs/Pose.h>

// ROS
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace engineer_middleware {

enum HandMotion {
  FREEZE = 0,
  OPEN_BEFORE_ARM_MOTION = 1,
  OPEN_AFTER_ARM_MOTION = 2,
  CLOSE_BEFORE_ARM_MOTION = 3,
  CLOSE_AFTER_ARM_MOTION = 4,
};

class ArmMotionBase {
 public:
  ArmMotionBase(const XmlRpc::XmlRpcValue &arm_motion,
                moveit::planning_interface::MoveGroupInterface &arm_group,
                moveit::planning_interface::MoveGroupInterface &hand_group);

  ~ArmMotionBase() = default;
  virtual bool compute(const moveit::core::RobotState &current_state);
 protected:
  moveit::planning_interface::MoveGroupInterface &arm_group_, &hand_group_;
  moveit::planning_interface::MoveGroupInterface::Plan arm_plan_, hand_plan_;
  HandMotion hand_motion_;
};

class EndEffectorTarget : public ArmMotionBase {
 public:
  EndEffectorTarget(const XmlRpc::XmlRpcValue &arm_motion,
                    moveit::planning_interface::MoveGroupInterface &arm_group,
                    moveit::planning_interface::MoveGroupInterface &hand_group);
  bool compute(const moveit::core::RobotState &current_state) override;
 private:
  bool has_pos_, has_ori_;
  geometry_msgs::PoseStamped target_;
};

class JointTarget : public ArmMotionBase {
 public:
  JointTarget(const XmlRpc::XmlRpcValue &arm_motion,
              moveit::planning_interface::MoveGroupInterface &arm_group,
              moveit::planning_interface::MoveGroupInterface &hand_group);
  bool compute(const moveit::core::RobotState &current_state) override;
 private:
  bool has_joints_;
  std::vector<double> target_;
};

} /* namespace */
