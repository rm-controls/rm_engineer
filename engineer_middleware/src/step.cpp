#include "engineer_middleware/step.h"

#include <uuid/uuid.h>

namespace engineer_middleware {

Step::Step(const XmlRpc::XmlRpcValue &step,
           moveit::planning_interface::MoveGroupInterface &arm_group,
           moveit::planning_interface::MoveGroupInterface &hand_group,
           BaseMotion *base_motion) {
  ROS_ASSERT(step.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  if (step.hasMember("end_effector_target")) {
    arm_motion_ = std::make_unique<EndEffectorTarget>(step["end_effector_target"], arm_group, hand_group);
  } else if (step.hasMember("joints_target")) {
    arm_motion_ = std::make_unique<JointsTarget>(step["joints_target"], arm_group, hand_group);
  }
  if (step.hasMember("base_target")) {
    XmlRpc::XmlRpcValue position = step["base_target"];
    base_motion_ = base_motion;
    if (step["base_target"].hasMember("chassis")) {
      has_chassis = true;
      ROS_ASSERT(position["chassis"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      base_motion_position_.push_back(xmlRpcGetDouble(position["chassis"], 0, 0.0));
      base_motion_position_.push_back(xmlRpcGetDouble(position["chassis"], 1, 0.0));
      base_motion_position_.push_back(xmlRpcGetDouble(position["chassis"], 2, 0.0));
    }
    if (step["base_target"].hasMember("gimbal")) {
      has_gimbal = true;
      ROS_ASSERT(position["gimbal"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      base_motion_position_.push_back(xmlRpcGetDouble(position["gimbal"], 0, 0.0));
      base_motion_position_.push_back(xmlRpcGetDouble(position["gimbal"], 1, 0.0));
    }
  }
}

bool Step::compute(const moveit::core::RobotState &current_state) {
  if (hasArmMotion()) {
    arm_motion_->compute(current_state);
  }
  return true;
}

bool Step::move() {
  if (hasArmMotion())
    arm_motion_->move();
  if (!hasArmMotion() && has_chassis)
    base_motion_->chassisMoveTo(base_motion_position_[0], base_motion_position_[1], base_motion_position_[2]);
  else if (has_chassis)
    base_motion_->setChassisPosition(base_motion_position_[0], base_motion_position_[1], base_motion_position_[2]);
  return true;
}

} /* namespace */
