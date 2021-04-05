#include "engineer_middleware/step.h"

#include <uuid/uuid.h>

namespace engineer_middleware {

Step::Step(const XmlRpc::XmlRpcValue &step,
           moveit::planning_interface::MoveGroupInterface &arm_group,
           moveit::planning_interface::MoveGroupInterface &hand_group) {
  ROS_ASSERT(step.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  if (step.hasMember("end_effector_target")) {
    arm_motion_ = std::make_unique<EndEffectorTarget>(step["end_effector_target"], arm_group, hand_group);
  } else if (step.hasMember("joints_target")) {
    arm_motion_ = std::make_unique<JointsTarget>(step["joints_target"], arm_group, hand_group);
  }
  if (step.hasMember("base_target")) {
    //TODO: add base motion
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
    return arm_motion_->move();
  return false;
}

} /* namespace */
