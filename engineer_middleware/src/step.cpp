#include "engineer_middleware/step.h"

#include <uuid/uuid.h>
#include <cmath>

namespace engineer_middleware {

Step::Step(const XmlRpc::XmlRpcValue &step) {
  ROS_ASSERT(step.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if (step.hasMember("end_effector_target")) {

  }
  if (step.hasMember("base_target")) {
    //TODO: add base motion
  }
}

Step::Step() : time_(0.0), total_duration_(0.0), is_updated_(false), is_computed_(false) {
  arm_motion_.reset();
  base_motion_.reset();

  // Generate unique id.
  uuid_t id;
  uuid_generate(id);
  char *stringId = new char[36];
  uuid_unparse(id, stringId);
  id_.assign(stringId, 36);
  delete[] stringId;
}

Step::Step(const Step &other)
    : time_(other.time_),
      total_duration_(other.total_duration_),
      is_updated_(other.is_updated_),
      is_computed_(other.is_computed_),
      id_(other.id_) {
  if (other.base_motion_) base_motion_ = other.base_motion_->clone();
  if (other.arm_motion_) arm_motion_ = other.arm_motion_->clone();
}

Step &Step::operator=(const Step &other) {
  time_ = other.time_;
  total_duration_ = other.time_;
  is_updated_ = other.is_updated_;
  is_computed_ = other.is_computed_;
  id_ = other.id_;
  if (other.base_motion_) base_motion_ = other.base_motion_->clone();
  if (other.arm_motion_) arm_motion_ = other.arm_motion_->clone();
  return *this;
}

std::unique_ptr<Step> Step::clone() const {
  std::unique_ptr<Step> pointer(new Step(*this));
  return pointer;
}

void Step::addBaseMotion(const BaseMotionBase &base_motion) {
  base_motion_ = base_motion.clone();
  is_updated_ = false;
  is_computed_ = false;
}

bool Step::needsComputation() const {
  bool needsComputation = false;
  if (arm_motion_->needsComputation())
    needsComputation = true;
  if (base_motion_->needsComputation()) {
    needsComputation = true;
  }
  return needsComputation;
}

bool Step::compute() {
  if (hasArmMotion()) {
    if (arm_motion_->needsComputation()) {
      if (!base_motion_->compute()) return false;
    }
  }
  if (hasBaseMotion()) {
    if (base_motion_->needsComputation()) {
      if (!base_motion_->compute()) return false;
    }
  }
  is_computed_ = true;
  return true;
}

bool Step::isUpdated() const {
  return is_updated_;
}

void Step::reset() {
  time_ = 0.0;
  total_duration_ = 0.0;
  is_updated_ = false;
  is_computed_ = false;
  if (hasArmMotion()) {
    arm_motion_.reset();
  }
  if (hasBaseMotion()) {
    base_motion_.reset();
  }
}

} /* namespace */
