#include "engineer_middleware/step.h"

#include <uuid/uuid.h>
#include <cmath>

namespace engineer_middleware {

inline void boundToRange(double *v, double min, double max) {
  if (*v < min) *v = min;
  if (*v > max) *v = max;
}

inline double mapTo01Range(double v, double min, double max) {
  double t = v;
  if (fabs(min - max) < 0.000000001) return 1;
  boundToRange(&t, min, max);
  t = (t - min) / (max - min);
  return t;
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

bool Step::update() {
  if (needsComputation() && !is_computed_) return false;
  total_duration_ = 0.0;
  if (hasArmMotion()) {
    if (arm_motion_->getDuration() > total_duration_)
      total_duration_ = arm_motion_->getDuration();
  }
  if (hasBaseMotion()) {
    if (base_motion_->getDuration() > total_duration_)
      total_duration_ = base_motion_->getDuration();
  }
  return is_updated_ = true;
}

bool Step::advance(double dt) {
  if (!is_updated_) throw std::runtime_error("Step::advance() cannot be called if step is not updated.");
  time_ += dt;
  if (time_ > getTotalDuration()) return false;
  return true;
}

double Step::getBaseMotionPhase() const {
  if (!is_updated_) throw std::runtime_error("Step::getBaseMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getBaseMotionDuration());
}
double Step::getArmMotionPhase() const {
  if (!is_updated_) throw std::runtime_error("Step::getArmMotionPhase() cannot be called if step is not updated.");
  return mapTo01Range(time_, 0.0, getArmMotionDuration());
}
bool Step::isApproachingEnd(double tolerance) const {
  if (!is_updated_) throw std::runtime_error("Step::isApproachingEnd() cannot be called if step is not updated.");
  if (getTime() + tolerance >= getTotalDuration()) return true;
  return false;
}

const std::string &Step::getId() const {
  return id_;
}

void Step::setId(const std::string &id) {
  id_ = id;
}

//std::ostream &operator<<(std::ostream &out, const Step &step) {
//  out << "------" << std::endl;
//  out << "ID: " << step.id_ << std::endl;
//  if (step.hasArmMotion()) {
//    out << "---" << std::endl;
//    out << "Arm motion: " << std::endl;
//    out << *(step.arm_motion_) << std::endl;
//  }
//  if (step.hasBaseMotion()) {
//    out << "---" << std::endl;
//    out << "Base motion: " << std::endl;
//    out << *(step.base_motion_) << std::endl;
//  }
//  return out;
//}

} /* namespace */
