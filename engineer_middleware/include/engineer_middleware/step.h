#pragma once

#include "arm_motion.h"

// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include <rm_common/ros_utilities.h>

namespace engineer_middleware {

class LegMotionBase;
class BaseMotionBase;

class Step {
 public:
  Step();
  ~Step() = default;
  Step(const XmlRpc::XmlRpcValue &step);
  Step(const Step &other);
  Step &operator=(const Step &other);

  std::unique_ptr<Step> clone() const;
  /*!
   * Add swing data for a arm.
   * @param data the step data.
   */
  void addArmMotion(const LegMotionBase &arm_motion);

  /*!
   * Add base shift data for a state.
   * @param state the corresponding state of the base shift data.
   * @param data the base shift data.
   */
  void addBaseMotion(const BaseMotionBase &base_motion);

  bool needsComputation() const;
  bool compute();

  bool update();
  bool isUpdated() const;
  void reset();

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if step is active, false if finished.
   */
  bool advance(double dt);

  bool hasArmMotion() const { return (bool) (arm_motion_); }
  const BaseMotionBase &getBaseMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getBaseMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };
  bool hasBaseMotion() const { return (bool) (base_motion_); };
  const BaseMotionBase &getArmMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getArmMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };

  /*!
   * Return the current time of the step, starting at 0.0 for each step.
   * @return the current time.
   */
  double getTime() const { return time_; }
  double getTotalDuration() const {
    if (!is_updated_) throw std::runtime_error("Step::getTotalDuration() cannot be called if step is not updated.");
    return total_duration_;
  }
  double getTotalPhase() const {
    if (!is_updated_) throw std::runtime_error("Step::getTotalDuration() cannot be called if step is not updated.");
    return total_duration_;
  }
  double getBaseMotionDuration() const {
    if (!is_updated_) throw std::runtime_error("Step::getBaseMotionDuration() cannot be called if step is not updated.");
    if (!hasBaseMotion()) return 0.0;
    return base_motion_->getDuration();
  }
  double getArmMotionDuration() const {
    if (!is_updated_) throw std::runtime_error("Step::getBaseMotionDuration() cannot be called if step is not updated.");
    if (!hasBaseMotion()) return 0.0;
    return base_motion_->getDuration();
  }

  const std::string &getId() const { return id_; }
  void setId(const std::string &id) { id_ = id; }

  friend class StepCompleter;

 protected:
  std::unique_ptr<ArmMotionBase> arm_motion_;
  std::unique_ptr<BaseMotionBase> base_motion_;

 private:
  //! Current time, starts at 0.0 at each step.
  double time_;
  double total_duration_;
  bool is_updated_;
  bool is_computed_;
  std::string id_;
};

} /* namespace */
