#pragma once

#include "engineer_middleware/step.h"
#include "engineer_middleware/state.h"

// STD
#include <string>
#include <memory>

namespace engineer_middleware {

class State;
class Step;

/*!
 * Base class for a generic arm motion.
 */
class ArmMotionBase {
 public:

  enum class TargetType {
    END_EFFECTOR,
    JOINT
  };
  enum class TrajectoryType {
    NORMAL,
    CARTESIAN
  };

  explicit ArmMotionBase(ArmMotionBase::TargetType type) : type_(type) {}
  ~ArmMotionBase() = default;

  ArmMotionBase(const ArmMotionBase &other) = default;
  ArmMotionBase &operator=(const ArmMotionBase &other) = default;
  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<ArmMotionBase> clone() const {
    throw std::runtime_error("ArmMotionBase::clone() not implemented.");
  }

  /*!
   * Returns the type of the arm motion trajectory.
   * @return the type of the arm motion trajectory.
   */
  ArmMotionBase::TargetType getType() const { return type_; }
  ArmMotionBase::TrajectoryType getTrajectoryType() const {
    throw std::runtime_error("ArmMotionBase::getTrajectoryType() not implemented.");
  }

  virtual bool needsComputation() const { throw std::runtime_error("ArmMotionBase::needsComputation() not implemented."); }
  virtual bool compute() { throw std::runtime_error("ArmMotionBase::compute() not implemented."); }

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const { throw std::runtime_error("ArmMotionBase::getDuration() not implemented."); }

  friend class StepCompleter;

 protected:
  /*!
   * Returns a desired time to fit within the start and end time of the motion.
   * @param time the desired time.
   * @return the time mapped within the motion duration.
   */
  double mapTimeWithinDuration(const double time) const;

 private:
  //! Type of the arm motion.

  TargetType type_;
};

} /* namespace */
