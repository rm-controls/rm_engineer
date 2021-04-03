//
// Created by qiayuan on 4/3/21.
//
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
 * Base class for a generic swing leg motion.
 */
class BaseMotionBase {
 public:

  enum class Type {
    LOCK,
    AUTO,
    TELE_OP,
  };

  explicit BaseMotionBase(BaseMotionBase::Type type) : type_(type) {}
  ~BaseMotionBase() = default;

  BaseMotionBase(const BaseMotionBase &other) = default;
  BaseMotionBase &operator=(const BaseMotionBase &other) = default;
  /*!
   * Deep copy clone.
   * @return a clone of this class.
   */
  virtual std::unique_ptr<BaseMotionBase> clone() const {
    throw std::runtime_error("BaseMotionBase::clone() not implemented.");
  }

  /*!
   * Returns the type of the arm motion trajectory.
   * @return the type of the arm motion trajectory.
   */
  BaseMotionBase::Type getType() const { return type_; }

  virtual bool needsComputation() const { throw std::runtime_error("BaseMotionBase::needsComputation() not implemented."); }
  virtual bool compute() { throw std::runtime_error("BaseMotionBase::compute() not implemented."); }

  /*!
   * Returns the total duration of the motion.
   * @return the duration.
   */
  virtual double getDuration() const { throw std::runtime_error("BaseMotionBase::getDuration() not implemented."); }

 private:
  //! Type of the base motion.
  Type type_;
};

} /* namespace */
