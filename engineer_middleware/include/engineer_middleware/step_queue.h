//
// Created by qiayuan on 4/3/21.
//

#ifndef ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#define ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
#pragma once

// STL
#include <deque>
#include <string>
#include <memory>
#include <vector>

#include "engineer_middleware/step.h"

namespace engineer_middleware {
class StepQueue {
 public:
  StepQueue() : active_(false), has_switched_step_(false), has_started_step_(false) {
    previous_step_.reset();
  }

  /*!
   * Clear entire queue including currently active step.
   */
  void clear() {
    previous_step_.reset();
    queue_.clear();
    active_ = false;
  }

  /*!
   * Returns the current step. Check empty() first!
   * @return the current step.
   */
  const Step &getCurrentStep() const;
  Step &getCurrentStep();
  void replaceCurrentStep(const Step &step);

  /*!
   * Returns the next step. Check if size() > 1 first!
   * @return the next step.
   */
  const Step &getNextStep() const;

  const std::deque<Step> &getQueue() const;

  /*!
   * Returns the previous step. Returns null pointer
   * if no previous step is available.
   * @return the previous step.
   */
  bool previousStepExists() const { return (bool) previous_step_; }
  const Step &getPreviousStep() const {
    if (!previous_step_) throw std::out_of_range("StepQueue::getPreviousStep(): No previous step available!");
    return *previous_step_;
  }

  /*!
   * Returns the number of steps in the queue.
   * @return the number of steps.
   */
  std::deque<Step>::size_type size() const { return queue_.size(); }

  friend class StepFrameConverter;

 private:

  //! Queue of step data.
  std::deque<Step> queue_;
  std::unique_ptr<Step> previous_step_;
  bool active_;
  bool has_switched_step_, has_started_step_;
};
}

#endif //ENGINEER_MIDDLEWARE_STEP_QUEUE_H_
