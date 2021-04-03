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

  virtual ~StepQueue() = default;
  StepQueue(const StepQueue &other)
      : queue_(other.queue_),
        active_(other.active_),
        has_switched_step_(other.has_switched_step_),
        has_started_step_(other.has_started_step_) {
    if (other.previous_step_) previous_step_ = other.previous_step_->clone();
  }
  StepQueue &operator=(const StepQueue &other) {
    queue_ = other.queue_;
    if (other.previous_step_) previous_step_ = other.previous_step_->clone();
    active_ = other.active_;
    has_switched_step_ = other.has_switched_step_;
    has_started_step_ = other.has_started_step_;
    return *this;
  }

  void add(const Step &step) { queue_.push_back(step); }
  void add(const std::vector<Step> &steps) {
    auto iterator = queue_.end();
    queue_.insert(iterator, steps.begin(), steps.end());
  }
  void addInFront(const Step &step) {
    queue_.push_front(step);
    active_ = false;
  }

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if successful, false otherwise.
   */
  bool advance(double dt);

  bool hasSwitchedStep() const { return has_switched_step_; }
  bool hasStartedStep() const { return has_started_step_; }

  /*!
   * Queue is active if a step is available and the step is ready/updated.
   * @return true if step should be processed, false otherwise.
   */
  bool active() const {
    if (empty()) return false;
    return queue_.front().isUpdated();
  }
  /*!
   * Checks if queue contain steps.
   * @return true if queue empty, false otherwise.
   */
  bool empty() const { return queue_.empty(); }

  void skipCurrentStep() {
    if (empty()) return;
    previous_step_ = std::make_unique<Step>(queue_.front());
    queue_.pop_front();
    active_ = false;
  }

  /*!
   * Clear queue but keeps the currently running step.
   */
  void clearNextSteps() {
    if (empty()) return;
    queue_.erase(queue_.begin() + 1, queue_.end());
  }
  void clearLastNSteps(size_t nSteps) {
    if (empty()) return;
    queue_.erase(queue_.end() - nSteps, queue_.end());
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
