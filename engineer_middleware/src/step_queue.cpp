//
// Created by qiayuan on 4/3/21.
//

#include <memory>

#include "engineer_middleware/step_queue.h"

namespace engineer_middleware {

bool StepQueue::advance(double dt) {
  // Check if empty.
  has_switched_step_ = false;
  has_started_step_ = false;
  if (queue_.empty()) {
    active_ = false;
    return true;
  }

  // Special treatment of first step of queue.
  if (!active_) {
    active_ = true;
    has_switched_step_ = true;
    return true;
  }

  // Check if step is updated (multi-threading).
  if (!queue_.front().isUpdated()) {
    if (queue_.front().update()) {
      has_started_step_ = true;
    } else {
      return true;
    }
  }

  // Advance current step.
  if (!queue_.front().advance(dt)) {
    // Step finished.
    previous_step_ = std::make_unique<Step>(queue_.front());
    queue_.pop_front();
    if (queue_.empty()) {
      // End reached.
      active_ = false;
      has_started_step_ = false;
      return true;
    }
    has_switched_step_ = true;
  }

  return true;
}

const Step &StepQueue::getCurrentStep() const {
  if (empty()) throw std::out_of_range("StepQueue::getCurrentStep(): No steps in queue!");
  return queue_.front();
}

Step &StepQueue::getCurrentStep() {
  if (empty()) throw std::out_of_range("StepQueue::getCurrentStep(): No steps in queue!");
  return queue_.front();
}

void StepQueue::replaceCurrentStep(const Step &step) {
  queue_.pop_front();
  queue_.push_front(step);
}

const Step &StepQueue::getNextStep() const {
  if (size() <= 1) throw std::out_of_range("StepQueue::getNextStep(): No next step in queue!");
  auto iterator = queue_.begin() + 1;
  return *iterator;
}

const std::deque<Step> &StepQueue::getQueue() const {
  return queue_;
}

}