//
// Created by ch on 2026/5/6.
//

#pragma once

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <ros/ros.h>
#include <string>

namespace engineer_planning_adapters
{
class SpecialJointConstraintAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  std::string getDescription() const override;
  void initialize(const ros::NodeHandle& nh) override;

  bool adaptAndPlan(const PlannerFn& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override;

private:
  bool enabled_{true};
  std::string joint_a_;
  std::string joint_b_;
  double constant_{0.0};
};
}  // namespace engineer_planning_adapters