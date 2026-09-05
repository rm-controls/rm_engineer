//
// Created by ch on 2026/5/6.
//

#include <engineer_planning_adapters/special_joint_constraint_adapter.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_list_macros.hpp>

namespace engineer_planning_adapters
{
std::string SpecialJointConstraintAdapter::getDescription() const
{
  return "SpecialJointConstraintAdapter";
}

void SpecialJointConstraintAdapter::initialize(const ros::NodeHandle& nh)
{
  ros::NodeHandle pnh(nh, "special_joint_constraint");
  pnh.param("enabled", enabled_, false);
  pnh.param("joint_a", joint_a_, std::string());
  pnh.param("joint_b", joint_b_, std::string());
  pnh.param("constant", constant_, 0.0);
}

bool SpecialJointConstraintAdapter::adaptAndPlan(
    const PlannerFn& planner,
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    planning_interface::MotionPlanResponse& res,
    std::vector<std::size_t>& added_path_index) const
{
  if (!enabled_)
    return planner(planning_scene, req, res);

  if (joint_a_.empty() || joint_b_.empty())
  {
    ROS_ERROR_STREAM("SpecialJointConstraintAdapter: joint_a/joint_b not set");
    return false;
  }

  planning_scene::PlanningScenePtr scene = planning_scene->diff();
  const robot_model::JointModel* jm_a = scene->getRobotModel()->getJointModel(joint_a_);
  const robot_model::JointModel* jm_b = scene->getRobotModel()->getJointModel(joint_b_);

  if (!jm_a || !jm_b)
  {
    ROS_ERROR_STREAM("SpecialJointConstraintAdapter: unknown joint(s): "
                     << joint_a_ << ", " << joint_b_);
    return false;
  }

  if (jm_a->getVariableCount() != 1 || jm_b->getVariableCount() != 1)
  {
    ROS_ERROR_STREAM("SpecialJointConstraintAdapter: only 1-DOF joints supported");
    return false;
  }

  const std::string var_a = jm_a->getVariableNames()[0];
  const std::string var_b = jm_b->getVariableNames()[0];
  const double constant = constant_;

  scene->setStateFeasibilityPredicate(
      [var_a, var_b, constant](const robot_state::RobotState& state, bool verbose) -> bool
      {
        const double a = state.getVariablePosition(var_a);
        const double b = state.getVariablePosition(var_b);
        if (a >= constant - b)
          return true;

        if (verbose)
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "Joint relation violated: " << var_a
                                       << " < " << constant << " - " << var_b);
        }
        return false;
      });

  return planner(scene, req, res);
}
}  // namespace engineer_planning_adapters

PLUGINLIB_EXPORT_CLASS(engineer_planning_adapters::SpecialJointConstraintAdapter,
                       planning_request_adapter::PlanningRequestAdapter)