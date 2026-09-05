//
// Created by ch on 24-12-6.
//
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include  "auto_exchange_planner/auto_exchange_context.h"
#include  "auto_exchange_planner/auto_exchange_planner.h"
namespace auto_exchange_planner
{
AutoExchangeContext::AutoExchangeContext(const std::string& context_name, const std::string& ns,
                                         const std::string& group_name, const moveit::core::RobotModelConstPtr& model)
  : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
  auto_exchange_planner_ = AutoExchangePlannerPtr(new AutoExchangePlanner(ros::NodeHandle(ns)));
}

bool AutoExchangeContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  moveit_msgs::MotionPlanDetailedResponse res_msg;
  bool auto_exchange_solved = auto_exchange_planner_->solve(planning_scene_, request_, res_msg);

  if (auto_exchange_solved)
  {
    res.trajectory_.resize(1);
    res.trajectory_[0] =
        robot_trajectory::RobotTrajectoryPtr(new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));

    moveit::core::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(res_msg.trajectory_start, start_state);

    res.trajectory_[0]->setRobotTrajectoryMsg(start_state, res_msg.trajectory[0]);
    res.description_.push_back("plan");
    res.processing_time_ = res_msg.processing_time;
    res.error_code_ = res_msg.error_code;

    return true;
  }

  res.error_code_ = res_msg.error_code;
  return false;
};

bool AutoExchangeContext::solve(planning_interface::MotionPlanResponse& res)
{
  planning_interface::MotionPlanDetailedResponse res_detailed;
  bool planning_success = solve(res_detailed);

  res.error_code_ = res_detailed.error_code_;

  if (planning_success)
  {
    res.trajectory_ = res_detailed.trajectory_[0];
    res.planning_time_ = res_detailed.processing_time_[0];
  }

  return planning_success;
}

bool AutoExchangeContext::terminate()
{
  return true;
}

void AutoExchangeContext::clear()
{
  // This planner has no state, so has nothing to clear
}

}  // namespace auto_exchange_planner
