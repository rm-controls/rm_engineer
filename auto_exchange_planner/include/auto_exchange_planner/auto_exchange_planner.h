//
// Created by ch on 24-12-1.
//

#pragma once

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

namespace auto_exchange_planner
{
  MOVEIT_CLASS_FORWARD(AutoExchangePlanner);

  class AutoExchangePlanner
  {
  public:
    AutoExchangePlanner(const ros::NodeHandle& nh = ros::NodeHandle("~"));

    bool solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
               const planning_interface::MotionPlanRequest& req, moveit_msgs::MotionPlanDetailedResponse& res);

  protected:
    ros::NodeHandle nh_;
    std::string name_;
    int num_steps_;
    int dof_;
    double trajectory2_length_;

  private:
    void interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& robot_state,
                     const moveit::core::JointModelGroup* joint_model_group, const std::vector<double>& start_joint_vals,
                     const std::vector<double>& goal_joint_vals, trajectory_msgs::JointTrajectory& joint_trajectory);
    void auto_exchange_interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& robot_state,
                 const moveit::core::JointModelGroup* joint_model_group, const geometry_msgs::Pose& start_pose,
                 const geometry_msgs::Pose& goal_pose, trajectory_msgs::JointTrajectory& joint_trajectory);
  };
}  // namespace auto_exchange_planner
