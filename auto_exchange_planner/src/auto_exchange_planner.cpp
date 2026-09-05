//
// Created by ch on 24-12-1.
//

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <ros/ros.h>

#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include "auto_exchange_planner/auto_exchange_planner.h"

namespace auto_exchange_planner
{
  AutoExchangePlanner::AutoExchangePlanner(const ros::NodeHandle& nh) : nh_(nh),name_("AutoExchangePlanner"), num_steps_(1), dof_(0)
  {
  }

  bool AutoExchangePlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          moveit_msgs::MotionPlanDetailedResponse& res)
  {
    // Load the planner-specific parameters
    nh_.param("num_steps", num_steps_, 50);

    ros::Time start_time = ros::Time::now();
    moveit::core::RobotModelConstPtr robot_model = planning_scene->getRobotModel();
    moveit::core::RobotStatePtr start_state(new moveit::core::RobotState(robot_model)), middle_state(new moveit::core::RobotState(robot_model));
    *start_state = planning_scene->getCurrentState();
    const moveit::core::JointModelGroup* joint_model_group = start_state->getJointModelGroup(req.group_name);
    const moveit::core::JointModelGroup* mid_joint_model_group = middle_state->getJointModelGroup(req.group_name);
    std::vector<std::string> joint_names = joint_model_group->getVariableNames();
    dof_ = joint_names.size();
    std::vector<double> start_joint_values;
    start_state->copyJointGroupPositions(joint_model_group, start_joint_values);
    trajectory_msgs::JointTrajectory joint_trajectory;

    if (req.goal_constraints.size() > 1)
    {
      geometry_msgs::Pose goal_pose_final;
      goal_pose_final.position = req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position;
      goal_pose_final.orientation = req.goal_constraints[0].orientation_constraints[0].orientation;

      geometry_msgs::Pose goal_pose_mid;
      goal_pose_mid.position = req.goal_constraints[1].position_constraints[0].constraint_region.primitive_poses[0].position;
      goal_pose_mid.orientation = req.goal_constraints[1].orientation_constraints[0].orientation;

      std::vector<double> middle_joint_values;
      bool found_mid_ik = middle_state->setFromIK(mid_joint_model_group, goal_pose_mid);
      if (found_mid_ik)
      {
        middle_state->copyJointGroupPositions(mid_joint_model_group, middle_joint_values);
      }
      else
      {
        ROS_INFO("Did not find middle_state's IK solution for middle state!");
        return false;
      }

      interpolate(joint_names, start_state, joint_model_group, start_joint_values, middle_joint_values, joint_trajectory);
      auto_exchange_interpolate(joint_names, start_state, joint_model_group, goal_pose_mid, goal_pose_final, joint_trajectory);

    }
    else
    {
      const std::vector<moveit_msgs::Constraints>& goal_constraints = req.goal_constraints;
      const std::vector<moveit_msgs::JointConstraint>& goal_joint_constraint = goal_constraints[0].joint_constraints;

      std::vector<double> goal_joint_values;
      goal_joint_values.reserve(goal_joint_constraint.size());

      for (const auto& constraint : goal_joint_constraint)
      {
        goal_joint_values.push_back(constraint.position);
      }

      interpolate(joint_names, start_state, joint_model_group, start_joint_values, goal_joint_values, joint_trajectory);

    }

    res.trajectory.resize(1);
    res.trajectory[0].joint_trajectory.joint_names = joint_names;
    res.trajectory[0].joint_trajectory.header = req.start_state.joint_state.header;
    res.trajectory[0].joint_trajectory = joint_trajectory;

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    res.processing_time.push_back((ros::Time::now() - start_time).toSec());

    res.group_name = req.group_name;
    res.trajectory_start.joint_state.name = joint_names;
    res.trajectory_start.joint_state.position = start_joint_values;

    return true;
  }

  void AutoExchangePlanner::interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
                                const moveit::core::JointModelGroup* joint_model_group,
                                const std::vector<double>& start_joint_vals, const std::vector<double>& goal_joint_vals,
                                trajectory_msgs::JointTrajectory& joint_trajectory)
  {
    std::vector<double> dt_vector, m_start_joint_vals;
    for (int joint_index = 0; joint_index < dof_; ++joint_index)
    {
      double dt = (goal_joint_vals[joint_index] - start_joint_vals[joint_index]) / num_steps_;
      dt_vector.push_back(dt);
    }

    for (double start_joint_val : start_joint_vals)
      m_start_joint_vals.push_back(start_joint_val + 0.001 );

    for (int step = 0; step <= num_steps_; ++step)
    {
      std::vector<double> joint_values;
      for (int k = 0; k < dof_; ++k)
      {
        double joint_value = m_start_joint_vals[k] + step * dt_vector[k];
        joint_values.push_back(joint_value);
      }
      rob_state->setJointGroupPositions(joint_model_group, joint_values);
      rob_state->update();

      joint_trajectory.joint_names = joint_names;
      trajectory_msgs::JointTrajectoryPoint trajectory_point;
      trajectory_point.positions = joint_values;
      joint_trajectory.points.push_back(trajectory_point);
    }
  }

  void AutoExchangePlanner::auto_exchange_interpolate(const std::vector<std::string>& joint_names, moveit::core::RobotStatePtr& rob_state,
               const moveit::core::JointModelGroup* joint_model_group, const geometry_msgs::Pose& start_pose,
               const geometry_msgs::Pose& goal_pose, trajectory_msgs::JointTrajectory& joint_trajectory)
  {
    double dt_x = (goal_pose.position.x - start_pose.position.x)/num_steps_,
           dt_y = (goal_pose.position.y - start_pose.position.y)/num_steps_,
           dt_z = (goal_pose.position.z - start_pose.position.z)/num_steps_;

    for (int step = 0; step <= num_steps_; ++step)
    {
      std::vector<double> joint_values;
      geometry_msgs::Pose waypoint_pose;
      waypoint_pose.orientation = start_pose.orientation;
      waypoint_pose.position.x = start_pose.position.x + dt_x * step;
      waypoint_pose.position.y = start_pose.position.y + dt_y * step;
      waypoint_pose.position.z = start_pose.position.z + dt_z * step;
      // ROS_INFO("waypoint%d x:%f  y:%f  z:%f",step,waypoint_pose.position.x,waypoint_pose.position.y,waypoint_pose.position.z);
      bool found_waypoint_ik = rob_state->setFromIK(joint_model_group, waypoint_pose);
      if (found_waypoint_ik)
      {
        rob_state->copyJointGroupPositions(joint_model_group, joint_values);
        rob_state->update();

        joint_trajectory.joint_names = joint_names;
        trajectory_msgs::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions = joint_values;
        joint_trajectory.points.push_back(trajectory_point);
      }
      else
      {
        ROS_INFO("Did not find IK solution in step %d !", step);
      }
    }
  }

}  // namespace auto_exchange_planner