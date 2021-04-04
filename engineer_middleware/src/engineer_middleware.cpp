//
// Created by qiayuan on 4/4/21.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory &posture) {
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_joint";
  posture.joint_names[1] = "right_finger_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.;
  posture.points[0].positions[1] = 0.;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture) {
  posture.joint_names.resize(2);
  posture.joint_names[0] = "left_finger_joint";
  posture.joint_names[1] = "right_finger_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.02;
  posture.points[0].positions[1] = 0.02;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group) {
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.4;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.30;

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.0;
  grasps[0].pre_grasp_approach.desired_distance = 0.1;

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
/* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0;
  grasps[0].post_grasp_retreat.desired_distance = 0.10;
  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &group) {
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, 0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  place_location[0].place_pose.pose.position.x = 0.35;
  place_location[0].place_pose.pose.position.y = 0.0;
  place_location[0].place_pose.pose.position.z = 0.2;

  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.05;
  place_location[0].pre_place_approach.desired_distance = 0.1;
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  place_location[0].post_place_retreat.direction.vector.z = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.05;
  place_location[0].post_place_retreat.desired_distance = 0.1;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);
  group.setSupportSurfaceName("table2");
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 1.0;
  collision_objects[0].primitives[0].dimensions[1] = 1.0;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 1.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.05;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.0;
  collision_objects[1].primitives[0].dimensions[1] = 1.0;
  collision_objects[1].primitives[0].dimensions[2] = 0.01;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 1.0;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.05;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.1;
  collision_objects[2].primitives[0].dimensions[1] = 0.1;
  collision_objects[2].primitives[0].dimensions[2] = 0.1;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.35;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.1;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  collision_objects[2].operation = collision_objects[0].ADD;
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "engineer_middleware");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "engineer_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  const moveit::core::JointModelGroup *joint_model_group =
//      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//
//  namespace rvt = rviz_visual_tools;
//  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
//
//  visual_tools.deleteAllMarkers();
//  visual_tools.loadRemoteControl();
//
//  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//  text_pose.translation().z() = 1.0;
//  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
//
//  visual_tools.trigger();
//
//  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
//  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
//
//  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//  std::copy(move_group_interface.getJointModelGroupNames().begin(),
//            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
//
////  geometry_msgs::Pose target_pose1;
////  target_pose1.position.x = 0.4;
////  target_pose1.position.y = 0.1;
////  target_pose1.position.z = 0.2;
////  move_group_interface.setPoseTarget(target_pose1);
//  move_group_interface.setPositionTarget(0.0, 0.3, 0.35);
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group_interface.move();
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
  // Wait a bit for ROS things to initialize

  addCollisionObjects(planning_scene_interface);

  ros::WallDuration(1.0).sleep();
  pick(move_group_interface);
  ros::WallDuration(1.0).sleep();
//  place(move_group_interface);
  ros::shutdown();
  return 0;
}