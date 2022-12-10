//
// Created by ljq on 2022/4/28.
//

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace engineer_middleware
{
class PlanningScene
{
public:
  PlanningScene(const XmlRpc::XmlRpcValue& scene, moveit::planning_interface::MoveGroupInterface& arm_group,
                tf2_ros::Buffer& tf)
    : arm_group_(arm_group), tf_(tf)
  {
    trans_.header.frame_id = std::string(scene["target_position"]["frame_id"]);
    trans_.twist.linear.x = xmlRpcGetDouble(scene["target_position"]["pose"]["position"], 0);
    trans_.twist.linear.y = xmlRpcGetDouble(scene["target_position"]["pose"]["position"], 1);
    trans_.twist.linear.z = xmlRpcGetDouble(scene["target_position"]["pose"]["position"], 2);
    trans_.twist.angular.x = xmlRpcGetDouble(scene["target_position"]["pose"]["rpy"], 0);
    trans_.twist.angular.y = xmlRpcGetDouble(scene["target_position"]["pose"]["rpy"], 1);
    trans_.twist.angular.z = xmlRpcGetDouble(scene["target_position"]["pose"]["rpy"], 2);
    for (int i = 0; i < scene["scene"].size(); i++)
    {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = std::string(scene["scene"][i]["frame_id"]);
      frame_id_.frame_id = std::string(scene["scene"][i]["frame_id"]);
      collision_object.id = std::string(scene["scene"][i]["id"]);
      collision_object.primitives.push_back(Primitive(scene["scene"][i]["primitive"]));
      collision_object.primitive_poses.push_back(Pose(scene["scene"][i]["pose"]));
      collision_objects_.push_back(collision_object);
      if (scene["scene"][i].hasMember("attach"))
        is_attached_ = scene["scene"][i]["attach"];
    }
    ROS_INFO("9");
  };

  shape_msgs::SolidPrimitive Primitive(const XmlRpc::XmlRpcValue& xmlRpc)
  {
    shape_msgs::SolidPrimitive primitive;
    if (std::string(xmlRpc["type"]) == "box")
    {
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = xmlRpc["dimension"][0];
      primitive.dimensions[primitive.BOX_Y] = xmlRpc["dimension"][1];
      primitive.dimensions[primitive.BOX_Z] = xmlRpc["dimension"][2];
    }
    if (std::string(xmlRpc["type"]) == "cone")
    {
      primitive.type = primitive.CONE;
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CONE_HEIGHT] = xmlRpc["dimension"][0];
      primitive.dimensions[primitive.CONE_RADIUS] = xmlRpc["dimension"][1];
    }
    if (std::string(xmlRpc["type"]) == "cylinder")
    {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = xmlRpc["dimension"][0];
      primitive.dimensions[primitive.CYLINDER_RADIUS] = xmlRpc["dimension"][1];
    }
    if (std::string(xmlRpc["type"]) == "sphere")
    {
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1);
      primitive.dimensions[primitive.SPHERE_RADIUS] = xmlRpc["dimension"][0];
    }
    return primitive;
  };

  geometry_msgs::Pose Pose(const XmlRpc::XmlRpcValue& xmlRpc)
  {
    geometry_msgs::Pose pose;
    pose.orientation.x = xmlRpcGetDouble(xmlRpc["orientation"], 0);
    pose.orientation.y = xmlRpcGetDouble(xmlRpc["orientation"], 1);
    pose.orientation.z = xmlRpcGetDouble(xmlRpc["orientation"], 2);
    pose.orientation.w = xmlRpcGetDouble(xmlRpc["orientation"], 3);

    pose.position.x = xmlRpcGetDouble(xmlRpc["position"], 0);
    pose.position.y = xmlRpcGetDouble(xmlRpc["position"], 1);
    pose.position.z = xmlRpcGetDouble(xmlRpc["position"], 2);
    return pose;
  }

  bool calculateFinalPosition(geometry_msgs::TwistStamped target_transform)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = target_transform.header.frame_id;
    pose.pose.position.x = target_transform.twist.linear.x;
    pose.pose.position.y = target_transform.twist.linear.y;
    pose.pose.position.z = target_transform.twist.linear.z;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(target_transform.twist.linear.x, target_transform.twist.linear.x, target_transform.twist.linear.x);
    geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
    pose.pose.orientation = quat_msg;
    try
    {
      tf2::doTransform(pose.pose, pose.pose,
                       tf_.lookupTransform(frame_id_.frame_id, target_transform.header.frame_id, ros::Time(0)));
      target_transform.header.frame_id = frame_id_.frame_id;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
    double roll, pitch, yaw;
    roll = target_transform.twist.angular.x;
    pitch = target_transform.twist.angular.y;
    yaw = target_transform.twist.angular.z;
    for (long unsigned int i = 0; i < collision_objects_.size(); i++)
    {
      double roll_temp, pitch_temp, yaw_temp;
      quatToRPY(collision_objects_[i].primitive_poses[0].orientation, roll_temp, pitch_temp, yaw_temp);
      roll_temp += roll;
      pitch_temp += pitch;
      yaw_temp += yaw;
      tf2::Quaternion quat_tf1;
      quat_tf1.setRPY(roll_temp, pitch_temp, yaw_temp);
      geometry_msgs::Quaternion quat_msg1 = tf2::toMsg(quat_tf1);
      collision_objects_[i].primitive_poses[0].orientation = quat_msg1;
      collision_objects_[i].primitive_poses[0].position.x += pose.pose.position.x;
      collision_objects_[i].primitive_poses[0].position.y += pose.pose.position.y;
      collision_objects_[i].primitive_poses[0].position.z += pose.pose.position.z;
    }
    return true;
  }

  void Add()
  {
    calculateFinalPosition(trans_);
    for (long unsigned int i = 0; i < collision_objects_.size(); i++)
      collision_objects_[i].operation = collision_objects_[i].ADD;
    planning_scene_interface_.addCollisionObjects(collision_objects_);
    if (is_attached_)
      arm_group_.attachObject(collision_objects_[0].id, collision_objects_[0].header.frame_id);
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::planning_interface::MoveGroupInterface& arm_group_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
  bool is_attached_, is_current_;
  std_msgs::Header frame_id_;
  tf2_ros::Buffer& tf_;
  geometry_msgs::TwistStamped trans_;
};

}  // namespace engineer_middleware
