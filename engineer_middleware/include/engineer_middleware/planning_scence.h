//
// Created by ljq on 2022/4/28.
//

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace engineer_middleware
{
class PlanningScence
{
public:
  PlanningScence(const XmlRpc::XmlRpcValue& scence)
  {
    for (int i = 0; i < scence.size(); i++)
    {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = std::string(scence[i]["frame_id"]);
      collision_object.id = std::string(scence[i]["id"]);
      object_ids.push_back(collision_object.id);
      shape_msgs::SolidPrimitive primitives;
      primitives = Primitive(scence[i]["primitive"]);

      geometry_msgs::Pose pose;
      pose = Pose(scence[i]["pose"]);

      collision_object.primitives.push_back(primitives);
      collision_object.primitive_poses.push_back(pose);

      collision_objects.push_back(collision_object);
    }
  };

  shape_msgs::SolidPrimitive Primitive(const XmlRpc::XmlRpcValue& primitives)
  {
    shape_msgs::SolidPrimitive primitive;
    if (std::string(primitives["type"]) == "box")
    {
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = primitives["dimension"][0];
      primitive.dimensions[primitive.BOX_Y] = primitives["dimension"][1];
      primitive.dimensions[primitive.BOX_Z] = primitives["dimension"][2];
    }
    return primitive;
  };

  geometry_msgs::Pose Pose(const XmlRpc::XmlRpcValue& poses)
  {
    geometry_msgs::Pose pose;

    pose.orientation.x = xmlRpcGetDouble(poses["orientation"], 0);
    pose.orientation.y = xmlRpcGetDouble(poses["orientation"], 1);
    pose.orientation.z = xmlRpcGetDouble(poses["orientation"], 2);
    pose.orientation.w = xmlRpcGetDouble(poses["orientation"], 3);

    pose.position.x = xmlRpcGetDouble(poses["position"], 0);
    pose.position.y = xmlRpcGetDouble(poses["position"], 1);
    pose.position.z = xmlRpcGetDouble(poses["position"], 2);

    return pose;
  }

  void Add()
  {
    for (long unsigned int i = 0; i < collision_objects.size(); i++)
    {
      collision_objects[i].operation = collision_objects[i].ADD;
    }
    planning_scene_interface.addCollisionObjects(collision_objects);
  }

  void Delete()
  {
    planning_scene_interface.removeCollisionObjects(object_ids);
  }

  std::vector<std::string> object_ids;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
};

}  // namespace engineer_middleware
