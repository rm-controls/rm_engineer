//
// Created by ljq on 2022/4/28.
//

#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace engineer_middleware
{
class PlanningScene
{
public:
  PlanningScene(const XmlRpc::XmlRpcValue& scene)
  {
    for (int i = 0; i < scene.size(); i++)
    {
      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = std::string(scene[i]["frame_id"]);
      collision_object.id = std::string(scene[i]["id"]);
      collision_object.primitives.push_back(Primitive(scene[i]["primitive"]));
      collision_object.primitive_poses.push_back(Pose(scene[i]["pose"]));
      collision_objects_.push_back(collision_object);
    }
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

  void Add()
  {
    for (long unsigned int i = 0; i < collision_objects_.size(); i++)
      collision_objects_[i].operation = collision_objects_[i].ADD;
    planning_scene_interface_.addCollisionObjects(collision_objects_);
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::CollisionObject> collision_objects_;
};

}  // namespace engineer_middleware
