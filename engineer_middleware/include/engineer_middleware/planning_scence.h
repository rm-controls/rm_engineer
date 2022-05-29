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
      collision_object.primitives.push_back(Primitive(scence[i]["primitive"]);
      collision_object.primitive_poses.push_back(Pose(scence[i]["pose"]));
      collision_objects.push_back(collision_object);
      object_ids.push_back(collision_object.id);
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
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.CONE_HEIGHT] = xmlRpc["dimension"][0];
      primitive.dimensions[primitive.CONE_RADIUS] = xmlRpc["dimension"][1];
    }
    if (std::string(xmlRpc["type"]) == "cylinder")
    {
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = xmlRpc["dimension"][0];
      primitive.dimensions[primitive.CYLINDER_RADIUS] = xmlRpc["dimension"][1];
    }
    if (std::string(xmlRpc["type"]) == "sphere")
    {
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(3);
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
    for (long unsigned int i = 0; i < collision_objects.size(); i++)
      collision_objects[i].operation = collision_objects[i].ADD;
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
