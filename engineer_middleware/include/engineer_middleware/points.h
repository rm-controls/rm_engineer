//
// Created by lsy on 23-5-9.
//

#pragma once

#include <algorithm>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace engineer_middleware
{
class Points
{
public:
  Points() = default;
  ~Points() = default;
  struct Point
  {
    double x, y, z, distance;
    static bool comparePointsDistance(const Point& a, const Point& b)
    {
      return a.distance < b.distance;
    }
  };
  enum Geometry
  {
    SPHERE,
    BASICS
  };

  void generateSpherePoints(double center_x, double center_y, double center_z, double r, double point_resolution)
  {
    points_final_.clear();
    std::vector<Point> points;
    double angular_resolution = 2 * M_PI * (1 - point_resolution);

    for (double r_temp = 0.001; r_temp <= r; r_temp += r * (1 - point_resolution))
    {
      for (double phi = -M_PI; phi <= M_PI; phi += angular_resolution)
      {
        for (double theta = 0.; theta <= M_PI; theta += angular_resolution)
        {
          double x = center_x + r_temp * sin(phi) * cos(theta);
          double y = center_y + r_temp * sin(phi) * sin(theta);
          double z = center_z + r_temp * cos(phi);
          double distance = sqrt(pow(x - center_x, 2) + pow(y - center_y, 2) + pow(z - center_z, 2));
          points.push_back({ x, y, z, distance });
        }
      }
    }
    std::sort(points.begin(), points.end(), Point::comparePointsDistance);
    for (auto& p : points)
    {
      points_final_.push_back(p);
    }
  }
  sensor_msgs::PointCloud2 getPointCloud2()
  {
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "exchanger";
    cloud.points.resize(points_final_.size());

    cloud.channels.resize(1);
    cloud.channels[0].name = "motion";
    cloud.channels[0].values.resize(points_final_.size());

    for (int i = 0; i < (int)points_final_.size(); ++i)
    {
      cloud.points[i].x = points_final_[i].x;
      cloud.points[i].y = points_final_[i].y;
      cloud.points[i].z = points_final_[i].z;
    }
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
    return cloud2;
  }
  void rectifyForRPY(double theta, double beta, double k_x, double k_theta, double k_beta)
  {
    geometry_msgs::Vector3 rectify;
    rectify.x = abs(points_final_[0].x) * sin(theta) * k_x;
    rectify.y = abs(points_final_[0].x) * sin(beta) * k_beta;
    rectify.z = abs(points_final_[0].x) * sin(theta) * k_theta;
    for (int i = 0; i < (int)points_final_.size(); ++i)
    {
      points_final_[i].x += rectify.x;
      points_final_[i].y += rectify.y;
      points_final_[i].z -= rectify.z;
    }
  }
  void rectifyForLink7(double theta, double link7_length)
  {
    for (int i = 0; i < (int)points_final_.size(); ++i)
    {
      points_final_[i].x -= link7_length * pow(sin(theta), 2) / (tan(M_PI_2 - theta / 2));
      points_final_[i].z = link7_length * sin(theta) * cos(theta) / (tan(M_PI_2 - theta / 2));
    }
  }
  void generateBasicsPoints(double center_x, double center_y, double center_z, double x_length, double y_length,
                            double z_length, double point_resolution)
  {
    points_final_.clear();
    std::vector<Point> points;
    double resolution = 1 - point_resolution;
    for (double x = center_x - x_length / 2; x <= center_x + x_length / 2; x += x_length * resolution)
    {
      for (double y = center_y - y_length / 2; y <= center_y + y_length / 2; y += y_length * resolution)
      {
        for (double z = center_z - z_length / 2; z <= center_z + z_length / 2; z += z_length * resolution)
        {
          double distance = sqrt(pow(x - center_x, 2) + pow(y - center_y, 2) + pow(z - center_z, 2));
          points.push_back({ x, y, z, distance });
        }
      }
    }
    std::sort(points.begin(), points.end(), Point::comparePointsDistance);
    for (auto& p : points)
    {
      points_final_.push_back(p);
    }
  }

  void cleanPoints()
  {
    points_final_.clear();
  }
  std::vector<Point> getPoints()
  {
    return points_final_;
  }

  void generateGeometryPoints()
  {
    switch (shape_)
    {
      case SPHERE:
        generateSpherePoints(target_x_, target_y_, target_z_, radius_, point_resolution_);
        break;
      case BASICS:
        generateBasicsPoints(target_x_, target_y_, target_z_, x_length_, y_length_, z_length_, point_resolution_);
        break;
    }
  }

  void setValue(Geometry shape, double x, double y, double z, double radius, double point_resolution)
  {
    shape_ = shape;
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    radius_ = radius;
    point_resolution_ = point_resolution;
  }
  void setValue(Geometry shape, double x, double y, double z, double x_length, double y_length, double z_length,
                double point_resolution)
  {
    shape_ = shape;
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    x_length_ = x_length;
    y_length_ = y_length;
    z_length_ = z_length;
    point_resolution_ = point_resolution;
  }

private:
  Geometry shape_;
  geometry_msgs::PoseStamped target_;
  std::vector<Point> points_final_{};
  double target_x_, target_y_, target_z_, x_length_, y_length_, z_length_, point_resolution_, radius_;
};

}  // namespace engineer_middleware
