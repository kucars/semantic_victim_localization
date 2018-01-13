#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "math.h"
#include <iostream>
#include <string>
#include <vector>
#include "geometry_msgs/PointStamped.h"
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <sensor_msgs/PointCloud.h>
#include "victim_localization/common.h"
#include <octomap_world/octomap_manager.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <victim_localization/view_generator_ig.h>
#include <control/drone_communicator.h>


typedef geometry_msgs::Point Point;
typedef geometry_msgs::PoseStamped PoseStamped;

using namespace grid_map;

struct OctomapKeyCompare {
  bool operator() (const octomap::OcTreeKey& lhs, const octomap::OcTreeKey& rhs) const
  {
    size_t h1 = size_t(lhs.k[0]) + 1447*size_t(lhs.k[1]) + 345637*size_t(lhs.k[2]);
    size_t h2 = size_t(rhs.k[0]) + 1447*size_t(rhs.k[1]) + 345637*size_t(rhs.k[2]);
    return h1< h2;
  }
};

class Raytracing
{
public:
  geometry_msgs::Pose current_pose_;
  std::shared_ptr<octomap::OcTree> tree_;
  drone_communicator *drone_comm;
  nav_msgs::OccupancyGridPtr grid_;

  std::string Layer_name_;
  double HFOV_deg;
  double VFOV_deg;
  double range_max_;
  double range_min_;
  double x_arena_max;
  double y_arena_max;
  std::string camera_frame;
  std::string base_frame;
  double tree_resolution;
  Eigen::Matrix3d camera_rotation_mtx_; // Camera rotation mtx
  double map_res;
  ros::Publisher pub_temp_map;
  ros::NodeHandle nh_;


std::vector<Eigen::Vector3d> rays_far_plane_;
std::vector<octomap::point3d> rays_far_plane_at_pose_;

double nav_bounds_x_max_, nav_bounds_y_max_, nav_bounds_z_max_;
double nav_bounds_x_min_, nav_bounds_y_min_, nav_bounds_z_min_;

virtual grid_map::GridMap Generate_2D_Safe_Plane(geometry_msgs::Pose p);
virtual grid_map::GridMap Generate_2D_Safe_Plane(geometry_msgs::Pose p, bool publish_);


Raytracing(double map_res_);
Raytracing(double map_res_, double HFOV_deg, double VFOV_deg, double max_d, double min_d);
~Raytracing();

void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);
bool isInsideBounds(Position p);

public:

  bool isPointInBounds(octomap::point3d &p);
  void   getCameraRotationMtxs();
  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  int getPointCountAtOcTreeKey(octomap::OcTreeKey key);
  double computeRelativeRays();
  void computeRaysAtPose(geometry_msgs::Pose p);
  bool  isInsideRegionofInterest(double z , double tolerance=0.5);
  void  setDroneCommunicator(drone_communicator *drone_comm_);
  void  publish_Map(grid_map::GridMap Map);
  void SetNavMap(nav_msgs::OccupancyGridPtr Nav_map);
  virtual void  update();
  virtual void Initiate(bool rebuild_once, bool publish){};
  virtual void Done(){};


};

#endif // RAY_TRACING_H
