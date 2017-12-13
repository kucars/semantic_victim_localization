#ifndef VICTIM_MAP_H
#define VICTIM_MAP_H

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
#include <victim_localization/raytracing.h>


typedef geometry_msgs::Point Point;
typedef geometry_msgs::PoseStamped PoseStamped;

using namespace grid_map;

struct detector_status {
  bool victim_found;
  Position victim_loc;
};


class Victim_Map_Base
{

protected://get it from config ..
  double Prob_D_H;  //P(D|H)
  double Prob_D_Hc;  //P(D|Hc)
  double Prob_Dc_H; //P(Dc|H)
  double Prob_Dc_Hc; //P(Dc|Hc)

public:
  std::string layer_name="general";//="victim";
  ros::NodeHandle nh_;
  ros::Publisher pub_map;
  ros::Subscriber sub_loc;
  ros::Publisher pub_polygon;


  float const_;
  geometry_msgs::Pose current_loc_;
  double current_yaw_;
  detector_status status;


  double HFOV_deg;
  double VFOV_deg;
  double max_depth_d;
  double x_arena_max;
  double y_arena_max;

  grid_map::GridMap map;
  grid_map::GridMap projected_map;
  grid_map::Polygon polygon;
  volumetric_mapping::OctomapManager *manager_;
  RayTracing *raytracing_;
  std::string victimMapName;



  //Detection_info//
  Position detect_loc_;
  bool is_detect_;
  Point p1; // rectangle corners for projected map update
  Point p2;
  Point p3;
  Point p4;

  //**************//

  virtual void Update(){};
  virtual detector_status getDetectionStatus();
  std::string VictimMpaType();



  Position approximate_detect(Position x);
  //bool valid(Position loc);
  void publish_Map();
  grid_map::Polygon draw_FOV();
  void callbackdrawFOV(const PoseStamped &ps_stamped);

  std::string getlayer_name();
  void setlayer_name(std::string layer_);
  void setCurrentPose(geometry_msgs::Pose ps);
  void setDetectionResult(detector_status status);
  void setOctomapManager(volumetric_mapping::OctomapManager *manager);
  void setRaytracing(RayTracing *Ray);

  grid_map::GridMap Project_3d_rayes_to_2D_plane();
  void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);
  grid_map::Polygon Update_region(grid_map::GridMap Map, geometry_msgs::Pose corner_);

  //%%%%%
  std::vector<Eigen::Vector3d> rays_far_plane_;
  std::vector<octomap::point3d> rays_far_plane_at_pose_;
  Eigen::Matrix3d  camera_rotation_mtx_; // Camera rotation mtx




  std::string camera_frame;
  std::string base_frame;

  double tree_resolution;
  double calculateIG(geometry_msgs::Pose p);

  double calculateIG_New(geometry_msgs::Pose p);
  double getCellEntropy(Index cell_);

  octomap::OcTree *tree_;
  bool isEntropyLow();
  bool isNodeInBounds(octomap::OcTreeKey &key);
  bool isNodeFree(octomap::OcTreeNode node);
  bool isNodeOccupied(octomap::OcTreeNode node);
  bool isNodeUnknown(octomap::OcTreeNode node);
  bool isPointInBounds(octomap::point3d &p);

  void   getCameraRotationMtxs();
  double getNodeOccupancy(octomap::OcTreeNode* node);
  double getNodeEntropy(octomap::OcTreeNode* node);
  int getPointCountAtOcTreeKey(octomap::OcTreeKey key);

  //%%%%
  double computeRelativeRays();
  void computeRaysAtPose(geometry_msgs::Pose p);



  Victim_Map_Base();
  ~Victim_Map_Base();

};




#endif // VICTIM_MAP_H
