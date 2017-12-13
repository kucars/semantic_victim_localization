#ifndef VIEW_GENERATOR_IG_H
#define VIEW_GENERATOR_IG_H


#include <iostream>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <victim_localization/common.h>
#include <octomap_world/octomap_manager.h>
#include <victim_localization/nbv_history.h>
#include <victim_localization/Volumetric_Map_Manager.h>


//typedef geometry_msgs::Pose Pose;

class view_generator_IG
{

protected:
   double res_x_, res_y_, res_z_, res_yaw_;

   double nav_bounds_x_max_, nav_bounds_y_max_, nav_bounds_z_max_;
   double nav_bounds_x_min_, nav_bounds_y_min_, nav_bounds_z_min_;

   double uav_fixed_height;
   double extensionRange_;
   Eigen::Vector3d boundingbox_;
   double boundingbox_x_,boundingbox_y_,boundingbox_z_;
   double dOvershoot_;

public:
   //....variables....
  volumetric_mapping::OctomapManager *manager_;
  nbv_history *nbv_history_;
  Volumetric_Map *occlusion_map;
  double obj_bounds_x_max_, obj_bounds_y_max_, obj_bounds_z_max_;
  double obj_bounds_x_min_, obj_bounds_y_min_, obj_bounds_z_min_;
  geometry_msgs::Pose current_pose_;
  std::vector<geometry_msgs::Pose> generated_poses;
  std::vector<geometry_msgs::Pose> rejected_poses;

  // Visualizer
  int vis_marker_array_prev_size_;
  int vis_sphere_counter_;
  ros::Publisher pub_view_marker_array_;
  ros::Publisher pub_view_drone_marker_;

  //...methods...
  view_generator_IG();
  void setCurrentPose(geometry_msgs::Pose p);
  void setHistory(nbv_history* h);
  void setOcclusionMap(Volumetric_Map* Occ);


  bool isInsideBounds(geometry_msgs::Pose p);
  bool isSafe(geometry_msgs::Pose p);
  bool isValidViewpoint(geometry_msgs::Pose p);

  //generate views
  void generateViews(bool check);
  virtual void generateViews(); //viewpoints is  generated at current pose


 //visualize
  void visualize(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses, geometry_msgs::Pose selected_pose);
  visualization_msgs::Marker visualizeDeleteArrowMarker(int id);
  visualization_msgs::Marker visualizeCreateArrowMarker(int id, geometry_msgs::Pose pose, bool valid, double max_z = 0, double min_z = 0);
  void visualizeSelectedArrowMarker(geometry_msgs::Pose selected_pose, visualization_msgs::MarkerArray &All_poses);

  bool ComparePoses(geometry_msgs::Pose Pose1, geometry_msgs::Pose Pose2);
  void visualizeDrawSphere(geometry_msgs::Pose p, double r);
  void setOctomapManager(volumetric_mapping::OctomapManager *manager);


};

#endif // VIEW_GENERATOR_IG_H
