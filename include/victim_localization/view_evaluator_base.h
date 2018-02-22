#ifndef VIEW_EVALUATOR_BASE_H
#define VIEW_EVALUATOR_BASE_H
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <octomap_world/octomap_manager.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>

class view_evaluator_base
{
public:
  float info_selected_utility_;
  float info_dl_selected_utility_;
  float info_thermal_selected_utility_;
  float info_wireless_selected_utility_;

  float info_selected_direction_; // this param is used for in wireless Viewpoint evaluation
  float info_entropy_total_;
  float info_dl_entropy_total_;
  float info_thermal_entropy_total_;
  float info_wireless_entropy_total_;


  view_evaluator_base();
  geometry_msgs::Pose getTargetPose();
  void setViewGenerator(view_generator_IG* v);
  void setMappingModule(Victim_Map_Base* m);
  void update_parameters();
  virtual void evaluate();
  double getFOVMAXEntropy(double Num, Victim_Map_Base *mapping_module);


   void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);

   view_generator_IG *view_gen_;
   Victim_Map_Base *mapping_module_;


public:
   ros::NodeHandle nh_;
   std::shared_ptr<octomap::OcTree> tree_;
  //ros::NodeHandle nh_private_;
  ros::Subscriber pointcloud_sub_;


  //volumetric_mapping::OctomapManager *manager_;

  geometry_msgs::Pose current_pose_;
  double current_yaw_;
  std::string MapLayer;
  geometry_msgs::Pose selected_pose_;
  int selected_index;
  double info_distance_total_;
  std::vector<double> info_utilities_;

  double HFOV_deg;
  double VFOV_deg;
  double max_depth_d;
  double x_arena_max;
  double y_arena_max;
  double wireless_max_range;
  std::string camera_optical_frame;
  std::string base_frame;

  double tree_resolution;
  double const_;
  double calculateIG(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
  virtual double calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);

  //--------- functions for wireless map which overwrite the original equivalent functions
  virtual void evaluateWireless();
  double calculateWirelessIG(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
  virtual double calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
  //----------

  //--------- functions for wireless map which overwrite the original equivalent functions
  virtual void evaluateCombined();
  virtual double calculateCombinedUtility(geometry_msgs::Pose p);
  double alpha;
  double beta;
  double gama;
  //----------

  double getCellEntropy(Position cell_,Victim_Map_Base *mapping_module);
  double calculateDistance(geometry_msgs::Pose p);
  bool IsSamePosition(geometry_msgs::Pose p1,geometry_msgs::Pose p2);
  virtual std::string getMethodName();

};

#endif // VIEW_EVALUATOR_BASE_H
