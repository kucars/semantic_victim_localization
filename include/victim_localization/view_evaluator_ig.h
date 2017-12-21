#ifndef VIEW_EVALUATOR_IG_H
#define VIEW_EVALUATOR_IG_H

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeNode.h>

#include <octomap_world/octomap_manager.h>
#include <Eigen/Geometry>

class view_evaluator_IG
{
public:
  float info_selected_utility_;
  float info_entropy_total_;

  view_evaluator_IG();
  geometry_msgs::Pose getTargetPose();
  void setViewGenerator(view_generator_IG* v);
  void setMappingModule(Victim_Map_Base* m);
  void update_parameters();
  void evaluate();

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
  double info_distance_total_;
  std::vector<double> info_utilities_;

  double HFOV_deg;
  double VFOV_deg;
  double max_depth_d;
  double x_arena_max;
  double y_arena_max;
  std::string camera_frame;
  std::string base_frame;

  double tree_resolution;
  double const_;
  double calculateIG(geometry_msgs::Pose p);
  double getCellEntropy(Position cell_);
  double calculateDistance(geometry_msgs::Pose p);
  std::string getMethodName();




};

#endif // VIEW_EVALUATOR_IG_H
