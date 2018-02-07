#ifndef RAY_TRACING_2D_H
#define RAY_TRACING_2D_H

#include <victim_localization/raytracing.h>


typedef geometry_msgs::Point Point;
typedef geometry_msgs::PoseStamped PoseStamped;

using namespace grid_map;
using namespace volumetric_mapping;



class Raytracing2D : public Raytracing
{
public:

  Raytracing2D(double map_res_);
  Raytracing2D(double map_res_, double HFOV_deg, double VFOV_deg, double max_d, double min_d);
  ~Raytracing2D();

  void GenerateOctomap(bool rebuild_once, bool publish);
  void generateMarkerArray(
      const std::string& tf_frame,
      visualization_msgs::MarkerArray* occupied_nodes,
      visualization_msgs::MarkerArray* free_nodes);

  double colorizeMapByHeight(double z, double min_z,
                             double max_z) const {
    return (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0), 1.0));
  }

  std_msgs::ColorRGBA percentToColor(double h)const;

  void Publish2DOctomap();
  void updateOctomapOccupancy(octomap::KeySet* free_cells,
                              octomap::KeySet* occupied_cells);
  void ResetOctomapRebuild();

  grid_map::GridMap Generate_2D_Safe_Plane(geometry_msgs::Pose p, bool publish_=false, bool castThroughUnkown=true);

  void publish_Map2(GridMap Map);

  // Publish markers for visualization.
  ros::Publisher occupied_nodes_pub_;
  ros::Publisher free_nodes_pub_;
  ros::Publisher pub_temp_map1;

  grid_map::GridMap map_;
  std::shared_ptr<octomap::OcTree> octree_2d;
  bool rebuild_octomap_once;
  double octomap_height;
  bool rebuild_octomap_;

  double sensor_max_range,visualize_max_z,visualize_min_z,map_publish_frequency;
  bool treat_unknown_as_occupied;

public:

  void  update();
  double compute2DRelativeRays();
  void compute2DRaysAtPose(geometry_msgs::Pose p);
  virtual void Initiate(bool publish, bool rebuild_once=false);
  void Done();


};

#endif // RAY_TRACING_2D_H
