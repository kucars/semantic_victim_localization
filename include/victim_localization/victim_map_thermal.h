#ifndef VICTIM_MAP_THERMAL_H
#define VICTIM_MAP_THERMAL_H

#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_thermal_detector.h"



class victim_map_Thermal: public Victim_Map_Base
{

private:
  std::string thermal_polygon_topic="polygon_thermal";
  std::string Thermal_layer_name="victim_thermal";
  double thermal_img_x_res;
  double thermal_img_y_res;
  double thermal_x_offset;
  double thermal_y_offset;
  double max_thermal_d;
  double min_thermal_d;

  Position ThermalRayStart;
  Position ThermalRayEnd;
  std::vector<Position> ThermalRay;

  rviz_visual_tools::RvizVisualToolsPtr visualize_thermal_ray;


public:
  victim_map_Thermal(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  victim_thermal_detector *detector_;
  void Update();
  void runDetector();
  void GetCameraCenter2World (geometry_msgs::PoseStamped &CamCentertoWorld, ros::Time caputre_time);
  void GenerateRayVector(grid_map::GridMap Map,Position start,Position End);
  bool IsInsideRay(Position P);

  tf::TransformListener *tf_listener;

};

#endif // VICTIM_MAP_THERMAL_H
