#ifndef VICTIM_MAP_DL_H
#define VICTIM_MAP_DL_H

#include "victim_localization/victim_map_base.h"



class victim_map_Thermal: public Victim_Map_Base
{

private:
  std::string thermal_map_topic="victim_map/grid_map_thermal";
  std::string thermal_polygon_topic="polygon_thermal";
  std::string Thermal_layer_name="victim_thermal";
  double thermal_img_x_res;
  double thermal_img_y_res;
  double thermal_x_offset;
  double thermal_y_offset;
  Position thermal_vic_loc;

  Position ThermalRayStart;
  Position ThermalRayEnd;
  grid_map::Polygon ThermalRay;
  std::vector<Position> ThermalRay;

public:
  victim_map_Thermal();
  void Update();
  void GetCameraCenter2World (geometry_msgs::PoseStamped &CamCentertoWorld);
  void GenerateRayVector(grid_map::GridMap Map,Position start,Position End);
  bool IsInsideRay(Position P);

  tf::TransformListener *tf_listener;
};

#endif // VICTIM_MAP_DL_H
