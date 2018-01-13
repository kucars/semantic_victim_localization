#ifndef VICTIM_MAP_WIRELESS_H
#define VICTIM_MAP_WIRELESS_H

#include "victim_localization/victim_map_base.h"


class Victim_Map_Wireless: public Victim_Map_Base
{

private:
  std::string wireless_polygon_topic="polygon_wireless";
  std::string wireless_layer_name="victim_wireless";
  grid_map::Polygon wireless_polygon;
  Position victim_loc;

  grid_map::GridMap offline_map;
  double offline_map_resol;
  double detection_threshold;
  double wireless_error;

  ros::Publisher pub_map_offline;

public:
  Victim_Map_Wireless(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  void Update();
  void runDetector(){};
  void Generate_offline_wireless_map(double error);
  void Publish_Offline_Map();
};

#endif // VICTIM_MAP_WIRELESS_H
