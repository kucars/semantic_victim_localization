#ifndef VICTIM_MAP_WIRELESS_2_H
#define VICTIM_MAP_WIRELESS_2_H

#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_wireless_detector.h"


class Victim_Map_Wireless: public Victim_Map_Base
{

private:
  std::string wireless_polygon_topic="polygon_wireless";
  std::string wireless_layer_name="victim_wireless";
  Position victim_loc;
  double vision_map_resol;
  double max_wireless_range;

public:
  Victim_Map_Wireless(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  victim_wireless_detector *detector_;
  void Update();
  void runDetector();
  virtual void setDetectionResult(Status detection_status);

  double Distance_to_center(Position p1, Position p2);


};

#endif // VICTIM_MAP_WIRELESS_2_H
