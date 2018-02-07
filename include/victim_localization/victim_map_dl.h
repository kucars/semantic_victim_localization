#ifndef VICTIM_MAP_DL_H
#define VICTIM_MAP_DL_H

#include "victim_localization/victim_map_base.h"
#include "ssd_keras/ssd_detection_with_ecludian_clustering.h"



class victim_map_DL: public Victim_Map_Base
{

private:
  std::string DL_polygon_topic="polygon_DL";
  std::string DL_layer_name="victim_DL";

  SSD_Detection_with_clustering *detector_;


public:
  victim_map_DL(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  void Update();
  void runDetector();



};

#endif // VICTIM_MAP_DL_H
