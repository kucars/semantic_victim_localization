#ifndef VICTIM_MAP_DL_H
#define VICTIM_MAP_DL_H

#include "victim_localization/victim_map_base.h"



class victim_map_DL: public Victim_Map_Base
{

private:
  std::string DL_map_topic="victim_map/grid_map_DL";
  std::string DL_polygon_topic="polygon_DL";
  std::string DL_layer_name="victim_DL";
  detector_status DL_status;


public:
  victim_map_DL();
  void Update();
  detector_status getDetectionStatus();

};

#endif // VICTIM_MAP_DL_H
