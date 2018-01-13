#ifndef VICTIM_COMBINED_MAP_H
#define VICTIM_COMBINED_MAP_H

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <victim_localization/victim_map_thermal.h>
#include <victim_localization/victim_map_wireless.h>

class victim_map_combined : public Victim_Map_Base
{
public:
  victim_map_combined(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  victim_map_DL *victim_map_dl_;
  victim_map_Thermal *victim_map_Thermal_;
  Victim_Map_Wireless *victim_map_wireless_;


  std::string combinedMap_layer_name="victim_fused";
  double alpha;
  double beta;
  double gama;

  void Update();
  void setDroneCommunicator(drone_communicator *drone_comm_);


};



#endif // VICTIM_COMBINED_MAP_H
