#ifndef VICTIM_COMBINED_MAP_H
#define VICTIM_COMBINED_MAP_H

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <victim_localization/victim_map_thermal.h>
#include <victim_localization/victim_map_wireless_2.h>


class victim_map_combined : public Victim_Map_Base
{
public:
  victim_map_combined(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);



  std::string combinedMap_layer_name="victim_fused";
  double alpha;
  double beta;
  double gama;

  void Update();
  void setDroneCommunicator(vehicle_communicator *drone_comm_);
  void setVehicle(VehicleControlBase *vehicle);
  void setOctomapManager(volumetric_mapping::OctomapManager *manager);
  void SetNavMap(nav_msgs::OccupancyGridPtr Nav_map);
  void setCurrentPose(geometry_msgs::Pose ps);

  Victim_Map_Base *victim_map_dl_;
  Victim_Map_Base *victim_map_thermal_;
  Victim_Map_Base *victim_map_wireless_;

  Victim_Map_Base* getMapLayer(int map_number);

};



#endif // VICTIM_COMBINED_MAP_H
