#include "victim_localization/victim_detector_base.h"


victim_detector_base::victim_detector_base()
{
}

victim_detector_base::~victim_detector_base(){}


void victim_detector_base::SetVehicleCommunicator(vehicle_communicator *vehicle_comm)
{
  vehicle_comm_=vehicle_comm;
}

ros::Duration victim_detector_base::GetConnectionTime()
{
  return ros::Duration(0);
}
