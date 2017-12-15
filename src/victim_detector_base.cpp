#include "victim_localization/victim_detector_base.h"


victim_detector_base::victim_detector_base()
{
}

victim_detector_base::~victim_detector_base(){}

void victim_detector_base::SetCurrentSetpoint(geometry_msgs::Pose setpoint)
{
 setpoint_.pose=setpoint;
 setpoint_.header.frame_id = "base_link";
 setpoint_.header.stamp = ros::Time::now();
}
