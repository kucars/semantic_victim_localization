#ifndef VEHICLE_CONTROL_IRIS_ORIGIN_H
#define VEHICLE_CONTROL_IRIS_ORIGIN_H

#include <control/vehicle_control_iris.h>

class VehicleControlIrisOrigin : public VehicleControlIris
{
public:
  VehicleControlIrisOrigin();

  void setOffboardState();
  //void moveVehicle(double threshold_sensitivity);

  geometry_msgs::Pose transformSetpoint2Global (const geometry_msgs::Pose p_set);
  geometry_msgs::Pose transformGlobal2Setpoint (const geometry_msgs::Pose p_global);
};

#endif // VEHICLE_CONTROL_IRIS_ORIGIN_H
