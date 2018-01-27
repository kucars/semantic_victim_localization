#include "control/vehicle_control_iris_origin.h"

VehicleControlIrisOrigin::VehicleControlIrisOrigin():
VehicleControlIris()
{
}

void VehicleControlIrisOrigin::setOffboardState()
{
  std::cout << cc.yellow << "Setoffboard is not implemented in the old Firmware\n" << cc.reset;
}


geometry_msgs::Pose VehicleControlIrisOrigin::transformSetpoint2Global (const geometry_msgs::Pose p_set)
{
  geometry_msgs::Pose p_global;

  // Apply a 90 degree clockwise rotation on the z-axis
  p_global.position.x = p_set.position.y;
  p_global.position.y =-p_set.position.x;
  p_global.position.z = p_set.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_set.orientation);
  yaw -= M_PI_2;
  p_global.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  return p_global;
}


geometry_msgs::Pose VehicleControlIrisOrigin::transformGlobal2Setpoint (const geometry_msgs::Pose p_global)
{
  geometry_msgs::Pose p_set;

  // Apply a 90 degree anti-clockwise rotation on the z-axis
  p_set.position.x =-p_global.position.y;
  p_set.position.y = p_global.position.x;
  p_set.position.z = p_global.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_global.orientation);
  yaw += M_PI_2;
  p_set.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  std::cout << "transformed setpoint is...." << p_set << std::endl;
  return p_set;


}

