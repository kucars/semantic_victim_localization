#ifndef VEHICLE_CONTROL_BASE_H
#define VEHICLE_CONTROL_BASE_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

#include "victim_localization/common.h"

class VehicleControlBase
{
protected:
  bool is_ready_;

public:
  double distance_threshold_;
  double angular_threshold_;
  double linear_speed_threshold_;
  double angular_speed_threshold_;

  geometry_msgs::Pose  vehicle_current_pose_;
  geometry_msgs::Twist vehicle_current_twist_;
  mavros_msgs::State vehicle_current_state_;

  geometry_msgs::Pose setpoint_;
  std::vector<geometry_msgs::Pose> setpath_;

  VehicleControlBase();
  virtual void initialize(){};
  virtual void start(double x, double y, double z, double yaw){};
  virtual void setOffboardState(geometry_msgs::PoseStamped ps){};
  virtual bool isReady(){};
  virtual bool isStationary(double threshold_sensitivity){};
  virtual void setSpeed(double speed){};
  virtual void setWaypoint(double x, double y, double z, double yaw){};
  virtual void setWaypoint(double x, double y, double z, geometry_msgs::Quaternion orientation_){};
  virtual void setWaypoint(geometry_msgs::Pose p){};
  virtual void setWaypointIncrement(double x, double y, double z, double yaw){};
  virtual void moveVehicle(double threshold_sensitivity = 1){};
  virtual void rotateOnTheSpot(){};


  geometry_msgs::Pose  getPose();
  geometry_msgs::Twist getTwist();
  geometry_msgs::Point getPosition();
  geometry_msgs::Quaternion getOrientation();
  double getYaw();
  mavros_msgs::State getState();


  double getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  bool   isNear(double p1, double p2, double threshold_sensitivity );
  bool   isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current, double threshold_sensitivity);
};

#endif // VEHICLECONTROLBASE_H
