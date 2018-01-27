#ifndef VEHICLE_CONTROL_IRIS_H
#define VEHICLE_CONTROL_IRIS_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "control/vehicle_control_base.h"
#include "victim_localization/common.h"

class VehicleControlIris : public VehicleControlBase
{
private:
  double uav_height_min_;
  double uav_height_max_;


public:
  ros::Subscriber sub_odom;
  ros::Subscriber sub_state;
  ros::Subscriber sub_setpoint;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  ros::Publisher  pub_setpoint;
  VehicleControlIris();

//private:

  void callbackOdometry(const nav_msgs::Odometry& odom_msg);
  void callbackState(const mavros_msgs::State& state_msg);
  void callbackSetpoint(const geometry_msgs::PoseStamped& ps_msg);
  void callbackSP(const geometry_msgs::PoseStamped::ConstPtr& msg);


  bool isReady();
  bool isStationary(double threshold_sensitivity = 1);

  void setOffboardState();
  void moveVehicle(double threshold_sensitivity = 1);
  void FollowPath(double threshold_sensitivity = 1);
  void setSpeed(double speed);
  void setPath(nav_msgs::Path path) ;
  void setPath(std::vector<geometry_msgs::Pose> path) ;
  void setWaypoint(double x, double y, double z, double yaw);
  void setWaypoint(double x, double y, double z, geometry_msgs::Quaternion orientation_);
  void setWaypoint(geometry_msgs::Pose p);
  void setWaypointIncrement(double x, double y, double z, double yaw);
  void start(double x, double y, double z, double yaw);
  void start();
  void rotateOnTheSpot();
  void rotateOnTheSpot(double x, double y, double z);
  void Evaluate4Points(double x, double y, double z);
  virtual geometry_msgs::Pose getlastSP();
  virtual ros::Time getlastSPTime();
  geometry_msgs::Pose transformSetpoint2Global (const geometry_msgs::Pose p_set);
  geometry_msgs::Pose transformGlobal2Setpoint (const geometry_msgs::Pose p_global);

};
#endif // VEHICLECONTROLIRIS_H
