#ifndef VEHICLE_CONTROL_FLOATING_SENSOR_H
#define VEHICLE_CONTROL_FLOATING_SENSOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "control/vehicle_control_base.h"
#include "victim_localization/common.h"

class VehicleControlFloatingSensor : public VehicleControlBase
{
private:
  ros::Subscriber sub_pose;
  ros::Publisher  pub_pose;
  ros::Publisher  pub_twist;

  double speed_;
  double time_to_target_;
  geometry_msgs::Twist twist_;

public:
  VehicleControlFloatingSensor();

  void callbackPose(const geometry_msgs::PoseStamped& pose_msg);

  bool isReady();
  bool isSationary(double threshold_sensitivity = 1);
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
  void updateTwist();
  void Evaluate4Points(double x, double y, double z);
  void rotateOnTheSpot();
  geometry_msgs::Pose getlastSP();
  ros::Time getlastSPTime();
};

#endif // VEHICLECONTROLIRIS_H
