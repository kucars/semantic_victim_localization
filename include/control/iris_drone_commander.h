#ifndef IRIS_DRONE_COMMANDER_H
#define IRIS_DRONE_COMMANDER_H

#include "control/vehicle_control_iris.h"

namespace Command {
enum State {
  INITIALIZING,
  STARTING_DRONE,
  TAKEOFF,
  READY_FOR_WAYPOINT,
};
}

class iris_drone_commander
{
public:
  iris_drone_commander(const ros::NodeHandle &nh, const ros::NodeHandle &nhPrivate);
  VehicleControlIris *vehicle_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher  localPosePub;

  geometry_msgs::PoseStamped hoverPose;
  geometry_msgs::PoseStamped currentGoal;
  geometry_msgs::PoseStamped goalPose;

  void Takeoff();
  void start();


  // communication with the victim localization node
  bool FollowingPath_succeed;
  bool StartFollowPath;
  Command::State state;
};

#endif // IRIS_DRONE_COMMANDER_H
