#ifndef IRIS_DRONE_COMMANDER_H
#define IRIS_DRONE_COMMANDER_H

#include "control/vehicle_control_iris.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "victim_localization/rotate_action.h"
#include "victim_localization/waypoint_action.h"
#include "victim_localization/path_action.h"
#include "victim_localization/path2_action.h"
#include "victim_localization/status_action.h"

namespace Command {
enum State {
  INITIALIZING,
  STARTING_DRONE,
  TAKEOFF,
  ROTATE,
  SEND_WAYPOINT,
  SEND_PATH,
  HOVER_IF_NO_WAYPOINT_RECEIVED,
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

  ros::Publisher  waypoint_status;
  ros::Publisher path_status;

  bool command_status;
  Command::State state;

  geometry_msgs::PoseStamped hoverPose;

  double start_x, start_y, start_z, start_yaw;

  ros::ServiceServer service_rotation;
  ros::ServiceServer service_waypoint;
  ros::ServiceServer service_path;
  ros::ServiceServer service_path2;
  ros::ServiceServer service_status;
  void Takeoff();
  void rotate();
  void start();
  bool execute_rotation(victim_localization::rotate_action::Request &request,
                     victim_localization::rotate_action::Response &respond);

  bool execute_waypoint(victim_localization::waypoint_action::Request &request,
                     victim_localization::waypoint_action::Response &respond);

  bool execute_path(victim_localization::path_action::Request &request,
                     victim_localization::path_action::Response &respond);

  bool execute_path2(victim_localization::path2_action::Request &request,
                     victim_localization::path2_action::Response &respond);

  bool check_status(victim_localization::status_action::Request &request,
                     victim_localization::status_action::Response &respond);

};

#endif // IRIS_DRONE_COMMANDER_H
