#ifndef DRONE_COMMUNICATOR_H
#define DRONE_COMMUNICATOR_H

#include "control/vehicle_control_iris.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "victim_localization/volumetric_map_manager.h"
#include "victim_localization/rotate_action.h"
#include "victim_localization/path_action.h"
#include "victim_localization/path2_action.h"
#include "victim_localization/waypoint_action.h"
#include "victim_localization/status_action.h"
#include "rviz_visual_tools/rviz_visual_tools.h"



namespace Command {
enum State {
  INITIALIZING,
  STARTING_DRONE,
  TAKEOFF,
  READY_FOR_WAYPOINT,
};
}

class drone_communicator
{
public:
  drone_communicator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                     volumetric_mapping::OctomapManager *mapManager);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  volumetric_mapping::OctomapManager *manager_;

  ros::Publisher pub_waypoint;
  ros::Publisher pub_path;

  ros::Subscriber sub_pose;
  ros::Subscriber sub_command_status;

  ros::ServiceClient clientExecuteRotation ;
  ros::ServiceClient clientExecuteWaypoint;
  ros::ServiceClient clientExecutePath;
  ros::ServiceClient clientExecutePath2;
  ros::ServiceClient ClientCheckStatus;

  victim_localization::rotate_action rotate_srv;
  victim_localization::waypoint_action waypoint_srv;
  victim_localization::path_action path_srv;
  victim_localization::path2_action path2_srv;
  victim_localization::status_action status_srv;

  geometry_msgs::Pose Waypoint_;
  nav_msgs::Path Path_;

  geometry_msgs::Pose current_pose;

  bool current_drone_status;

  rviz_visual_tools::RvizVisualToolsPtr visualTools;


  geometry_msgs::Pose GetPose();
  void Check_MapManager_and_Drone_Ready();
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void callbackCommandStatus(const std_msgs::Bool::ConstPtr& msg);
  bool GetStatus();

  ros::Time previous_time;

  bool Execute_waypoint(geometry_msgs::Pose p);
  bool Execute_path(nav_msgs::Path path);
  bool Execute_path(std::vector<geometry_msgs::Pose> path);

};

#endif // DRONE_COMMUNICATOR_H
