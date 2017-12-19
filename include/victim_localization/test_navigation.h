#ifndef TEST_NAVIGATION_H
#define TEST_NAVIGATION_H

#include "victim_localization/reactive_path_planner.h"
#include "control/drone_communicator.h"
#include "victim_localization/volumetric_map_manager.h"
#include "mutex"
#include "thread"

namespace NavigationState {
enum State {
  STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
  NAVIGATION_WAYPOINT, WAITING_FOR_WAYPOINT, NAVIGATION_WAYPOINT_COMPLETE,
  NAVIGATION_PATH, WAITING_FOR_PATH, NAVIGATION_PATH_COMPLETE,
  IDEL,
};
}
class test_navigation
{
  tf::TransformListener tf_;
public:
  test_navigation(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private );
  ReactivePathPlanner *planner_;
  drone_communicator *drone_communicator_;
  volumetric_mapping::OctomapManager *manager_;
  Volumetric_Map *Volumetric_Map_;
  costmap_2d::Costmap2DROS *CostMapROS_;
  nav_msgs::Path path_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  geometry_msgs::Pose set_waypoint;
  nav_msgs::Path set_path;

 NavigationState::State state;

  void Configuration();
  void Takeoff();
  void executeplanner();
  void GetTestPath();
  void PublishCurrentPose(geometry_msgs::Pose p);
  void plannerthread();
  void PublishSPCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void state_machine();


  bool Selectpath_;
  int Num_points;
};

#endif // TEST_NAVIGATION_H
