#ifndef TEST_NAVIGATION_H
#define TEST_NAVIGATION_H

#include "victim_localization/reactive_path_planner.h"
#include "control/vehicle_communicator.h"
#include "victim_localization/volumetric_map_manager.h"
#include "mutex"
#include "thread"
#include "rviz_visual_tools/rviz_visual_tools.h"


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
  vehicle_communicator *drone_communicator_;
  volumetric_mapping::OctomapManager *manager_;

  rviz_visual_tools::RvizVisualToolsPtr visualTools;


  Volumetric_Map *Volumetric_Map_;
  costmap_2d::Costmap2DROS *CostMapROS_;
  nav_msgs::Path path_;
  nav_msgs::Path path_to_waypoint;


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  geometry_msgs::Pose set_waypoint;
  nav_msgs::Path set_path;
  double grid_size_x, grid_size_y;
  double grid_origin_x, grid_origin_y;


   std::vector<geometry_msgs::Pose> collection_of_poses;
   std::vector<std::vector<geometry_msgs::Pose> > Final_paths;
   NavigationState::State state;

  void Configuration();
  void Takeoff();
  void executeplanner();
  void GetTestPath();
  void GetTestPath2();
  void GetTestPath3();
  std::vector<geometry_msgs::Pose> Path_discretization(geometry_msgs::Pose p_1 , geometry_msgs::Pose p_2 , double step_size);


  void PublishCurrentPose(geometry_msgs::Pose p);
  void plannerthread();
  void PublishSPCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void state_machine();

  bool Selectpath_;
  int last_point;
  double number_of_path, Path_size;
};

#endif // TEST_NAVIGATION_H
