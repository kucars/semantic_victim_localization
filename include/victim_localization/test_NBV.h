#ifndef TEST_FLIGHT_H
#define TEST_FLIGHT_H

#include <victim_localization/common.h>

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <victim_localization/victim_map_thermal.h>
#include <victim_localization/victim_map_combined.h>
#include <control/vehicle_control_base.h>
#include <control/vehicle_control_floating_sensor.h>
#include <control/vehicle_control_iris.h>
#include <control/vehicle_control_iris_origin.h>
#include <control/vehicle_communicator.h>
#include <victim_localization/victim_detector_base.h>
#include <ssd_keras/ssd_detection_with_ecludian_clustering.h>
#include <victim_localization/victim_thermal_detector.h>
#include <victim_localization/view_evaluator_base.h>
#include <victim_localization/view_evaluator_ig.h>
#include <victim_localization/view_evaluator_max_max.h>
#include <victim_localization/view_evaluator_max_min.h>
#include <victim_localization/view_evaluator_max_sum.h>
#include <victim_localization/view_evaluator_min_neigh.h>
#include "victim_localization/view_evaluator_log_reward.h"
#include <victim_localization/view_evaluator_ig_exp.h>
#include <victim_localization/view_evaluator_ig_exp_max.h>

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/view_generator_ig_nn_adaptive.h>
#include <victim_localization/view_generator_ig_frontier.h>
#include <victim_localization/view_generator_ig_adaptive_frontier.h>
#include <victim_localization/volumetric_map_manager.h>
#include <victim_localization/nbv_history.h>

#include <victim_localization/navigation_base.h>
#include <victim_localization/reactive_path_planner.h>
#include <victim_localization/straightline.h>

#include "victim_localization/IterationInfo.h"

#include <victim_localization/victim_map_wireless_2.h>
#include <victim_localization/victim_wireless_detector.h>



namespace NBVState {
enum State {
  INITIALIZING,
  IDLE,
  STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
  START_MAP_UPDATE, UPDATE_MAP_COMPLETE,
  NAVIGATION, NAVIGATION_COMPLETE,
  MOVING, MOVING_COMPLETE,
  VIEWPOINT_EVALUATION,
  VIEWPOINT_EVALUATION_COMPLETE,
  VIEWPOINT_GENERATION,
  VIEWPOINT_GENERATION_COMPLETE,
  TERMINATION_MET,
  TERMINATION_NOT_MET,
};
}


class TestNBZ
{

  tf::TransformListener tf_;
public:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Publisher  pub_iteration_info;
  TimeProfiler timer;

  rviz_visual_tools::RvizVisualToolsPtr visualTools;
  ros::Time t_start;


  vehicle_communicator *drone_communicator_;
  VehicleControlBase *vehicle_;
  Victim_Map_Base *Map_;
  nbv_history *history_;

  volumetric_mapping::OctomapManager *manager_;
  Volumetric_Map *Occlusion_Map_;
  costmap_2d::Costmap2DROS *CostMapROS_;
  victim_detector_base *detector_;

  int map_type;
  int nav_type;
  int view_generator_type;
  int view_evaluator_type;

  //debug
 ros::Time T1;
 ros::Duration D;
  ros::Rate NBV_loop_rate;

  //Navigation
  ReactivePathPlanner *navigation_;
  double grid_size_x,grid_size_y;
  double grid_origin_x,grid_origin_y;
  std::vector<geometry_msgs::Pose> path_to_waypoint;


  view_generator_IG *view_generate_;
  view_evaluator_base *View_evaluate_;

  TestNBZ(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
  NBVState::State state;
  bool is_done_map_update;
  int waypointNum;
  void runStateMachine();
  void initVehicle();
  void initMap();
  void initOctomap();
  void initParameters();
  void initNavigation();
  void initViewGenerator();
  void initViewEvaluator();



  void generateViewpoints();
  void evaluateViewpoints();
  void navigate();
  void UpdateMap();
  void updateHistory();

  bool detection_enabled; //for debugging

  geometry_msgs::Pose p_;


};



#endif // TEST_FLIGHT_H




