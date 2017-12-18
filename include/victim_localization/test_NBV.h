#ifndef TEST_FLIGHT_H
#define TEST_FLIGHT_H

#include <victim_localization/common.h>

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <victim_localization/victim_map_thermal.h>
#include <victim_localization/victim_map_combined.h>
#include <control/vehicle_control_base.h>
#include <control/vehicle_control_iris.h>
#include <victim_localization/victim_detector_base.h>
#include <ssd_keras/ssd_detection_with_ecludian_clustering.h>
#include <victim_localization/victim_thermal_detector.h>
#include <victim_localization/view_evaluator_ig.h>
#include <victim_localization/view_generator_ig.h>
#include <victim_localization/view_generator_ig_nn_adaptive.h>
#include <victim_localization/view_generator_ig_frontier.h>
#include <victim_localization/volumetric_map_manager.h>
#include <victim_localization/nbv_history.h>

#include <victim_localization/navigation_base.h>
#include <victim_localization/reactive_path_planner.h>
#include <victim_localization/straightline.h>



namespace NBVState {
enum State {
  INITIALIZING,
  IDLE,
  STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
  UPDATE_MAP, UPDATE_MAP_COMPLETE,
  NAVIGATION, NAVIGATION_COMPLETE,
  MOVING, MOVING_COMPLETE,
  VIEWPOINT_EVALUATION,
  VIEWPOINT_EVALUATION_COMPLETE,
  VIEWPOINT_GENERATION,
  VIEWPOINT_GENERATION_COMPLETE,
  STARTING_DETECTION,
  DETECTION_COMPLETE,
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

  VehicleControlBase *vehicle_;
  Victim_Map_Base *Map_;
  RayTracing *Ray;
  nbv_history *history_;

  volumetric_mapping::OctomapManager *manager_;
  Volumetric_Map *Occlusion_Map_;
  costmap_2d::Costmap2DROS *CostMapROS_;

  victim_detector_base *detector_;

  //Navigation
  navigationBase *navigation_;

  view_generator_IG *view_generate_;
  view_evaluator_IG *View_evaluate_;

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


  void generateViewpoints();
  void evaluateViewpoints();
  void navigate();
  void UpdateMap();
  void updateHistory();

  bool detection_enabled; //for debugging

  geometry_msgs::Pose p_;


};



#endif // TEST_FLIGHT_H




