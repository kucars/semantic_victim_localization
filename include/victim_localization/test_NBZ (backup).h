#ifndef TEST_FLIGHT_H
#define TEST_FLIGHT_H

#include <victim_localization/common.h>

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <control/vehicle_control_base.h>
#include <control/vehicle_control_iris.h>
#include <ssd_keras/ssd_detection_with_ecludian_clustering.h>
#include <victim_localization/view_evaluator_ig.h>
#include <victim_localization/view_generator_ig.h>


namespace NBVState {
enum State {
  INITIALIZING,
  IDLE,
  STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
  UPDATE_MAP, UPDATE_MAP_COMPLETE,
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
public:
  VehicleControlBase *vehicle_;
  Victim_Map_Base *Map_;
  SSD_Detection_with_clustering *Victim_detection_DL_;
  view_evaluator_IG *View_evaluate_;
  view_generator_IG *view_generate_;
  TestNBZ();
  NBVState::State state;
  bool is_done_map_update;
  int waypointNum;
  void runStateMachine();
  void initVehicle(void);
  void initMap(void);
  void initParameters();

  void generateViewpoints();
  void evaluateViewpoints();
  void UpdateMap();

  bool detection_enabled; //for debugging


};



#endif // TEST_FLIGHT_H




