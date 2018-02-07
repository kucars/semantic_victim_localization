#ifndef VICTIM_WIRELESS_DETECTOR_H
#define VICTIM_WIRELESS_DETECTOR_H

#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <victim_localization/victim_detector_base.h>

// matlab generated include files
#include "RSSmodel2.h"
#include "rt_nonfinite.h"
#include "RSSmodel2_terminate.h"
#include "RSSmodel2_emxAPI.h"
#include "RSSmodel2_initialize.h"
#include "RSSmodel2_types.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"



class victim_wireless_detector : public victim_detector_base
{
public:
  victim_wireless_detector();
  ~victim_wireless_detector();


  double current_node_loc;
  double target_node_loc;
  double d0;
  double N_sensor;
  double lamda;
  double frequency;
  double SNR;
  double K;
  double path_loss;
  double exact_dist;
  double distance_to_victim;
  double distance_threshold;
  double current_loc[2];
  double target_loc[2];


  void setCurrentPose();


  void performDetection();

  void EstimateDist();
  Status getDetectorStatus();

};

#endif // VICTIM_WIRELESS_DETECTOR_H
