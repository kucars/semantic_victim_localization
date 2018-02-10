#ifndef TERMINATION_CHECK_BASE_H
#define TERMINATION_CHECK_BASE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "victim_localization/common.h"
#include "victim_localization/nbv_history.h"
#include "victim_localization/view_evaluator_base.h"
#include "grid_map_cv/grid_map_cv.hpp"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace std;
using namespace grid_map;

class TerminationCheckBase
{
public:
  TerminationCheckBase();
  virtual bool isTerminated();
  void setHistory(nbv_history* h);
  void setViewEvaluator(view_evaluator_base* v);
  void SaveResults();
  cv::Mat upscaleOccupancyImage(grid_map::GridMap inputMap , std::string Input,
                       grid_map::GridMap upscaledMap, std::string Output);

  void setNBVTimeTaken(ros::Duration t);
  ros::Duration NBVDurationTaken;
  std::string saveFolder;

protected:
  nbv_history* nbv_history_;
  view_evaluator_base* view_evaluator;

  int repeat_window_size_;
};

#endif // TERMINATION_CHECK_BASE_H
