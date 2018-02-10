#ifndef VICTIM_DETECTOR_BASE_H
#define VICTIM_DETECTOR_BASE_H

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <victim_localization/victim_map_base.h>

class victim_detector_base
{
public:
  victim_detector_base();
  ~victim_detector_base();

  vehicle_communicator *vehicle_comm_;
  virtual Status getDetectorStatus(){};
  virtual void performDetection(){};

  geometry_msgs::PoseStamped capture_ps;
  geometry_msgs::PoseStamped current_ps;
  void SetVehicleCommunicator(vehicle_communicator *vehicle_comm);
  virtual ros::Time getCaptureTime(){};
  virtual ros::Duration GetConnectionTime(); // GetConnectionTime only use with
                                                               //  DeepLeanring Detector to account for possible connection drop
};

#endif // VICTIM_DETECTOR_BASE_H
