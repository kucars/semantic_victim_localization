#ifndef VICTIM_THERMAL_DETECTOR_H
#define VICTIM_THERMAL_DETECTOR_H

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class victim_thermal_detector
{
public:
  victim_thermal_detector(ros::NodeHandle& nh_,ros::NodeHandle& pnh_);
  ~victim_thermal_detector();

  image_transport::Subscriber  sub_image;
  image_transport::Publisher pub_detection_;

  double minTempVictim_;
  double maxTempVictim_;
  double minAreaVictim_;
  double minDistBetweenBlobs_;
  double blob_temperature_;
  std::vector<geometry_msgs::Point> victim_loc;

  void imageCallback(const sensor_msgs::ImageConstPtr& img);
  std::vector<geometry_msgs::Point> GetDetectionResult();




};

#endif // VICTIM_THERMAL_DETECTOR_H
