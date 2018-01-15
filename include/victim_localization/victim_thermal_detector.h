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

#include <victim_localization/victim_detector_base.h>
#include <string.h>

class victim_thermal_detector : public victim_detector_base
{
public:
  victim_thermal_detector();
  ~victim_thermal_detector();

  image_transport::Subscriber  sub_image;
  image_transport::Publisher pub_detection_;

  double minTempVictim_;
  double maxTempVictim_;
  double minAreaVictim_;
  double minDistBetweenBlobs_;
  double blob_temperature_;
  sensor_msgs::ImageConstPtr input_image;
  cv::Mat img_proc;
  Position victim_loc;
  bool victim_found;
  bool plot_;


  void imageCallback(const sensor_msgs::ImageConstPtr& img,
                                              const geometry_msgs::PoseStamped::ConstPtr& loc);
  void performDetection();

  void BlobDetection();
  Status getDetectorStatus();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,geometry_msgs::PoseStamped> MySyncPolicy;
  message_filters::Subscriber<sensor_msgs::Image> *thermal_image_sub;
  message_filters::Subscriber<geometry_msgs::PoseStamped> *loc_sub_;
  message_filters::Synchronizer<MySyncPolicy> *sync;
  std::string thermal_image_topic;
  std::string topic_Pose;


  ros::Time getCaptureTime();
};

#endif // VICTIM_THERMAL_DETECTOR_H
