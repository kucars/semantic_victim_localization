#ifndef SSD_CLIENT_H
#define SSD_CLIENT_H


#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <string>
#include <sstream>
#include "victim_localization/DL_box.h"

class SSD_client {
  protected:
  ros::NodeHandle n;
  ros::Rate loop_rate;
  ros::ServiceClient client ;

  public:
  bool Detection_success;
  victim_localization::DL_box srv;
  SSD_client::SSD_client();
  void Get_SSD_Detection();

};

#endif // SSD_CLIENT_H
