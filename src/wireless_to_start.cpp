#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <iostream>
#include <vector>


bool y= false;
int i=0;
double time_=0;
using namespace grid_map;

ros::Time last_pos_time = ros::Time(0.0);
ros::Duration update_period = ros::Duration(1);


int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "gaussian_demo");

  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(90, 60), 0.1);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
   map.clearAll();
  
  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  std::vector <float> x;
  float start;
  //if (i==0)  start = ros::Time::now().toSec();
  while (nh.ok()) {
 
    // Add data to grid map.
    if ((ros::Time::now() - last_pos_time) <update_period)  continue;
  
  last_pos_time =ros::Time::now();
  //ros::Time time = ros::Time::now();
    //float timed = (ros::Time::now().toSec());
float sigma_x= 0.398;
float sigma_y= 0.398;
    float mean_x=-2;
    float mean_y=1;
    //float sigma_x= 0.398942280 *((ros::Time::now().toSec()-start+100)/(100));
    //float sigma_y= 0.398942280 *((ros::Time::now().toSec()-start+100)/(100));
    sigma_x= sigma_x ;
    sigma_y= sigma_y ;
    Position center (0,0);
    double radius = sigma_x*20; 
    
  for (CircleIterator it(map, center, radius);
      !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = (1/(2*M_PI*sigma_x*sigma_y))*exp(-((pow(position.x()-mean_x,2)/(2*pow(sigma_x,2)))+(pow(position.y()-mean_y,2)/(2*pow(sigma_y,2)))));
      x.push_back(map.at("elevation", *it));
	//std::cout <<  (*it).transpose()  << std::endl;
    }

/*
   for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = map.at("elevation", *it) / *max_element(x.begin(), x.end());

}
*/
      /*Position position;
      map.getPosition( *max_element(x.begin(), x.end()) , position);
     std::cout << *max_element(x.begin(), x.end()) << " at location: " << position << std::endl;
    // std::cout << ros::Time::now().toSec() << std::endl;
     */

 sigma_x= 0.398;
 sigma_y= 0.398;
     mean_x=2.5;
     mean_y=4;
    //float sigma_x= 0.398942280 *((ros::Time::now().toSec()-start+100)/(100));
    //float sigma_y= 0.398942280 *((ros::Time::now().toSec()-start+100)/(100));
    sigma_x= sigma_x ;
    sigma_y= sigma_y;
    center (0,0);	
    radius = sigma_x*20; 
    
  for (CircleIterator it(map, center, radius);
      !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = map.at("elevation", *it) + (1/(2*M_PI*sigma_x*sigma_y))*exp(-((pow(position.x()-mean_x,2)/(2*pow(sigma_x,2)))+(pow(position.y()-mean_y,2)/(2*pow(sigma_y,2)))));
      x.push_back(map.at("elevation", *it));
	//std::cout <<  (*it).transpose()  << std::endl;
    }

    time_=time_+1;
    printf("elaspsed time: %lf s, max_probablity: %lf \n", time_, *max_element(x.begin(), x.end()));
    x.clear();
    // Publish grid map.
    map.setTimestamp(last_pos_time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

  map["elevation"].setConstant(0.0);
    
    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
