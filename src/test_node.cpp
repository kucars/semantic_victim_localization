#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include "math.h"
#include <iostream>
#include <string>
#include <vector>

using namespace grid_map;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  grid_map::GridMap map;
  map.setFrameId("map");
  map.setGeometry(Length(80,80), 0.1); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add("layer_name",0.5); // initialize probability in the map to 0.5

  ros::Time p_time=ros::Time::now();
  double curr_max_prob=0;

  for (grid_map::GridMapIterator iterator(map);
     !iterator.isPastEnd(); ++iterator) {
  Position position;
  Index index=*iterator;
  map.getPosition(index, position);

  //check current max probabilty
  if (map.atPosition("layer_name", position)> curr_max_prob){
    curr_max_prob=map.atPosition("layer_name", position);
}

  }
  ROS_INFO("Loop Based on Position"
           "execution time: %f",ros::Time::now().toSec()- p_time.toSec());



  p_time=ros::Time::now();
  curr_max_prob=0;
  for (grid_map::GridMapIterator iterator(map);
     !iterator.isPastEnd(); ++iterator) {
  Index index=*iterator;

  //check current max probabilty
  if (map.at("layer_name", index)> curr_max_prob){
    curr_max_prob=map.at("layer_name", index);
}

  }
  ROS_INFO("Loop Based on index"
           "execution time: %f",ros::Time::now().toSec()- p_time.toSec());



  p_time=ros::Time::now();
  curr_max_prob=0;

    grid_map::Matrix& data = map["layer_name"];
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const Index index(*iterator);
        if (data(index(0), index(1)) > curr_max_prob ) {
          curr_max_prob=data(index(0), index(1));
          break;
        }
    }

  ROS_INFO("Loop Based on FastMode"
           "execution time: %f",ros::Time::now().toSec()- p_time.toSec());

}
