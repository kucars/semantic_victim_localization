#include "victim_localization/victim_map_combined.h"

victim_map_combined::victim_map_combined():
  Victim_Map_Base()
{ 
  victim_map_dl_=new victim_map_DL();
  victim_map_Thermal_= new victim_map_Thermal();

  ros::param::param<std::string>("~map_topic_combined", map_topic , "victim_map/grid_map_combined");
  ros::param::param<double>("~map_resol_combined", map_resol , 0.2);

  setlayer_name(combinedMap_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), 1); //(Map is 20 by 20 meter with a resolution of 0.2m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5);
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  //Values for probability
  ros::param::param<double>("~alpha", alpha , 0.7);
  ros::param::param<double>("~beta", beta , 0.3);
  //ros::param::param<double>("~gamma", gamma , 0.4);

  victimMapName="victim fused map";
}

void victim_map_combined::Update()
{
  victim_map_dl_->Update();
  victim_map_Thermal_->Update();

  // update map
  map[this->getlayer_name()]= alpha*victim_map_dl_->map[victim_map_dl_->getlayer_name()] +
                              beta*victim_map_Thermal_->map[victim_map_Thermal_->getlayer_name()];

  map_status.victim_found=false; //initialize detection to false

  grid_map::Matrix& data = map[this->getlayer_name()];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      if (data(index(0), index(1)) > 0.9 ) {
        map_status.victim_found=true;
        map.getPosition(index,map_status.victim_loc);
        break;
      }
  }

  publish_Map();
}
