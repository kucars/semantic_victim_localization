#include "victim_localization/victim_map_combined.h"

victim_map_combined::victim_map_combined(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  Victim_Map_Base(nh,nh_private)
{ 
  victim_map_dl_=new victim_map_DL(nh_,nh_private_);
  victim_map_Thermal_= new victim_map_Thermal(nh_,nh_private_);
  victim_map_wireless_= new Victim_Map_Wireless(nh_,nh_private_);

  double max_d,min_d;

  ros::param::param<double>("~max_d", max_d , 10.0);
  ros::param::param<double>("~min_d", min_d , 0.5);

  ros::param::param<std::string>("~map_topic_combined", map_topic , "victim_map/grid_map_combined");
  ros::param::param<double>("~map_resol_combined", map_resol , 0.2);

  setlayer_name(combinedMap_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), map_resol); //(Map is 20 by 20 meter with a resolution of 0.2m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5);
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  //Values for probability
  ros::param::param<double>("~alpha", alpha , 0.6);
  ros::param::param<double>("~beta", beta , 0.25);
  ros::param::param<double>("~gama", gama , 0.15);

  switch(raytracing_type) {
  case 0:
    if (map_resol> octomap_resol){
      raytracing_ = new Raytracing(map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  case 1:
    if (map_resol> octomap_resol){
      raytracing_ = new Raytracing2D(map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing2D(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  }

  victimMapName="victim fused map";
}

void victim_map_combined::Update()
{
//  victim_map_Thermal_->Update();
//  std::cout << " Done updating thermal map\n";
//  victim_map_dl_->Update();
//  std::cout << " Done updating dl map\n";
//  victim_map_wireless_->Update();
//  std::cout << " Done updating wireless map\n";


//  map_status.victim_found=false; //initialize detection to false
//  curr_max_prob=0; //initiate current max probabitity to zero


//  // update map
//  Position position;
//  Index index;

//  //ros::Time time_1= ros::Time::now();

//  for (grid_map::GridMapIterator iterator(map);
//       !iterator.isPastEnd(); ++iterator) {

//    index=*iterator;
//    map.getPosition(index, position);

//    map.atPosition(layer_name,position)= alpha*victim_map_dl_->map.atPosition(victim_map_dl_->getlayer_name(),position)
//        + beta*victim_map_Thermal_->map.atPosition(victim_map_Thermal_->getlayer_name(),position)
//        + gama*victim_map_wireless_->map.atPosition(victim_map_wireless_->getlayer_name(),position);

//    if (map.atPosition(layer_name,position) >= victim_found_prob ) {
//      std::cout << "yes victim found...." << std::endl;
//      map_status.victim_found=true;
//      map.getPosition(index,map_status.victim_loc);
//    }
//  }
//  curr_max_prob=alpha*victim_map_dl_->curr_max_prob + beta*victim_map_Thermal_->curr_max_prob
//      + gama*victim_map_wireless_->curr_max_prob;

//  std::cout<< "current max prob is..." << curr_max_prob << std::endl;

//  //ros::Duration elapsed_time= ros::Time::now()-time_1;
//  //ROS_INFO("Time taken to update is the combined map is: %f", elapsed_time.toSec());


//  //      if (data(index(0), index(1)) > 0.9 ) {
//  //        map_status.victim_found=true;
//  //        map.getPosition(index,map_status.victim_loc);
//  //        break;
//  //      }

//  //  grid_map::Matrix& data = map[this->getlayer_name()];
//  //  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
//  //      const Index index(*iterator);
//  //      if (data(index(0), index(1)) > 0.9 ) {
//  //        map_status.victim_found=true;
//  //        map.getPosition(index,map_status.victim_loc);
//  //        break;
//  //      }
//  //  }

  //debugging.....

 victim_map_dl_->Update();
 std::cout << " Done updating dl map\n";

   // update map
   Position position;
   Index index;

   for (grid_map::GridMapIterator iterator(map);
        !iterator.isPastEnd(); ++iterator) {

     index=*iterator;
     map.getPosition(index, position);

     map.atPosition(layer_name,position)= victim_map_dl_->map.atPosition(victim_map_dl_->getlayer_name(),position);

  publish_Map();
  std::cout << " Done updating Combined map\n";

  //ros::Rate(0.0001).sleep();
}
}
void victim_map_combined::setDroneCommunicator(drone_communicator *drone_comm_)
{
  Victim_Map_Base::setDroneCommunicator(drone_comm_);
  victim_map_dl_->setDroneCommunicator(drone_comm_);
  victim_map_Thermal_->setDroneCommunicator(drone_comm_);
  victim_map_wireless_->setDroneCommunicator(drone_comm_);
  std::cout << "combined_drone_comm is set in Combined Map" << std::endl;
}

void victim_map_combined::SetNavMap(nav_msgs::OccupancyGridPtr Nav_map)
{
  Victim_Map_Base::SetNavMap(Nav_map);

  victim_map_dl_->raytracing_->SetNavMap(Nav_map);
  victim_map_Thermal_->raytracing_->SetNavMap(Nav_map);
  victim_map_wireless_->raytracing_->SetNavMap(Nav_map);

 std::cout << "Nav_Map is set in Combined Map" << std::endl;
}
