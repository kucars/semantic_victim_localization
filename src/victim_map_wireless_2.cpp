#include "victim_localization/victim_map_wireless_2.h"

Victim_Map_Wireless::Victim_Map_Wireless(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  Victim_Map_Base(nh,nh_private)
{
  Maptype=MAP::WIRELESS;
  setlayer_name(wireless_layer_name);

  ros::param::param<std::string>("~map_topic_wireless", map_topic , "victim_map/grid_map_wireless");
  ros::param::param<double>("~map_resol_wireless", map_resol , 0.5);
  ros::param::param<double>("~victim_location_x", victim_loc[0] , -4.331);
  ros::param::param<double>("~victim_location_y", victim_loc[1] , -1.427);
  ros::param::param<double>("~distance_threshold", max_wireless_range , 7.0);

  ros::param::param<double>("~map_resol_DL", vision_map_resol , 1.0);

  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), map_resol); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5); // initialize probability in the map to 0.5

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_wireless", Prob_D_H , 0.8);
  ros::param::param<double>("~Prob_D_Hc_for_wireless", Prob_D_Hc , 0.15);
  ros::param::param<double>("~Prob_Dc_H_for_wireless", Prob_Dc_H , 0.2);
  ros::param::param<double>("~Prob_Dc_Hc_for_wireless", Prob_Dc_Hc , 0.85);

  victimMapName="victim map Wireless";

  detector_ = new victim_wireless_detector();

  switch(raytracing_type) {
  case 0:
    if (vision_map_resol> octomap_resol){
      raytracing_ = new Raytracing(vision_map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  case 1:
    if (map_resol> octomap_resol){
      raytracing_ = new Raytracing2D(vision_map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing2D(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  }
}


void Victim_Map_Wireless::Update()
{
  detector_->SetVehicleCommunicator(drone_comm);
  runDetector();

  map_status.victim_found=false; //initialize detection to false
  curr_max_prob=0; //initiate current max probabitity to zero
  Position center(drone_comm->GetPose().position.x,drone_comm->GetPose().position.y);

  if (is_detect_==true){
    double radius = detect_victim_loc_[0];
    for (grid_map::CircleIterator iterator(map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Position position;
      Index index=*iterator;
      map.getPosition(index, position);
      float P_prior=map.at(layer_name, *iterator);

       // if ((fabs(Distance_to_center(position,center)-radius))<=map_resol)
       // {
          double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
          map.atPosition(layer_name, position)= Detec_prob;

          //check if the victim is found
          if (Detec_prob>victim_found_prob) {
            map_status.victim_found=true;
            map_status.victim_loc=position;
          }
       // }

        //else
        //  map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
    }
  }

    else{
    double radius = max_wireless_range;
    for (grid_map::CircleIterator iterator(map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Position position;
      Index index=*iterator;
      map.getPosition(index, position);
      float P_prior=map.at(layer_name, *iterator);
      map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
    }
  }


  //fastest way to acces grid map
  grid_map::Matrix& data = map[layer_name];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    if(data(index(0), index(1))> curr_max_prob)
      curr_max_prob=data(index(0), index(1));
  }

  if (pub_map.getNumSubscribers()>0)
  publish_Map();
}


void Victim_Map_Wireless::runDetector()
{
  // added if statement for debugging
  if (detection_enabled){
    // run wireless detector
    detector_->performDetection();

    this->setDetectionResult(detector_->getDetectorStatus());
  }
  else {
    Status status_temp;
    status_temp.victim_found= false;
    Position g(0,0);
    status_temp.victim_loc=g;
    this->setDetectionResult(status_temp);
    this->setCurrentPose(detector_->capture_ps.pose);
  }
}

void Victim_Map_Wireless::setDetectionResult(Status detection_status) {
  detect_victim_loc_=detection_status.victim_loc;
  is_detect_=detection_status.victim_found;
}

double Victim_Map_Wireless::Distance_to_center(Position p1, Position p2)
{
  return sqrt(((p1[0]-p2[0])*(p1[0]-p2[0]))+((p1[1]-p2[1])*(p1[1]-p2[1])));
}



