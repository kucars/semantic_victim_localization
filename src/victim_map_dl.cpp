#include "victim_localization/victim_map_dl.h"

victim_map_DL::victim_map_DL(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  Victim_Map_Base(nh,nh_private)
{
  Maptype=MAP::DL;
  setlayer_name(DL_layer_name);

  detector_ = new SSD_Detection_with_clustering();

  ros::param::param<std::string>("~map_topic_DL", map_topic , "victim_map/grid_map_DL");
  ros::param::param<double>("~map_resol_DL", map_resol , 1.0);

  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), map_resol); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5); // initialize probability in the map to 0.5
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg/2));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_DL", Prob_D_H , 0.9);
  ros::param::param<double>("~Prob_D_Hc_for_DL", Prob_D_Hc , 0.05);
  ros::param::param<double>("~Prob_Dc_H_for_DL", Prob_Dc_H , 0.1);
  ros::param::param<double>("~Prob_Dc_Hc_for_DL", Prob_Dc_Hc , 0.95);

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

  victimMapName="victim map DL";

  //pub_polygon=nh_.advertise<geometry_msgs::PolygonStamped>(DL_polygon_topic, 1, true);
}

void victim_map_DL::Update()
{
  //run deep learning detector
  runDetector();
  grid_map::GridMap temp_Map;

  bool rebuild=true;
  bool publish=true;
  raytracing_->Initiate(publish,rebuild);

  temp_Map=raytracing_->Generate_2D_Safe_Plane(current_loc_,true,true);

  map_status.victim_found=false; //initialize detection to false

  curr_max_prob=0; //initiate current max probabitity to zero

    for (grid_map::GridMapIterator iterator(map);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    map.getPosition(index, position);

    //check current max probabilty
    if (map.atPosition(layer_name, position)> curr_max_prob){
      curr_max_prob=map.atPosition(layer_name, position);
      curr_max_loc=position;
}
    if (!temp_Map.isInside(position)) continue;
    if (!raytracing_->isInsideBounds(position)) continue;

    float P_prior=map.atPosition(layer_name, position);

    // Update victim location is case it is recoganized obstacle from raytracing
    if (is_detect_==true && temp_Map.atPosition("temp", position)==1)
    {
      if ((index[0]== detect_victim_index[0]) && (index[1]== detect_victim_index[1]))
      {
        if (P_prior>0.01)
        {
          double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
          map.atPosition(layer_name, position)= Detec_prob;

          //check if the victim is found
          if (Detec_prob>victim_found_prob) {
            map_status.victim_found=true;
            map_status.victim_loc=position;
          }
        }
        else map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

//        //check max victim probablitiy
//          if (map.atPosition(layer_name, position)> curr_max_prob)
//            curr_max_prob=map.atPosition(layer_name, position);
      }
    }

    // if the cell is free ( contains no obstacle)
    if(temp_Map.atPosition("temp", position)==0)
    {
      if ((index[0]== detect_victim_index[0]) && (index[1]== detect_victim_index[1]))  {

        if (is_detect_== true && P_prior>0.01 ) {
          double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
          map.atPosition(layer_name, position)= Detec_prob;

          //check if the victim is found
          if (Detec_prob>victim_found_prob) {
            map_status.victim_found=true;
            map_status.victim_loc=position;
          }
        }

        if (is_detect_== true && P_prior<0.01 ) map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

        if (is_detect_== false )  map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
      }
      else  {map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior))); }


//    //check max victim probablitiy
//      if (map.atPosition(layer_name, position)> curr_max_prob)
//        curr_max_prob=map.atPosition(layer_name, position);
    }
}

//    std::cout << "current max prob..." << curr_max_prob << std::endl;
//    if (is_detect_){
//      std::cout << "max_prob" << curr_max_prob << std::endl;
//    }

  if (pub_map.getNumSubscribers()>0)
  publish_Map();
}


void victim_map_DL::runDetector()
{
  // added if statement for debugging
  if (detection_enabled){
   // run thermal detector
  detector_->performDetection();

    this->setDetectionResult(detector_->getDetectorStatus());
    this->setCurrentPose(detector_->capture_ps.pose);
  }
  else {
  Status status_temp;
  status_temp.victim_found= false;
  Position g(0,0);
  status_temp.victim_loc=g;
   this->setDetectionResult(status_temp);
}
}




