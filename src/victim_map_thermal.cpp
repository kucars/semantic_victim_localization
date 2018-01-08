#include "victim_localization/victim_map_thermal.h"


victim_map_Thermal::victim_map_Thermal(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  Victim_Map_Base(nh,nh_private)
{
  ros::param::param<std::string>("~map_topic_thermal", map_topic , "victim_map/grid_map_thermal");
  ros::param::param<double>("~map_resol_thermal", map_resol , 0.2);

  tf_listener = new tf::TransformListener();
  detector_ = new victim_thermal_detector();

  setlayer_name(Thermal_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), map_resol); //(Map is 20 by 20 meter with a resolution of 0.2m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5); // initialize map probability to 0.5
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg/2));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  //Camera Settings
  ros::param::param<double>("~fov_horizontal_thermal_cam", HFOV_deg , 48);
  ros::param::param<double>("~fov_vertical_thermal_cam", VFOV_deg , 45);
  ros::param::param<double>("~thermal_range_max", max_thermal_d , 15);
  ros::param::param<double>("~thermal_range_max", min_thermal_d , 0.5);
  ros::param::param<double>("~octomap_resol", octomap_resol , 0.3);


  ros::param::param<double>("~thermal_image_x_resolution", thermal_img_x_res , 160.0);
  ros::param::param<double>("~thermal_image_y_resolution", thermal_img_y_res , 120.0);
  ros::param::param<double>("~thermal_image_x_offset", thermal_x_offset , 79.5);
  ros::param::param<double>("~thermal_image_x_offset", thermal_y_offset , 59.5);

  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_thermal", Prob_D_H , 0.6);
  ros::param::param<double>("~Prob_D_Hc_for_thermal", Prob_D_Hc , 0.3);
  ros::param::param<double>("~Prob_Dc_H_for_thermal", Prob_Dc_H , 0.4);
  ros::param::param<double>("~Prob_Dc_Hc_for_thermal", Prob_Dc_Hc , 0.7);

  // for debugging
  ros::param::param("~detection_enabled", detection_enabled, false);//for debugging
  if (map_resol> octomap_resol)
    raytracing_ = new Raytracing(map_resol,HFOV_deg,VFOV_deg,max_thermal_d,min_thermal_d);
  else
    raytracing_ = new Raytracing(octomap_resol,HFOV_deg,VFOV_deg,max_thermal_d,min_thermal_d);

  victimMapName="victim map thermal";
  max_depth_d=max_thermal_d;

  // visualize the thermal detection vector
  visualize_thermal_ray.reset(new rviz_visual_tools::RvizVisualTools("world", "/thermal_vector"));
  visualize_thermal_ray->loadMarkerPub();

  visualize_thermal_ray->deleteAllMarkers();
  visualize_thermal_ray->enableBatchPublishing();
}


void victim_map_Thermal::Update(){

  runDetector();

  grid_map::GridMap temp_Map;
  temp_Map=raytracing_->Project_3d_rayes_to_2D_plane(drone_comm->GetPose(),true);

  //polygon=Update_region(temp_Map,(raytracing_->current_pose_));

  if (is_detect_== true) {
  // Find Camera Cener Pose and Yaw in world Frame which needed for Thermal Ray Generation;
  geometry_msgs::PoseStamped CameraCentertoWorld ;
  GetCameraCenter2World(CameraCentertoWorld,detector_->getCaptureTime());
  Position CamCenterLoc(CameraCentertoWorld.pose.position.x,CameraCentertoWorld.pose.position.y);
  double CamYaw= pose_conversion::getYawFromQuaternion(CameraCentertoWorld.pose.orientation);
  double Half_HFOV_RAD = DEG2RAD(HFOV_deg/2);


  // Generate Ray from camera center through heat pixel
  ThermalRayStart[0]=CamCenterLoc[0];
  ThermalRayStart[1]=CamCenterLoc[1];

  if (detect_victim_loc_[0] >= thermal_x_offset){
    ThermalRayEnd[0]=CamCenterLoc[0] + max_thermal_d*cos(CamYaw+(((detect_victim_loc_[0]-thermal_x_offset)*Half_HFOV_RAD)/thermal_x_offset)) ;
    ThermalRayEnd[1]=CamCenterLoc[1] + max_thermal_d*sin(CamYaw+(((detect_victim_loc_[0]-thermal_x_offset)*Half_HFOV_RAD)/-thermal_x_offset)) ;
  }
  else {
    ThermalRayEnd[0]=CamCenterLoc[0] + max_thermal_d*cos(CamYaw+(((detect_victim_loc_[0]-thermal_x_offset)*-Half_HFOV_RAD)/(-thermal_x_offset))) ;
    ThermalRayEnd[1]=CamCenterLoc[1] + max_thermal_d*sin(CamYaw+(((detect_victim_loc_[0]-thermal_x_offset)*-Half_HFOV_RAD)/(thermal_x_offset))) ;
  }

  std::cout << "BTW... theraml start and end are:" << ThermalRayStart << " " << ThermalRayEnd << std::endl;
   GenerateRayVector(temp_Map,ThermalRayStart,ThermalRayEnd);

}
  map_status.victim_found=false; //initialize detection to false
  double Detec_prob=0;

  //The iteration should be done on the map of the smaller size

//  if(temp_Map.getResolution() < map.getResolution())
//  {
//    for (grid_map::GridMapIterator iterator(temp_Map);
//         !iterator.isPastEnd(); ++iterator) {
//      Position position;
//      Index index=*iterator;
//      temp_Map.getPosition(index, position);
//      if (!raytracing_->isInsideBounds(position)) continue;
//      float P_prior=map.atPosition(layer_name, position);
//      if(temp_Map.atPosition("temp", position)==0){ // if the cell is free ( contains no obsticales)
//        if (is_detect_==true)
//        {
//          if (IsInsideRay(position))
//          {
//            if (P_prior>0.01 )
//            {
//              Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
//              map.atPosition(layer_name, position)= Detec_prob;
//            }
//            else map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));
//          }
//          //not inside Ray
//          else map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
//        }
//        if (is_detect_== false )  map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));

//        if (Detec_prob>0.9) {
//          //std::cout << "Detec_prob" << std::endl;
//          map_status.victim_found=true;
//          map_status.victim_loc=position;
//        }
//      }
//    }
//  }
 // else{
    for (grid_map::GridMapIterator iterator(map);
         !iterator.isPastEnd(); ++iterator) {
      Position position;
      Index index=*iterator;
      map.getPosition(index, position);
      if (!temp_Map.isInside(position)) continue;
      if (!raytracing_->isInsideBounds(position)) continue;

      float P_prior=map.atPosition(layer_name, position);
      if(temp_Map.atPosition("temp", position)==0)
      { // if the cell is free ( contains no obstacle)
        if (is_detect_==true)
        {
          if (IsInsideRay(position))
          {
            if (P_prior>0.01 )
            {
              Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
              map.atPosition(layer_name, position)= Detec_prob;
            }
            else map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));
          }
          //not inside Ray
          else map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
        }
        if (is_detect_== false )  map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));

        if (Detec_prob>0.9) {
          //std::cout << "Detec_prob" << std::endl;
          map_status.victim_found=true;
          map_status.victim_loc=position;
        }
      }

      // if the cell is occupied not by obstacle but by the human
      else {
        if (is_detect_==true)
        {
          if (IsInsideRay(position))
          {
            if (P_prior>0.01 )
            {
              Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
              map.atPosition(layer_name, position)= Detec_prob;
            }
            else map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));
          }
        }
      }


    }
 // }
  publish_Map();
}


void victim_map_Thermal::GenerateRayVector
(grid_map::GridMap Occlusion_Map,Position start,Position End)
{
  ThermalRay.clear(); // initialize Thermal Ray
  visualize_thermal_ray->deleteAllMarkers();

//  if (Occlusion_Map.getResolution()< map.getResolution())
//  {
  for (grid_map::LineIterator iterator(map,start,End);!iterator.isPastEnd(); ++iterator)
  {
    Position position;
    Index index=*iterator;
    map.getPosition(index, position);
    if (!raytracing_->isInsideBounds(position)) continue;
    if(Occlusion_Map.atPosition("temp", position)==0) // if the cell is free ( contains no obsticales)
      ThermalRay.push_back(position);
  }
//}
//  else
//  {
//    for (grid_map::LineIterator iterator(Occlusion_Map,start,End);!iterator.isPastEnd(); ++iterator)
//    {
//      Position position;
//      Index index=*iterator;
//      Occlusion_Map.getPosition(index, position);
//      if (!raytracing_->isInsideBounds(position)) continue;
//      if(Occlusion_Map.atPosition("temp", position)==0) // if the cell is free ( contains no obsticales)
//        ThermalRay.push_back(position);
//    }
  //}

  if (ThermalRay.size()>2) {
    std::vector<geometry_msgs::Pose> vector_;
    geometry_msgs::Pose p_;

    for (int i=0; i<ThermalRay.size(); i++)
    {
      p_.position.x = ThermalRay[i][0];
      p_.position.y = ThermalRay[i][1];
      p_.position.z = 1;

      std::cout << "the point is " << ThermalRay[i][0] << ThermalRay[i][1] << std::endl;
      vector_.push_back(p_);
    }
    std::cout << "size the of thermal vector is" << vector_.size() << std::endl;

    visualize_thermal_ray->publishPath(vector_,rviz_visual_tools::RED, rviz_visual_tools::XLARGE,"theraml_");

    // exact line
    geometry_msgs::Pose p_s;
    geometry_msgs::Pose p_e;

    p_s.position.x= start[0];
    p_s.position.y= start[1];
    p_s.position.z= 0.0;

    p_e.position.x= End[0];
    p_e.position.y= End[1];
    p_e.position.z= 0.0;

    visualize_thermal_ray->publishLine(p_s.position,p_e.position,rviz_visual_tools::BLACK,rviz_visual_tools::XLARGE);
    visualize_thermal_ray->trigger();
  }
}
void victim_map_Thermal::GetCameraCenter2World
(geometry_msgs::PoseStamped &CamCentertoWorld, ros::Time caputre_time){

  geometry_msgs::PoseStamped CamCenter;
  CamCenter.header.frame_id = "thermal_cam_thermal_link";
  CamCenter.header.stamp = caputre_time;
  CamCenter.pose.position.x=0; CamCenter.pose.position.y=0; CamCenter.pose.position.z=0;
  CamCenter.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  while (true) {
    try
    {
      tf_listener->transformPose("/world", CamCenter, CamCentertoWorld);
      break; // Success, break out of the loop
    }
    catch (tf::TransformException &ex)
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }

  std::cout << "this is the  yaw after tansformation..."  << pose_conversion::getYawFromQuaternion(CamCentertoWorld.pose.orientation) << std::endl;
  std::cout << "this is the time after transformation..." << CamCentertoWorld.header.stamp << std::endl;
  std::cout << "time difference" << CamCentertoWorld.header.stamp.toSec() - caputre_time.toSec() << std::endl;
}


bool victim_map_Thermal::IsInsideRay(Position P){
  for (int i=0;i<ThermalRay.size();i++){
    if ((ThermalRay[i][0]==P[0]) && (ThermalRay[i][1]==P[1]))
      return true;
  }
  return false;
}

void victim_map_Thermal::runDetector()
{
  // added if statement for debugging
  if (detection_enabled){
    // run thermal detector
    detector_->SetCurrentSetpoint(current_loc_);
    detector_->performDetection();

    this->setDetectionResult(detector_->getDetectorStatus());
  }
  else {
    Status status_temp;
    status_temp.victim_found= false;
    Position g(0,0);
    status_temp.victim_loc=g;
    this->setDetectionResult(status_temp);
  }
}
