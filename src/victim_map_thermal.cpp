#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_map_thermal.h"

victim_map_Thermal::victim_map_Thermal():
  Victim_Map_Base(),
  thermal_vic_loc(&detect_loc_)
{
  tf_listener = new tf::TransformListener();

  setlayer_name(Thermal_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), 1); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5);
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(DL_map_topic, 1, true);

  //Camera Settings
  ros::param::param<double>("~fov_horizontal", HFOV_deg , 48);
  ros::param::param<double>("~fov_vertical", VFOV_deg , 45);
  ros::param::param<double>("~depth_range_max", max_thermal_d , 15);
  ros::param::param<double>("~thermal_image_x_resolution", thermal_img_x_res , 160.0);
  ros::param::param<double>("~thermal_image_y_resolution", thermal_img_y_res , 120.0);
  ros::param::param<double>("~thermal_image_x_offset", thermal_x_offset , 79.5);
  ros::param::param<double>("~thermal_image_x_offset", thermal_y_offset , 59.5);

  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_DL", Prob_D_H , 0.6);
  ros::param::param<double>("~Prob_D_Hc_for_DL", Prob_D_Hc , 0.3);
  ros::param::param<double>("~Prob_Dc_H_for_DL", Prob_Dc_H , 0.4);
  ros::param::param<double>("~Prob_Dc_Hc_for_DL", Prob_Dc_Hc , 0.7);
}


void victim_map_Thermal::Update(){

  grid_map::GridMap temp_Map;
  temp_Map=raytracing_->Project_3d_rayes_to_2D_plane(raytracing_->current_pose_);

  polygon=Update_region(temp_Map,(raytracing_->current_pose_));

  // Find Camera Cener Pose and Yaw in world Frame which needed for Thermal Ray Generation;
  geometry_msgs::PoseStamped CameraCentertoWorld ;
  GetCameraCenter2World(CameraCentertoWorld);
  Position CamCenterLoc(CameraCentertoWorld.pose.position.x,CameraCentertoWorld.pose.position.y);
  double CamYaw= pose_conversion::getYawFromQuaternion(CameraCentertoWorld.pose.orientation);
  double Half_HFOV_RAD = DEG2RAD(HFOV_deg/2);


  // Generate Ray from camera center to heat pixel
  ThermalRayStart[0]=CamCenterLoc[0];
  ThermalRayStart[1]=CamCenterLoc[1];

  if (thermal_vic_loc[0] >= thermal_x_offset){
    ThermalRayEnd[0]=CamCenterLoc[0] + max_thermal_d*cos(CamYaw+(((thermal_vic_loc[0]-thermal_x_offset)*Half_HFOV_RAD)/thermal_x_offset)) ;
    ThermalRayEnd[1]=CamCenterLoc[1] + max_thermal_d*sin(CamYaw+(((thermal_vic_loc[0]-thermal_x_offset)*Half_HFOV_RAD)/-thermal_x_offset)) ;
   }
    else {
    ThermalRayEnd[0]=CamCenterLoc[0] + max_thermal_d*cos(CamYaw+(((thermal_vic_loc[0]-thermal_x_offset)*-Half_HFOV_RAD)/(-thermal_x_offset))) ;
    ThermalRayEnd[1]=CamCenterLoc[1] + max_thermal_d*sin(CamYaw+(((thermal_vic_loc[0]-thermal_x_offset)*-Half_HFOV_RAD)/(thermal_x_offset))) ;
   }

  if (is_detect_== true)  GenerateRayVector(temp_Map,ThermalRayStart,ThermalRayEnd);

 status.victim_found=false; //initialize detection to false

 for (grid_map::GridMapIterator iterator(temp_Map);
                     !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    temp_Map.getPosition(index, position);
   if (!raytracing_->isInsideBounds(position)) continue;
    float P_prior=map.atPosition(layer_name, position);
    if(temp_Map.atPosition("temp", position)==0){ // if the cell is free ( contains no obsticales)
      if (is_detect_==true)
      {
      if (IsInsideRay(position))
      {
        if (P_prior>0.01 )
        {
          double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
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
      status.victim_found=true;
      status.victim_loc=position;
      }
  }
    }
  publish_Map();
}


void victim_map_Thermal::GenerateRayVector
                                (grid_map::GridMap Map,Position start,Position End)
{
  ThermalRay.clear(); // initialize Thermal Ray

  for (grid_map::LineIterator iterator(Map,start,End);!iterator.isPastEnd(); ++iterator)
  {
    Position position;
    Index index=*iterator;
    Map.getPosition(index, position);
    if (!raytracing_->isInsideBounds(position)) continue;
    if(temp_Map.atPosition("temp", position)==0) // if the cell is free ( contains no obsticales)
      ThermalRay.push_back(position);
  }
}

void victim_map_Thermal::GetCameraCenter2World
                          (geometry_msgs::PoseStamped &CamCentertoWorld){

  geometry_msgs::PoseStamped CamCenter;
  geometry_msgs::PoseStamped CamCentertoWorld;

  CamCenter.header.frame_id = "front_cam_depth_frame";
  CamCenter.header.stamp = ros::Time::now();
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
}


bool victim_map_Thermal::IsInsideRay(Position P){
  for (i=0;i<ThermalRay.size();i++){
    if ((ThermalRay[i][0]==P[0]) && (ThermalRay[i][1]==P[1]))
      return true;
  }
  return false;
}

