#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_map_dl.h"

victim_map_DL::victim_map_DL():
  Victim_Map_Base()
{
  setlayer_name(DL_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), 1); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5);
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(DL_map_topic, 1, true);

  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_DL", Prob_D_H , 0.9);
  ros::param::param<double>("~Prob_D_Hc_for_DL", Prob_D_Hc , 0.05);
  ros::param::param<double>("~Prob_Dc_H_for_DL", Prob_Dc_H , 0.1);
  ros::param::param<double>("~Prob_Dc_Hc_for_DL", Prob_Dc_Hc , 0.95);

  //pub_polygon=nh_.advertise<geometry_msgs::PolygonStamped>(DL_polygon_topic, 1, true);
}

void victim_map_DL::Update(){

  grid_map::GridMap temp_Map;
  temp_Map=raytracing_->Project_3d_rayes_to_2D_plane(raytracing_->current_pose_);

  polygon=Update_region(temp_Map,(raytracing_->current_pose_));

  Position D_loc;
  D_loc[0]=detect_loc_[0];
  D_loc[1]=detect_loc_[0];
  D_loc=approximate_detect(D_loc);

  status.victim_found=false; //initialize detection to false

  for (grid_map::GridMapIterator iterator(temp_Map);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    temp_Map.getPosition(index, position);
   if (!raytracing_->isInsideBounds(position)) continue;

    float P_prior=map.atPosition(layer_name, position);
    if(temp_Map.atPosition("temp", position)==0){ // if the cell is free ( contains no obsticales)
      if (position[0]== D_loc[0] && position[1]== D_loc[1])  {

        if (is_detect_== true && P_prior>0.01 ) {
          double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
          map.atPosition(layer_name, position)= Detec_prob;
          std::cout << cc.blue << "current_detection_probablitiy:" << Detec_prob << std::endl;

          if (Detec_prob>0.9) {
            //std::cout << "Detec_prob" << std::endl;
            status.victim_found=true;
            status.victim_loc=position;
          }
        }

        if (is_detect_== true && P_prior<0.01 ) map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

        if (is_detect_== false )  map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
      }
      else  {map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior))); }
    }
  }
  publish_Map();
}






