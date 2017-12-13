#include <iostream>

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "victim_localization/view_evaluator_ig.h"
#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>

view_evaluator_IG::view_evaluator_IG():
  info_selected_utility_(-std::numeric_limits<float>::infinity()) //-inf
{
  ros::param::param<double>("~fov_horizontal", HFOV_deg , 58);
  ros::param::param<double>("~fov_vertical", VFOV_deg , 45);
  ros::param::param<double>("~depth_range_min", max_depth_d , 5);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);

  camera_frame="/front_cam_depth_optical_frame";
  base_frame="/base_link";

  const_=max_depth_d/(2*cos(DEG2RAD(HFOV_deg)));

  ros::NodeHandle nh_private_("~");
}

double view_evaluator_IG::getCellEntropy(Position cell_)
{
 // std::cout << "possible_location4" << std::endl;

  double p= mapping_module_->map.atPosition(MapLayer,cell_);
  return - p*log(p) - (1-p)*log(1-p);
}

void view_evaluator_IG::setViewGenerator(view_generator_IG* v)
{
  view_gen_ = v;
}

void view_evaluator_IG::setMappingModule(Victim_Map_Base* m)
{
  mapping_module_ = m;
}

void view_evaluator_IG::update_parameters()
{
  MapLayer = mapping_module_->getlayer_name();
  current_pose_ = view_gen_->current_pose_;
  tree_ = view_gen_->manager_->octree_.get();

  current_yaw_=pose_conversion::getYawFromQuaternion(current_pose_.orientation);

  info_entropy_total_=0;

    for (grid_map::GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator)
     info_entropy_total_+=mapping_module_->map.at(mapping_module_->layer_name,*iterator);

}

double view_evaluator_IG::calculateIG(geometry_msgs::Pose p){      //TOFIX project FOV from camera center not drone center
 // std::cout << "tested Pose: " << p <<std::endl;
  grid_map::GridMap temp_Map;
  grid_map::Polygon polygon_view;
  temp_Map=mapping_module_->raytracing_->Project_3d_rayes_to_2D_plane(p);

  polygon_view=mapping_module_->Update_region(temp_Map,p);  //std::cout << p1 << " " << p2 << std::endl;
  int count=0;
  double IG_view=0;
  for (grid_map::GridMapIterator iterator(temp_Map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    temp_Map.getPosition(index, position);
    //std::cout << "possible_location5" << std::endl;
    if(temp_Map.atPosition("temp", position)==0){
      IG_view+=getCellEntropy(position);
    }
    //std::cout << "yes" << std::endl;
   // std::cout << "loc: " << position << "  value: "<< temp_Map.atPosition("temp", position) << " count: " << count << std::endl;
     //std::cout << "loc: " << position << " count: " << count << std::endl;
    count++;
  }
  return IG_view;
}

void view_evaluator_IG::evaluate(){      //TOFIX project FOV from camera center not drone center

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
   selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateIG(p);
        if (utility > info_selected_utility_)
        {
         info_selected_utility_ = utility;
          selected_pose_ = p;
        }
    }

       // No valid poses found, end
       if ( std::isnan(selected_pose_.position.x) )
       {
         return;
       }

      view_gen_->visualize(view_gen_->generated_poses, view_gen_->rejected_poses,selected_pose_);
}

geometry_msgs::Pose view_evaluator_IG::getTargetPose()
{
  return selected_pose_;
}









