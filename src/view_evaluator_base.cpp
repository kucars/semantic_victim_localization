#include "victim_localization/view_evaluator_base.h"


view_evaluator_base::view_evaluator_base():
  info_selected_utility_(-std::numeric_limits<float>::infinity()), //-inf
  info_distance_total_(0),
  info_dl_selected_utility_(0),
  info_thermal_selected_utility_(0),
  info_wireless_selected_utility_(0)
{
  ros::param::param<double>("~fov_horizontal", HFOV_deg , 58);
  ros::param::param<double>("~fov_vertical", VFOV_deg , 45);
  ros::param::param<double>("~depth_range_min", max_depth_d , 5);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);
  ros::param::param<double>("~distance_threshold", wireless_max_range , 20);
  ros::param::param<double>("~alpha", wireless_max_range , 20);
  ros::param::param<double>("~distance_threshold", wireless_max_range , 20);
  ros::param::param<double>("~alpha", alpha , 0.65);
  ros::param::param<double>("~beta", beta , 0.2);
  ros::param::param<double>("~gama", gama , 0.15);

  ros::param::param<std::string>("~camera_optical_frame", camera_optical_frame ,std::string("/floating_sensor/camera_depth_optical_frame"));
  ros::param::param<std::string>("~base_frame", base_frame ,std::string("/floating_sensor/base_link"));

  const_=max_depth_d/(2*cos(DEG2RAD(HFOV_deg)));

  ros::NodeHandle nh_private_("~");
}

double view_evaluator_base::getCellEntropy(Position cell_, Victim_Map_Base *mapping_module)
{
  double p= mapping_module->map.atPosition(mapping_module->getlayer_name(),cell_);
  return - p*log(p) - (1-p)*log(1-p);
}

void view_evaluator_base::setViewGenerator(view_generator_IG* v)
{
  view_gen_ = v;
}

void view_evaluator_base::setMappingModule(Victim_Map_Base* m)
{
  mapping_module_ = m;
}

void view_evaluator_base::update_parameters()
{
  current_pose_ = view_gen_->current_pose_;
  tree_ = view_gen_->manager_->octree_;

  current_yaw_=pose_conversion::getYawFromQuaternion(current_pose_.orientation);

  info_entropy_total_=0;
  info_dl_entropy_total_=0;
  info_thermal_entropy_total_=0;
  info_wireless_entropy_total_=0;

  // calculate entropy total for the combined or a specific type of map
  grid_map::Matrix& data = mapping_module_->map[mapping_module_->layer_name];
  for (GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      info_entropy_total_+=data(index(0), index(1));
  }

  std::cout << "totla MAP info is:" << info_entropy_total_ << std::endl;
  std::cout << "Original MAP resolution is:" << mapping_module_->map.getResolution() << std::endl;

  if (mapping_module_->Maptype!=MAP::COMBINED)
    return;

  // calculate vision map entropy total
  grid_map::Matrix& data_DL = mapping_module_->getMapLayer(MAP::DL)->map[mapping_module_->getMapLayer(MAP::DL)->layer_name];
  for (GridMapIterator iterator(mapping_module_->getMapLayer(MAP::DL)->map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      info_dl_entropy_total_+=data_DL(index(0), index(1));
  }
  std::cout << "totla dl info is:" << info_dl_entropy_total_ << std::endl;
  std::cout << "DL resolution is:" << mapping_module_->getMapLayer(MAP::DL)->map.getResolution() << std::endl;
  std::cout << "DL Name:" << mapping_module_->getMapLayer(MAP::DL)->getlayer_name()<< std::endl;

  // calculate thermal map entropy total
  grid_map::Matrix& data_thermal = mapping_module_->getMapLayer(MAP::THERMAL)->map[mapping_module_->getMapLayer(MAP::THERMAL)->layer_name];
  for (GridMapIterator iterator(mapping_module_->getMapLayer(MAP::THERMAL)->map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      info_thermal_entropy_total_+=data_thermal(index(0), index(1));
  }

  std::cout << "totla thermal info is:" << info_thermal_entropy_total_ << std::endl;
  std::cout << "thremal resolution is:" << mapping_module_->getMapLayer(MAP::THERMAL)->map.getResolution() << std::endl;
  std::cout << "thremal map layer name is:" << mapping_module_->getMapLayer(MAP::THERMAL)->layer_name << std::endl;

  // calculate wireless map entropy total
  grid_map::Matrix& data_combined = mapping_module_->getMapLayer(MAP::WIRELESS)->map[mapping_module_->getMapLayer(MAP::WIRELESS)->layer_name];
  for (GridMapIterator iterator(mapping_module_->getMapLayer(MAP::WIRELESS)->map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      info_wireless_entropy_total_+=data_combined(index(0), index(1));
  }

  std::cout << "totla wireless info is:" << info_wireless_entropy_total_ << std::endl;
  std::cout << "wireless resoluiton is:" << mapping_module_->getMapLayer(MAP::WIRELESS)->map.getResolution() << std::endl;

}

double view_evaluator_base::calculateIG(geometry_msgs::Pose p, Victim_Map_Base *mapping_module){

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  double IG_view=0;
  double IG_view_count=0;

  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      IG_view+=getCellEntropy(position,mapping_module);
      IG_view_count+=1;
    }
  }

  // std::cout << "found information gain is.. " << IG_view << std::endl;
  //if (IG_view_count!=0) IG_view=IG_view/IG_view_count;
  return IG_view;
}

double view_evaluator_base::calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
  //std::cout << "[ViewEvaluatorBase]: " << cc.yellow << "Warning: calculateUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;
  double IG = calculateIG(p,mapping_module);
  return IG;
}

void view_evaluator_base::evaluate(){

  if (mapping_module_->Maptype==MAP::WIRELESS)   // if the map is Wireless then use wireless evaluator
  {
      evaluateWireless();
      return;
  }

  if (mapping_module_->Maptype==MAP::COMBINED)   // if the map is combined then use combined evaluator
  {
      evaluateCombined();
      return;
  }

  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateUtiltiy(p,mapping_module_);

        if (utility>=0){
    info_utilities_.push_back(utility);
}
        // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
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

       view_gen_->visualizeSelectedpose(selected_pose_);
      //view_gen_->visualize(view_gen_->generated_poses, view_gen_->rejected_poses,selected_pose_);

 info_distance_total_ += calculateDistance(selected_pose_);
 mapping_module_->raytracing_->Done();

 std::cout << "Map of resoltuion " << mapping_module_->map.getResolution() << std::endl;
}

geometry_msgs::Pose view_evaluator_base::getTargetPose()
{
  return selected_pose_;
}


double view_evaluator_base::calculateDistance(geometry_msgs::Pose p)
{
  return sqrt(
        (p.position.x-current_pose_.position.x)*(p.position.x-current_pose_.position.x) +
        (p.position.y-current_pose_.position.y)*(p.position.y-current_pose_.position.y) +
        (p.position.z-current_pose_.position.z)*(p.position.z-current_pose_.position.z)
        );
}

double view_evaluator_base::calculateCombinedUtility(geometry_msgs::Pose p)
{
 // std::cout << "[ViewEvaluatorBase]: " << cc.yellow << "Warning: calculateCombinedUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;

  double IG_vision = calculateUtiltiy(p,mapping_module_->getMapLayer(MAP::DL));
  double IG_thermal = calculateUtiltiy(p,mapping_module_->getMapLayer(MAP::THERMAL));
  double IG_wireless=calculateWirelessUtility(p,mapping_module_->getMapLayer(MAP::WIRELESS));

  return ((alpha*IG_vision)+(beta*IG_thermal)+(gama*IG_wireless))/(alpha+beta+gama);
}


void view_evaluator_base::evaluateCombined()
{
  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateCombinedUtility(p);

        if (utility>=0){
    info_utilities_.push_back(utility);
}
        // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
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

       view_gen_->visualizeSelectedpose(selected_pose_);
      //view_gen_->visualize(view_gen_->generated_poses, view_gen_->rejected_poses,selected_pose_);

info_distance_total_ += calculateDistance(selected_pose_);

info_dl_selected_utility_ = 0;
info_thermal_selected_utility_ = 0;
info_wireless_selected_utility_ = 0;

// // evaluation for vision map only
// for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
// {
//   geometry_msgs::Pose p = view_gen_->generated_poses[i];
//   double utility = calculateUtiltiy(p,mapping_module_->getMapLayer(MAP::DL));
//   if (utility > info_dl_selected_utility_)
//     info_dl_selected_utility_ = utility;
// }

// // evaluation for thermal map only
// for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
// {
//   geometry_msgs::Pose p = view_gen_->generated_poses[i];
//   double utility = calculateUtiltiy(p,mapping_module_->getMapLayer(MAP::THERMAL));
//   if (utility > info_thermal_selected_utility_)
//     info_thermal_selected_utility_ = utility;
// }

// // evaluation for wireless map only
// for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
// {
//   geometry_msgs::Pose p = view_gen_->generated_poses[i];
//   double utility = calculateWirelessUtility(p,mapping_module_->getMapLayer(MAP::WIRELESS));
//   if (utility > info_wireless_selected_utility_)
//     info_wireless_selected_utility_ = utility;
// }

 mapping_module_->raytracing_->Done();
 mapping_module_->getMapLayer(MAP::DL)->raytracing_->Done();
 mapping_module_->getMapLayer(MAP::THERMAL)->raytracing_->Done();
 mapping_module_->getMapLayer(MAP::WIRELESS)->raytracing_->Done();
}
// Wireless Related Functions
double view_evaluator_base::calculateWirelessIG(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
  double IG_view=0;
  Position center(p.position.x,p.position.y);
  double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Position position;
      Index index=*iterator;
      mapping_module->map.getPosition(index, position);
      IG_view+=getCellEntropy(position,mapping_module);
    }
      return IG_view;
}

void view_evaluator_base::evaluateWireless()
{
  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_selected_direction_=0;
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();


   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];

      double utility_direction = calculateIG(p,mapping_module_);
      double utility;
      if (i==0){
        utility = calculateWirelessUtility(p,mapping_module_);
      }
      if (i!=0) {
        if (!IsSamePosition(view_gen_->generated_poses[i],view_gen_->generated_poses[i-1]))
        {
        utility = calculateWirelessUtility(p,mapping_module_);
        info_selected_direction_=0;
        }
        else
        utility = info_utilities_.back();
      }

        if (utility>=0){
    info_utilities_.push_back(utility);
          }
         // First select the best Pose based on the Wireless utility
        if (utility > info_selected_utility_)
        {
         info_selected_utility_ = utility;
          selected_pose_ = p;
        }

        // Second select the best exploration direction for the wireless best selected pose
        if (!(utility-info_selected_utility_))
          if (utility_direction>info_selected_direction_)
        {
         info_selected_direction_ = utility_direction;
          selected_pose_ = p;
        }
    }
       // No valid poses found, end
       if ( std::isnan(selected_pose_.position.x) )
       {
         return;
       }

       view_gen_->visualizeSelectedpose(selected_pose_);
      //view_gen_->visualize(view_gen_->generated_poses, view_gen_->rejected_poses,selected_pose_);

 info_distance_total_ += calculateDistance(selected_pose_);
 mapping_module_->raytracing_->Done();
  }

double view_evaluator_base::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
 // std::cout << "[ViewEvaluatorBase]: " << cc.yellow << "Warning: calculateWirelessUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;
  double IG = calculateWirelessIG(p,mapping_module);
  return IG;
}


std::string view_evaluator_base::getMethodName()
{
  return "IG";
}

bool view_evaluator_base::IsSamePosition(geometry_msgs::Pose p1,geometry_msgs::Pose p2)
{
 return !((p1.position.x-p2.position.x)+(p1.position.y-p2.position.y)+(p1.position.z-p2.position.z));
}







