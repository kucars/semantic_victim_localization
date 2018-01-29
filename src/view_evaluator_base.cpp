#include "victim_localization/view_evaluator_base.h"


view_evaluator_base::view_evaluator_base():
  info_selected_utility_(-std::numeric_limits<float>::infinity()), //-inf
  info_distance_total_(0)
{
  ros::param::param<double>("~fov_horizontal", HFOV_deg , 58);
  ros::param::param<double>("~fov_vertical", VFOV_deg , 45);
  ros::param::param<double>("~depth_range_min", max_depth_d , 5);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);
  ros::param::param<std::string>("~camera_optical_frame", camera_optical_frame ,std::string("/floating_sensor/camera_depth_optical_frame"));
  ros::param::param<std::string>("~base_frame", base_frame ,std::string("/floating_sensor/base_link"));

  const_=max_depth_d/(2*cos(DEG2RAD(HFOV_deg)));

  ros::NodeHandle nh_private_("~");
}

double view_evaluator_base::getCellEntropy(Position cell_)
{
  double p= mapping_module_->map.atPosition(MapLayer,cell_);
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
  MapLayer = mapping_module_->getlayer_name();
  current_pose_ = view_gen_->current_pose_;
  tree_ = view_gen_->manager_->octree_;

  current_yaw_=pose_conversion::getYawFromQuaternion(current_pose_.orientation);

  info_entropy_total_=0;

  grid_map::Matrix& data = mapping_module_->map[mapping_module_->layer_name];
  for (GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator) {
      const Index index(*iterator);
      info_entropy_total_+=data(index(0), index(1));
  }
}

double view_evaluator_base::calculateIG(geometry_msgs::Pose p){

  grid_map::GridMap temp_Map;

  mapping_module_->raytracing_->Initiate(false);

  temp_Map=mapping_module_->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  double IG_view=0;

  for (grid_map::GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module_->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      IG_view+=getCellEntropy(position);
    }
  }
  return IG_view;
}

double view_evaluator_base::calculateUtiltiy(geometry_msgs::Pose p)
{
  std::cout << "[ViewEvaluatorBase]: " << cc.yellow << "Warning: calculateUtility() not implimented, defaulting to classical IG calculation\n" << cc.reset;
  double IG = calculateIG(p);
  return IG;
}

void view_evaluator_base::evaluate(){

  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateIG(p);

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

std::string view_evaluator_base::getMethodName()
{
  return "IG";
}







