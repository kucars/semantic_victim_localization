#include <victim_localization/view_evaluator_max_sum.h>

view_evaluator_MaxSUM::view_evaluator_MaxSUM():
  view_evaluator_base(){}

double view_evaluator_MaxSUM::calculateUtility(geometry_msgs::Pose p){

  grid_map::GridMap temp_Map;

  mapping_module_->raytracing_->Initiate(false);

  temp_Map=mapping_module_->raytracing_->Generate_2D_Safe_Plane(p,true);

  double Info_view=0;

  for (grid_map::GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module_->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      double curr_pro= mapping_module_->map.at(mapping_module_->getlayer_name(),index);
       Info_view+=curr_pro;
  }
}
  return Info_view;
}
void view_evaluator_MaxSUM::evaluate(){

  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateUtility(p);

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


std::string view_evaluator_MaxSUM::getMethodName()
{
  return "MAXSUM";
}







