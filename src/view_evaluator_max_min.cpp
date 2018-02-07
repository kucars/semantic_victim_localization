#include <victim_localization/view_evaluator_max_min.h>

view_evaluator_MaxMIN::view_evaluator_MaxMIN():
  view_evaluator_base()
{
  min_belief=std::numeric_limits<float>::infinity();
}

double view_evaluator_MaxMIN::calculateUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module){

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);

  double Info_view=std::numeric_limits<double>::infinity();

  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      double curr_pro= mapping_module->map.at(mapping_module_->getlayer_name(),index);
      if(curr_pro<min_belief)
       min_belief=curr_pro;
  }
}
  if (!std::isinf(min_belief))
       Info_view=min_belief;

  return Info_view;
}
void view_evaluator_MaxMIN::evaluate(){

  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  info_selected_utility_ =  std::numeric_limits<float>::infinity(); //-inf
  info_utilities_.clear();

  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double utility = calculateUtility(p,mapping_module_);

        if (utility>=0){
    info_utilities_.push_back(utility);
}
        // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
        if (utility < info_selected_utility_)
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


std::string view_evaluator_MaxMIN::getMethodName()
{
  return "MAXMIN";
}







