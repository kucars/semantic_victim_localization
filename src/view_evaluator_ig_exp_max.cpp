#include "victim_localization/view_evaluator_ig_exp_max.h"

view_evaluator_ig_exp_max::view_evaluator_ig_exp_max():
  view_evaluator_base() //Call base class constructor
{
   ros::param::param<double>("~view_selecter_weight_distance", w_dist_, 1.0);
}

double view_evaluator_ig_exp_max::calculateUtility(geometry_msgs::Pose p)
{
  double IG_MAX = calculateIGMax(p);
  double dist = calculateDistance(p);

  return IG_MAX*exp(-dist*w_dist_);
}

std::string view_evaluator_ig_exp_max::getMethodName()
{
return "IG_exp_max";

}

double view_evaluator_ig_exp_max::calculateIGMax(geometry_msgs::Pose p)
{
  grid_map::GridMap temp_Map;
  double max=0;
  double current_prob=0;
  double IG_view=0;


  mapping_module_->raytracing_->Initiate(false);

  temp_Map=mapping_module_->raytracing_->Generate_2D_Safe_Plane(p,true);

  for (grid_map::GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module_->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      IG_view+=getCellEntropy(position);
    }
    current_prob=mapping_module_->map.at(mapping_module_->getlayer_name(),index);

    if (current_prob>max){
      max=current_prob;
    }

  }
  return max*IG_view;
}

