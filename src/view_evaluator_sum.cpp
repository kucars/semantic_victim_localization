#include "victim_localization/view_evaluator_sum.h"

view_evaluator_sum::view_evaluator_sum():
    view_evaluator_base() //Call base class constructor
{
    ros::param::param<double>("~view_evaluator_weight_distance", w_dist_, 1.0);
}

view_evaluator_sum::~view_evaluator_sum(){}

double view_evaluator_sum::calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &new_cell_percentage){

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  double IG_view=0;
  double IG_view_count=0;
  double new_cells_count=0;
  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      IG_view+=mapping_module->map.atPosition(mapping_module->getlayer_name(),position);
      IG_view_count+=1;
    }

    if (mapping_module->map.atPosition(mapping_module->getlayer_name(),position)==0.5)
        new_cells_count++;
  }

  //double dist = calculateDistance(p);
  //return IG_view*exp(-dist*w_dist_);
  new_cell_percentage=new_cells_count/IG_view_count;
  return IG_view;
}

double view_evaluator_sum::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &new_cell_percentage)
{
  double IG_view=0;
  double IG_view_count=0;
  double new_cells_count=0;
  Position center(p.position.x,p.position.y);
  double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Position position;
      Index index=*iterator;
      mapping_module->map.getPosition(index, position);
      IG_view+=mapping_module->map.atPosition(mapping_module->getlayer_name(),position);
      IG_view_count+=1;

      if (mapping_module->map.atPosition(mapping_module->getlayer_name(),position)==0.5)
          new_cells_count++;

    }

    new_cell_percentage=new_cells_count/IG_view_count;
      return IG_view;
}



std::string view_evaluator_sum::getMethodName()
{
  return "Evaluator_SUM";
}
