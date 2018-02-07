#include <victim_localization/view_evaluator_max_max.h>

view_evaluator_MaxMax::view_evaluator_MaxMax():
  view_evaluator_base()
{
  max_belief=-std::numeric_limits<float>::infinity();
}

double view_evaluator_MaxMax::calculateUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module){

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);

  double Info_view=0;

  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
      double curr_pro= mapping_module->map.at(mapping_module->getlayer_name(),index);
      if(curr_pro>max_belief)
       max_belief=curr_pro;
  }
}
  if (!std::isinf(max_belief))
       Info_view=max_belief;

  return Info_view;
}

double view_evaluator_MaxMax::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
  double Info_view=0;
  Position center(p.position.x,p.position.y);
  double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Index index=*iterator;
      double curr_pro= mapping_module->map.at(mapping_module->getlayer_name(),index);
      if(curr_pro>max_belief)
       max_belief=curr_pro;
    }

    if (!std::isinf(max_belief))
         Info_view=max_belief;
      return Info_view;
}

std::string view_evaluator_MaxMax::getMethodName()
{
  return "MAXMAX";
}







