#include <victim_localization/view_evaluator_max_max.h>

view_evaluator_MaxMax::view_evaluator_MaxMax():
  view_evaluator_base()
{
  max_belief=0;
}

void view_evaluator_MaxMax::getAbsoluteMax(Victim_Map_Base *mapping_module, double &absolute_prob)
{
    double absoluteMax=0;
    grid_map::Matrix& data = mapping_module_->map[mapping_module_->layer_name];
    for (GridMapIterator iterator(mapping_module_->map); !iterator.isPastEnd(); ++iterator)
    {
        const Index index(*iterator);
        double prob=data(index(0),index(1));
        if (prob>absoluteMax)
            absoluteMax=prob;
    }
    absolute_prob=absoluteMax;
}

double view_evaluator_MaxMax::calculateUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module){

  double absolute_max;
  double curr_pro;
  max_belief=0;
  getAbsoluteMax(mapping_module,absolute_max);

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);

  double Info_view=0;
  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;
    if(temp_Map.atPosition("temp", position)==0.5) continue;

    if(temp_Map.atPosition("temp", position)==0){
      curr_pro = mapping_module->map.at(mapping_module->getlayer_name(),index);
      if(curr_pro==absolute_max)
       max_belief++;
  }
    if(temp_Map.atPosition("temp", position)==1){
       curr_pro= mapping_module->map.at(mapping_module->getlayer_name(),index);
      if (curr_pro != 0.5)
          if (curr_pro==absolute_max)
               max_belief++;
  }
}
       Info_view=max_belief;

  return Info_view;
}

double view_evaluator_MaxMax::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
    double absolute_max;
    max_belief=0;
    double curr_pro;
    getAbsoluteMax(mapping_module,absolute_max);

  double Info_view=0;
  Position center(p.position.x,p.position.y);
  double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
        !iterator.isPastEnd(); ++iterator) {
      Index index=*iterator;
      curr_pro= mapping_module->map.at(mapping_module->getlayer_name(),index);
      if(curr_pro==absolute_max)
       max_belief++;
    }

    Info_view=max_belief;
      return Info_view;
}

std::string view_evaluator_MaxMax::getMethodName()
{
  return "MAXMAX";
}







