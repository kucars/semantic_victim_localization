#include "victim_localization/view_evaluator_ig.h"

view_evaluator_ig::view_evaluator_ig():
  view_evaluator_base() //Call base class constructor
{
}

double view_evaluator_ig::calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module,double & new_cell_percentage)
{
  double IG = calculateIG(p,mapping_module,new_cell_percentage);
  return IG;
}

double view_evaluator_ig::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module,double & new_cell_percentage)
{
  double IG = calculateWirelessIG(p,mapping_module,new_cell_percentage);
  return IG;
}

double view_evaluator_ig::calculateCombinedUtility(geometry_msgs::Pose p,double & new_cell_percentage)
{
  double IG =view_evaluator_base::calculateCombinedUtility(p,new_cell_percentage);
  return IG;
}

std::string view_evaluator_ig::getMethodName()
{
  return "Classic IG";
}







