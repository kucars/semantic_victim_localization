#include "victim_localization/view_evaluator_ig.h"

view_evaluator_ig::view_evaluator_ig():
  view_evaluator_base() //Call base class constructor
{
}

double view_evaluator_ig::calculateUtility(geometry_msgs::Pose p)
{
  double IG = calculateIG(p);
  return IG;
}

std::string view_evaluator_ig::getMethodName()
{
  return "Classic IG";
}







