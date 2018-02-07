#ifndef VIEW_EVALUATOR_MAX_MAX_H
#define VIEW_EVALUATOR_MAX_MAX_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_MaxMax : public view_evaluator_base
{
public:
  view_evaluator_MaxMax();

  double calculateUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   double calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);

   std::string getMethodName();

private:
  double max_belief;

};

#endif // VIEW_EVALUATOR_MAX_MAX_H
