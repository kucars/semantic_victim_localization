#ifndef VIEW_EVALUATOR_MAX_MIN_H
#define VIEW_EVALUATOR_MAX_MIN_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_MaxMIN : public view_evaluator_base
{
public:
  view_evaluator_MaxMIN();

   void evaluate();
   double calculateUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   std::string getMethodName();

private:
  double min_belief;

};

#endif // VIEW_EVALUATOR_MAX_MIN_H
