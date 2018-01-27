#ifndef VIEW_EVALUATOR_MIN_NEIGH_H
#define VIEW_EVALUATOR_MIN_NEIGH_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_MinNEIGH : public view_evaluator_base
{
public:
  view_evaluator_MinNEIGH();

   void evaluate();
   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();

private:
  double min_belief;
  double max_belief;

};

#endif // VIEW_EVALUATOR_MIN_NEIGH_H
