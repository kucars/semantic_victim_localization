#ifndef VIEW_EVALUATOR_MAX_SUM_H
#define VIEW_EVALUATOR_MAX_SUM_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_MaxSUM : public view_evaluator_base
{
public:
  view_evaluator_MaxSUM();

   void evaluate();
   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();

};
#endif // VIEW_EVALUATOR_MAX_SUM_H
