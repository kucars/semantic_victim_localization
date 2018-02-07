#ifndef VIEW_EVALUATOR_IG_H
#define VIEW_EVALUATOR_IG_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_ig : public view_evaluator_base
{
public:
  view_evaluator_ig();

   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();


};

#endif // VIEW_EVALUATOR_IG_H
