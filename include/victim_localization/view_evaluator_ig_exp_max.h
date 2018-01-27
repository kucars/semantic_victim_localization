#ifndef VIEW_EVALUATOR_IG_EXP_MAX_H
#define VIEW_EVALUATOR_IG_EXP_MAX_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_ig_exp_max: public view_evaluator_base
{
public:
  view_evaluator_ig_exp_max();

   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();

private:
   double w_dist_;
   double calculateIGMax(geometry_msgs::Pose p);


};

#endif // VIEW_EVALUATOR_IG_EXP_MAX_H
