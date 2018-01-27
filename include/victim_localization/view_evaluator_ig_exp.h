#ifndef VIEW_EVALUATOR_IG_EXP_H
#define VIEW_EVALUATOR_IG_EXP_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_ig_exp : public view_evaluator_base
{
public:
  view_evaluator_ig_exp();

   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();
   double w_dist_;
};

#endif // VIEW_EVALUATOR_IG_EXP_H
