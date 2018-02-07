#ifndef VIEW_EVALUATOR_IG_EXP_H
#define VIEW_EVALUATOR_IG_EXP_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_ig_exp : public view_evaluator_base
{
public:
  view_evaluator_ig_exp();

   double calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   double calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   double calculateCombinedUtility(geometry_msgs::Pose p);

   std::string getMethodName();
   double w_dist_;
};

#endif // VIEW_EVALUATOR_IG_EXP_H
