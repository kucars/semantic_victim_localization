#ifndef VIEW_EVALUATOR_IG_EXP_MAX_H
#define VIEW_EVALUATOR_IG_EXP_MAX_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_ig_exp_max: public view_evaluator_base
{
public:
  view_evaluator_ig_exp_max();

  double calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &new_cell_percentage);
  double calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &new_cell_percentage);

   std::string getMethodName();

private:
   double w_dist_;
   double calculateIGMax(geometry_msgs::Pose p, Victim_Map_Base *mapping_module,  double &new_cell_percentage);
   double calculateWirelessIGMAX(geometry_msgs::Pose p, Victim_Map_Base *mapping_module ,  double &new_cell_percentage);
};

#endif // VIEW_EVALUATOR_IG_EXP_MAX_H
