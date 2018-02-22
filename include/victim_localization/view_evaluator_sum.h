#ifndef VIEW_EVALUATOR_SUMM_H
#define VIEW_EVALUATOR_SUMM_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_sum: public view_evaluator_base
{
public:
  view_evaluator_sum();
  ~view_evaluator_sum();

   double calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   double calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   std::string getMethodName();

private:
   double w_dist_;

};

#endif // VIEW_EVALUATOR_SUMM_H
