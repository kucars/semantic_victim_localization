#ifndef VIEW_EVALUATOR_FFOV_H
#define VIEW_EVALUATOR_FFOV_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_FOV: public view_evaluator_base
{
public:
  view_evaluator_FOV();
  ~view_evaluator_FOV();

   double FOVSize(geometry_msgs::Pose p, Victim_Map_Base *mapping_module);
   std::string getMethodName();

   std::vector<double> Info_View_counts;
   std::vector<geometry_msgs::Pose> Info_poses;

   std::vector<double> Info_View_counts_dl;
   std::vector<double> Info_View_counts_thermal;

   void evaluate();
   void evaluateCombined();

private:
   double w_dist_;

};

#endif // VIEW_EVALUATOR_FFOV_H
