#ifndef VIEW_EVALUATOR_LOG_H
#define VIEW_EVALUATOR_LOG_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_log_reward : public view_evaluator_base
{
public:
  view_evaluator_log_reward();

   double calculateUtility(geometry_msgs::Pose p);
   std::string getMethodName();


};

#endif // VIEW_EVALUATOR_LOG_H
