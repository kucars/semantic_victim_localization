#ifndef VIEW_EVALUATOR_MAX_MAX_H
#define VIEW_EVALUATOR_MAX_MAX_H

#include <victim_localization/view_evaluator_base.h>


class view_evaluator_MaxMax : public view_evaluator_base
{
public:
  view_evaluator_MaxMax();


   double GetMAXANDCOUNT(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count);
   double GetMAXANDCOUNTWIRELESS(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count);
   double GetMAXANDCOUNCombined(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count);
   void evaluate();
   void evaluateCombined();
   void evaluateWireless();


   std::string getMethodName();
   std::vector<double> Info_View_max;
   std::vector<double> Info_View_max_count;
   std::vector<double>  Info_WirelessDiection;
   std::vector<geometry_msgs::Pose> Info_poses;


private:
  double max_belief;

};

#endif // VIEW_EVALUATOR_MAX_MAX_H
