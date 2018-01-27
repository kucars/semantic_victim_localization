#ifndef VIEW_GENERATOR_IG_NN_ADAPTIVE_H
#define VIEW_GENERATOR_IG_NN_ADAPTIVE_H

#include "victim_localization/view_generator_ig.h"

class view_generator_ig_nn_adaptive : public view_generator_IG
{
public:
  view_generator_ig_nn_adaptive();
  virtual void generateViews();
  std::string getMethodName();
  double scale_factor_;
  double scale_multiplier_;


private:
  int minima_iterations_;
  double minima_threshold_;
  bool isStuckInLocalMinima();
  bool IsPreviouslyCheckedSamples(double i_x,double i_y);
  bool do_adaptive_generation;


};

#endif // VIEW_GENERATOR_IG_NN_ADAPTIVE_H
