#include "victim_localization/view_generator_ig_nn_adaptive.h"

view_generator_ig_nn_adaptive::view_generator_ig_nn_adaptive():
view_generator_IG(),
scale_factor_(1),
do_adaptive_generation(true)
{
  ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 5);
  ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 3.0);
  ros::param::param<double>("~view_generator_nn_adaptive_scale_multiplier", scale_multiplier_, 2.0);

  generator_type=Generator::NN_Generator;
}

bool view_generator_ig_nn_adaptive::isStuckInLocalMinima()
{
  if (minima_iterations_ > nbv_history_->iteration)
    return false;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(4))
    return true;

  // Find max entropy change in the past few iterations
  float utility = nbv_history_->getMaxUtility(minima_iterations_);
  // std::cout << "maximum_entropy_change_is: " << utility << std::endl;
  if (utility < minima_threshold_)
    return true;

  return false;
}

void view_generator_ig_nn_adaptive::generateViews()
{
  if (isStuckInLocalMinima() && do_adaptive_generation)
  {
    scale_factor_*= scale_multiplier_;
    std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Increasing scale factor to " << scale_factor_ << "\n" << cc.reset;

    if (scale_factor_ > 8)
    {
      std::cout << "[ViewGeneratorNNAdaptive]: " << cc.red << "Warning: Scale factor very large: Resetting" << scale_factor_ << "\n" << cc.reset;
      scale_factor_ = 1;
      do_adaptive_generation=false;
    }
  }
  else
  {
    scale_factor_ = 1;
  }

  // Scale up sampling resolution
  res_x_=1;
  res_y_=1;
  res_z_=1;
  double backup_res_x_ = res_x_;
  double backup_res_y_ = res_y_;
  double backup_res_z_ = res_z_;

  res_x_ *= scale_factor_;
  res_y_ *= scale_factor_;
  res_z_ *= scale_factor_;

  // for scale_factor=1, generate viewpoint using NN generator
  if (scale_factor_==1)
  {  res_x_=0.8;
    res_y_=0.8;
    res_z_=0.8;
    view_generator_IG::generateViews(true);
    nav_type = 0; // set navigation type as straight line for adaptive_nn_view_generator
    generator_type=Generator::NN_Generator;
    return;
  }

  // for scale_factor>1, generate viewpoint using sampled grid
  else
  {
    generator_type=Generator::NN_Adaptive_Generator;

    std::vector<geometry_msgs::Pose> initial_poses;
    generated_poses.clear();
    rejected_poses.clear();

    double currX = current_pose_.position.x;
    double currY = current_pose_.position.y;
    double currZ = current_pose_.position.z;
    double currYaw = pose_conversion::getYawFromQuaternion(current_pose_.orientation);


    //Generating 3-D state lattice as z-axis movement is restrained (fixed)
    for (double i_x=-res_x_; i_x<=res_x_; i_x=i_x+1)
    {
      for (double i_y=-res_y_; i_y<=res_y_; i_y=i_y+1)
      {
         //if (IsPreviouslyCheckedSamples(i_x,i_y)){
         //continue;
         //}
        for (double i_yaw=-M_PI; i_yaw<M_PI; i_yaw+=res_yaw_)
        {

          // Do not generate any viewpoints in current location
          //if (i_x==0 && i_y==0)
          //continue;

          geometry_msgs::Pose p;
          p.position.x = currX + i_x*cos(currYaw) + i_y*sin(currYaw);
          p.position.y = currY - i_x*sin(currYaw) + i_y*cos(currYaw);
          p.position.z = uav_fixed_height ;//+ res_z_*i_z; // z-axis movement is fixed

          p.orientation = pose_conversion::getQuaternionFromYaw(currYaw + i_yaw);
          initial_poses.push_back(p);
        }
      }
    }

  for (int i=0; i<initial_poses.size(); i++)
  {
    if ( isValidViewpoint(initial_poses[i]) )
    {
      generated_poses.push_back(initial_poses[i]);
    }
    else
    {
      rejected_poses.push_back(initial_poses[i]);
    }
  }

  std::cout << "[ViewGenerator] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;

  nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
  }

  // Return scale to normal
  res_x_ = backup_res_x_;
  res_y_ = backup_res_y_;
  res_z_ = backup_res_z_;
}

bool view_generator_ig_nn_adaptive::IsPreviouslyCheckedSamples(double i_x,double i_y)
{
  if ((i_x>=(-res_x_/scale_multiplier_)) && (i_x<=(res_x_/scale_multiplier_)))
    if ((i_y>=(-res_y_/scale_multiplier_)) && (i_y<=(res_y_/scale_multiplier_)))
       return true;

  return false;
}

std::string view_generator_ig_nn_adaptive::getMethodName()
{
  return "NN Adaptive";
}
