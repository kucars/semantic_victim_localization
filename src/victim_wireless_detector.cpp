#include "victim_localization/victim_wireless_detector.h"


victim_wireless_detector::victim_wireless_detector():
victim_detector_base()
{
  ros::param::param<double>("~Number_of_sensors", N_sensor , 5);
  ros::param::param<double>("~frequency", frequency , pow(10,9));
  ros::param::param<double>("~Lamda", lamda , 0.3);
  ros::param::param<double>("~SNR", SNR , 15);
  ros::param::param<double>("~Number_of_samples", K , 100);
  ros::param::param<double>("~reference_distance", d0 , 1);
  ros::param::param<double>("~path_loss", path_loss , 7);
  ros::param::param<double>("~distance_threshold", distance_threshold , 8);
  ros::param::param<double>("~victim_location_x", target_loc[0] , -4.331);
  ros::param::param<double>("~victim_location_y", target_loc[1] , -1.427);

  current_loc[0]=0;
  current_loc[1]=0;
}


victim_wireless_detector::~victim_wireless_detector(){}

void victim_wireless_detector::EstimateDist(){
    setCurrentPose();
    distance_to_victim=0;

    // pass ros parameters to matlab custom variables
    emxArray_real_T *meas_dist;
    emxArray_real_T *x;
    emxArray_real_T *y;
    emxInitArray_real_T(&meas_dist, 2);
    emxInitArray_real_T(&x, 1);
    emxInitArray_real_T(&y, 1);

    RSSmodel2(current_loc, target_loc,N_sensor,lamda,SNR,K,d0,path_loss,meas_dist, x, y);

    // get the result from matlab model
   distance_to_victim = meas_dist->data[0];

   std::cout << "victim_found_at_estimated_distance: " << distance_to_victim << std::endl;
   std::cout << "while the actual distance is: " <<
               sqrt((target_loc[0]-current_loc[0])*(target_loc[0]-current_loc[0])
               + (target_loc[1]-current_loc[1])*(target_loc[1]-current_loc[1]))
       << " " << "for an SNR value of " << SNR;
}


  void victim_wireless_detector::performDetection(){
    EstimateDist();
  }


Status victim_wireless_detector::getDetectorStatus(){
  Status status;
  if (distance_to_victim<distance_threshold)
  status.victim_found = true;
  else
  status.victim_found = false;

  status.victim_loc[0]=distance_to_victim;status.victim_loc[1]=0;
  return status;
}

void victim_wireless_detector::setCurrentPose(){
  current_loc[0]=vehicle_comm_->GetPose().position.x;
  current_loc[1]=vehicle_comm_->GetPose().position.y;
}

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "wireless_detector");
//  ros::NodeHandle nh_;
//  ros::NodeHandle pnh_;
//  victim_wireless_detector *WD;
//  WD= new victim_wireless_detector();
//  WD->performDetection();
//  ros::spin();
//  return 0;
//}
