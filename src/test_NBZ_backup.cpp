#include <victim_localization/test_NBV.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


TestNBZ::TestNBZ()
{
    // >>>>>>>>>>>>>>>>>
    // Initialization
    // >>>>>>>>>>>>>>>>>
    state = NBVState::INITIALIZING;
    ROS_INFO("test_NBZ: Begin!");
}


void TestNBZ::initVehicle(){

     vehicle_ = new VehicleControlIris();

    // Set starting position
    ros::Duration(2.0).sleep();
    int pose_number;
    std::string pose_number_str;
    ros::param::param<int>("~uav_start_pose", pose_number, 1);
    pose_number_str = std::to_string(pose_number);

    double x, y, z, yaw;
    ros::param::param<double>("~uav_pose_x_" + pose_number_str, x, -8);
    ros::param::param<double>("~uav_pose_y_" + pose_number_str, y, -8);
    ros::param::param<double>("~uav_pose_z_" + pose_number_str, z, 1.5);
    ros::param::param<double>("~uav_pose_yaw_" + pose_number_str, yaw, 0);

    vehicle_->setWaypoint(x, y, z, yaw);

    printf("Starting vehicle\n");
    vehicle_->start();
    printf("Moving vehicle\n");
    vehicle_->moveVehicle(1.5);
    printf("Done moving\n");
  }

void TestNBZ::initMap(){
  int map_type;
    ros::param::param("~map_type", map_type, 0);

    switch(map_type)
    {
    default:
    case 0:
      Map_ = new victim_map_DL();
      break;
    }
}


void TestNBZ::initParameters(){

  ros::param::param("~detection_enabled", detection_enabled, false);//for debugging

  View_evaluate_ = new view_evaluator_IG();
  view_generate_ = new view_generator_IG();
  if (detection_enabled) Victim_detection_DL_ = new SSD_Detection_with_clustering();

  //Passing the Mapping and view_generator module to the view_evaluator
  View_evaluate_->setMappingModule(Map_);
  View_evaluate_->setViewGenerator(view_generate_);


}



void TestNBZ::evaluateViewpoints()
{
  if (state != NBVState::VIEWPOINT_EVALUATION)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc.reset;
    return;
  }

   std::cout << "[test_NBZ] " << cc.green << "Evaluating viewpoints\n" << cc.reset;

   // Evaluate viewpoints
  View_evaluate_->update_parameters();
  View_evaluate_->evaluate();


  // Set the pose of the next best view
  geometry_msgs::Pose p = View_evaluate_->getTargetPose();
  if ( std::isnan(p.position.x) )
  {
    std::cout << "[test_NBZ] " << cc.red << "View selecter determined all poses are invalid. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }
  vehicle_->setWaypoint(p);


  std::cout << "[test_NBZ] " << cc.green << "Done evaluating viewpoints\n" << cc.reset;
  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void TestNBZ::generateViewpoints()
{
  if (state != NBVState::VIEWPOINT_GENERATION)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to generate viewpoints out of order\n" << cc.reset;
    return;
  }

   std::cout << "[test_NBZ] " << cc.green << "Generatring viewpoints\n" << cc.reset;

  view_generate_->setCurrentPose(vehicle_->getPose());
  view_generate_->generateViews();

  if (view_generate_->generated_poses.size() == 0)
  {
    std::cout << "[test_NBZ] " << cc.red << "View generator created no poses. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
  }
  else
  {
    state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
  }
}


void TestNBZ::UpdateMap()
{
  if (state != NBVState::UPDATE_MAP)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to update map out of order\n" << cc.reset;
    return;
  }

   std::cout << "[test_NBZ] " << cc.green << "Updateing [" << Map_->getlayer_name() << "]" << cc.reset;

  Map_->setCurrentPose(vehicle_->getPose());

  if (detection_enabled)
    Map_->setDetectionResult(Victim_detection_DL_->getClusterCentroid()
                           ,Victim_detection_DL_->getDetectorStatus());
  else Map_->setDetectionResult((vehicle_->getPose()).position,0);

  Map_->Update();

  if (Map_->getMapResultStatus().victim_found)
  {
    std::cout << cc.magenta << "Victim Found at Location: " << "(x,y)=" << "(" <<
                 (Map_->getMapResultStatus().victim_loc)[0] << "," <<
                 (Map_->getMapResultStatus().victim_loc)[1] << ") terminating...\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }

    state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
}


void TestNBZ::runStateMachine(){
  ROS_INFO("test_NBZ: Starting vehicle. Waiting for current position information.");
 state = NBVState::STARTING_ROBOT;

 ros::Rate loop_rate(30);
   while (ros::ok())
   {
     switch(state)
     {
       case NBVState::TERMINATION_MET:
       vehicle_->moveVehicle(1.5);
       break;

       case NBVState::STARTING_ROBOT:
         if( vehicle_->isReady())
         {
           state = NBVState::STARTING_ROBOT_COMPLETE;
           break;
         }
         vehicle_->start();
         break;

       case (NBVState::STARTING_ROBOT_COMPLETE):
       case(NBVState::STARTING_DETECTION):
       if (detection_enabled) Victim_detection_DL_->FindClusterCentroid();
       state = NBVState::DETECTION_COMPLETE;
       break;

     case (NBVState::DETECTION_COMPLETE):
       state = NBVState::UPDATE_MAP;
       UpdateMap();
       state = NBVState::UPDATE_MAP_COMPLETE;
       break;

       case NBVState::UPDATE_MAP_COMPLETE:
         state = NBVState::VIEWPOINT_GENERATION ;
         generateViewpoints();
       break;

     case NBVState::VIEWPOINT_GENERATION_COMPLETE:
       state = NBVState::VIEWPOINT_EVALUATION;
       evaluateViewpoints();
     break;

     case NBVState::VIEWPOINT_EVALUATION_COMPLETE:
       vehicle_->moveVehicle(1.5);
       state = NBVState::STARTING_DETECTION;
       break;

     }

     ros::spinOnce();
     loop_rate.sleep();
 }
}



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "test_NBZ");

  //Create an object of class test_NBZ
  TestNBZ *test_;
  test_ = new TestNBZ();
  test_->initMap();
  test_->initVehicle();
  test_->initParameters();



  test_->runStateMachine();

  return 0;
}
