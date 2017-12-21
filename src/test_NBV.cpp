#include <victim_localization/test_NBV.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


TestNBZ::TestNBZ(const ros::NodeHandle &nh_,const ros::NodeHandle &nh_private_ ):
  nh(nh_),
  nh_private(nh_private_)
{
    // >>>>>>>>>>>>>>>>>
    // Initialization
    // >>>>>>>>>>>>>>>>>
    //ros::NodeHandle nh_private("~");
   pub_iteration_info = nh.advertise<victim_localization::IterationInfo>("nbv_exploration/iteration_info", 10);
    state = NBVState::INITIALIZING;
    ROS_INFO("test_NBZ: Begin!");
}


void TestNBZ::initVehicle(){

     vehicle_ = new VehicleControlIris();

    // Set starting position
    ros::Duration(2.0).sleep();


         vehicle_ = new VehicleControlIris();
    
        // Set starting position
        ros::Duration(2.0).sleep();
    
        double x_, y_, z_, yaw_;
        ros::param::param<double>("~uav_pose_x" ,x_, -8.0);
        ros::param::param<double>("~uav_pose_y" , y_, -8.0);
        ros::param::param<double>("~uav_pose_z" , z_, 1.0);
        ros::param::param<double>("~uav_pose_yaw" , yaw_, 0.0);
    
        vehicle_->setWaypoint(x_, y_, z_, yaw_);
    
        printf("Starting vehicle\n");
        vehicle_->start(x_,y_,z_);
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
    case 1:
      Map_ = new victim_map_Thermal();
      break;
    }
}

void TestNBZ::initNavigation(){
  int nav_type;
    ros::param::param("~nav_type", nav_type, 1);
    switch(nav_type)
    {
    default:
    case 0:
      navigation_ = new straightLine();
      break;
    case 1:
      navigation_ = new ReactivePathPlanner();
      break;
    }
    navigation_->setConfiguration(nh,nh_private,manager_);
    navigation_->start();
}


void TestNBZ::initOctomap(){
  manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);
  Occlusion_Map_ = new Volumetric_Map(manager_);

  std::cout << "here" << std::endl;
  CostMapROS_ = new costmap_2d::Costmap2DROS ("costmap",tf_);
  CostMapROS_->start();
  Occlusion_Map_->SetCostMapRos(CostMapROS_);

  std::cout << "here1" << std::endl;

}


void TestNBZ::initParameters(){

  ros::param::param("~detection_enabled", detection_enabled, false);//for debugging

  View_evaluate_ = new view_evaluator_IG();
  view_generate_ = new view_generator_IG();  //try the adaptive_nn
  history_=  new nbv_history();
  Ray= new RayTracing(view_generate_);
  view_generate_->setHistory(history_);
  view_generate_->setOcclusionMap(Occlusion_Map_);

  //Passing the Mapping and view_generator module to the view_evaluator
  Map_->setRaytracing(Ray);
  view_generate_->setOctomapManager(manager_);
  View_evaluate_->setMappingModule(Map_);
  View_evaluate_->setViewGenerator(view_generate_);
}

void TestNBZ::updateHistory()
{
    history_->selected_poses.push_back(View_evaluate_->getTargetPose());
    history_->selected_utility.push_back(View_evaluate_->info_selected_utility_);
    history_->total_entropy.push_back(View_evaluate_->info_entropy_total_);
    history_->update();

    // Publish information about this iteration
      if (pub_iteration_info.getNumSubscribers() > 0)
      {
        victim_localization::IterationInfo iteration_msg;
        iteration_msg.iteration        = history_->iteration;
        iteration_msg.distance_total   = View_evaluate_->info_distance_total_;
        iteration_msg.entropy_total    = View_evaluate_->info_entropy_total_;

        iteration_msg.method_generation= view_generate_->getMethodName();
        iteration_msg.method_selection = View_evaluate_->getMethodName();
        iteration_msg.selected_pose    = View_evaluate_->getTargetPose();
        iteration_msg.selected_utility                 = View_evaluate_->info_selected_utility_;
        iteration_msg.utilities        = View_evaluate_->info_utilities_;

        iteration_msg.time_iteration   = timer.getLatestTime("[NBVLoop]Iteration");

         pub_iteration_info.publish(iteration_msg);
}
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
  p_ = View_evaluate_->getTargetPose();
  if ( std::isnan(p_.position.x) )
  {
    std::cout << "[test_NBZ] " << cc.red << "View evaluator determined all poses are invalid. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }


  std::cout << "[test_NBZ] " << cc.green << "Done evaluating viewpoints\n" << cc.reset;
  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void TestNBZ::navigate()
{
  if (state != NBVState::NAVIGATION)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to navigate to viewpoint out of order\n" << cc.reset;
    return;
  }

  std::cout << "[test_NBZ] " << cc.green << "Navigating to viewpoint\n" << cc.reset;
  navigation_->setCurrentPose(vehicle_->getPose());

  std::vector<geometry_msgs::Pose> path_;
  navigation_->GeneratePath(p_,path_);
  if (path_.size()==0) {
     printf("WARNING: pose can not be evaluated, Switch Back To GENERATE POINT\n");
     state =NBVState::UPDATE_MAP_COMPLETE;
     return;
  }


   for (int i=0;i<path_.size();i++)
  {
  vehicle_->setWaypoint(path_[i]);
  }
   std::cout << "[test_NBZ] " << cc.green << "Done Navigating to viewpoint\n" << cc.reset;
   state = NBVState::NAVIGATION_COMPLETE;
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
  geometry_msgs::Pose previous_pose=view_generate_->current_pose_;
  view_generate_->generateViews();

  if (view_generate_->generated_poses.size() == 0)
  {
    std::cout << "[test_NBZ] " << cc.red << "View generator created no poses. Terminating.\n" << cc.reset;
    state = NBVState::UPDATE_MAP_COMPLETE;
    vehicle_->setWaypoint(previous_pose);
    vehicle_->moveVehicle(0.7);
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

void TestNBZ::runStateMachine()
{
  ROS_INFO("test_NBZ: Starting vehicle. Waiting for current position information.");
 state = NBVState::STARTING_ROBOT;

 timer.start("NBV: Total Time");
 ros::Rate loop_rate(30);
   while (ros::ok())
   {
     switch(state)
     {
       case NBVState::TERMINATION_MET:
       vehicle_->moveVehicle(1.5);
       break;
       case NBVState::STARTING_ROBOT:

       if (manager_->getMapSize().norm() <= 0.0) break;

         if( vehicle_->isReady() && manager_->getMapSize().norm() > 0.0)
         {
           vehicle_->rotateOnTheSpot();
           state = NBVState::STARTING_ROBOT_COMPLETE;
           break;
         }
         break;

       case (NBVState::STARTING_ROBOT_COMPLETE):
       case(NBVState::STARTING_DETECTION):
       if (detection_enabled) {
         detector_->SetCurrentSetpoint(p_);
         detector_->performDetection();
       }
       state = NBVState::DETECTION_COMPLETE;
       break;

     case (NBVState::DETECTION_COMPLETE):
       timer.start("[NBVLoop]Iteration");
       state = NBVState::UPDATE_MAP;
       UpdateMap();
       if (state!=NBVState::TERMINATION_MET) state = NBVState::UPDATE_MAP_COMPLETE;
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
       state = NBVState::NAVIGATION;
       navigate();
       break;

     case NBVState::NAVIGATION_COMPLETE:
       vehicle_->moveVehicle(1.0);
       updateHistory();
       timer.stop("[NBVLoop]Iteration");
       state = NBVState::STARTING_DETECTION;
       break;
     }
     ros::spinOnce();
     loop_rate.sleep();     
 }
   timer.stop("NBV: Total Time");

   // Dump time data
   timer.dump();


   // Clean up
   std::cout << "[NBV_test] " << cc.yellow << "Shutting down\n" << cc.reset;
   ros::shutdown();
}




int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "test_NBZ");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  //tf::TransformListener tf;
  //tf(ros::Duration(10));

  //Create an object of class test_NBZ

  TestNBZ *test_;
  test_ = new TestNBZ(nh,nh_private);
  test_->initMap();
  test_->initVehicle();
  test_->initOctomap();
  test_->initNavigation();
  test_->initParameters();

  test_->runStateMachine();

  return 0;
}
