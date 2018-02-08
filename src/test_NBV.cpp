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
  nh_private(nh_private_),
  NBV_loop_rate(20)
{
    pub_iteration_info = nh.advertise<victim_localization::IterationInfo>("victim_localization/iteration_info", 10);
    // >>>>>>>>>>>>>>>>>
    // Initialization
    // >>>>>>>>>>>>>>>>>
    state = NBVState::INITIALIZING;

    visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/Final_path"));
    visualTools->loadMarkerPub();

    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();

    ROS_INFO("test_NBZ: Begin!");
}

void TestNBZ::initVehicle(){
  int vehicle_type;
  ros::param::param<int>("~vehicle_type",vehicle_type,0);
  switch(vehicle_type)
  {
  default:
  case 0:
    vehicle_ = new VehicleControlIris();
    break;
  case 1:
    vehicle_ = new VehicleControlIrisOrigin();
    break;
  case 2:
    vehicle_ = new VehicleControlFloatingSensor();
    break;
   }
}

void TestNBZ::initMap(){
    ros::param::param("~map_type", map_type, 0);
    switch(map_type)
    {
    default:
    case 0:
      Map_ = new victim_map_DL(nh,nh_private);
      break;
    case 1:
      Map_ = new victim_map_Thermal(nh,nh_private);
      break;
    case 2:
      Map_ = new Victim_Map_Wireless(nh,nh_private);
      break;
    case 3:
      Map_ = new victim_map_combined(nh,nh_private);
      break;
    }
}

void TestNBZ::initNavigation(){

    ros::param::param("~nav_type", nav_type, 1);
    switch(nav_type)
    {
    default:
    case 0:
     // navigation_ = new straightLine(nh,nh_private,manager_);
     // break;
    case 1:
      navigation_ = new ReactivePathPlanner(nh,nh_private,manager_);
      break;
    }
    navigation_->start();
}

void TestNBZ::initViewGenerator(){

    ros::param::param("~view_generator_type", view_generator_type, 0);

    switch(view_generator_type)
    {
    default:
    case 0:
      view_generate_ = new view_generator_IG();
      break;
    case 1:
      view_generate_ = new view_generator_ig_nn_adaptive();
      break;
    case 2:
      view_generate_ = new view_generator_ig_frontier();
      break;
    case 3:
      view_generate_ = new view_generator_ig_adaptive_frontier();
    break;
    }
}

void TestNBZ::initViewEvaluator(){

    ros::param::param("~view_evaluator_type", view_evaluator_type, 5);

    switch(view_evaluator_type)
    {
    default:
    case 0:
      View_evaluate_ = new view_evaluator_ig();
      break;
    case 1:
      View_evaluate_ = new  view_evaluator_MaxSUM();
      break;
    case 2:
      View_evaluate_ = new view_evaluator_MaxMax();
      break;
    case 3:
      View_evaluate_ = new view_evaluator_MaxMIN();
    case 4:
      View_evaluate_ = new view_evaluator_MinNEIGH();
    break;
    case 5:
      View_evaluate_ = new view_evaluator_log_reward();
    break;
    case 6:
      View_evaluate_ = new view_evaluator_ig_exp();
    break;
    case 7:
      View_evaluate_ = new view_evaluator_ig_exp_max();
    break;
    }
}

void TestNBZ::initOctomap(){

  manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);

  Occlusion_Map_ = new Volumetric_Map(manager_);

  CostMapROS_ = new costmap_2d::Costmap2DROS ("costmap",tf_);

  CostMapROS_->start();
  Occlusion_Map_->SetCostMapRos(CostMapROS_);
}


void TestNBZ::initParameters(){

  ros::param::param("~detection_enabled", detection_enabled, false);//for debugging

  history_=  new nbv_history();
  view_generate_->setHistory(history_);
  view_generate_->setOcclusionMap(Occlusion_Map_);
  view_generate_->setCostMapROS(CostMapROS_);

  //Passing the Mapping and view_generator module to the view_evaluator
  view_generate_->setOctomapManager(manager_);
  View_evaluate_->setMappingModule(Map_);
  View_evaluate_->setViewGenerator(view_generate_);

  // initialize vehicle communicator
  drone_communicator_ = new vehicle_communicator(nh,nh_private,manager_);

  Map_->setDroneCommunicator(drone_communicator_);


  Map_->SetNavMap(Occlusion_Map_->m_gridmap);

  Map_->setOctomapManager(manager_);
}

void TestNBZ::updateHistory()
{
    history_->selected_poses.push_back(View_evaluate_->getTargetPose());
    history_->selected_utility.push_back(View_evaluate_->info_selected_utility_);
    std::cout << "best utitlity is ...." << View_evaluate_->info_selected_utility_;
    history_->total_entropy.push_back(View_evaluate_->info_entropy_total_);
    std::cout << "totla utitliy is ...." << View_evaluate_->info_entropy_total_;

    history_->max_prob=Map_->curr_max_prob;

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
        iteration_msg.selected_utility  =View_evaluate_->info_selected_utility_;
        iteration_msg.curr_max_prob  = Map_->curr_max_prob;
        iteration_msg.curr_max_loc_x  =Map_->curr_max_loc[0];
        iteration_msg.curr_max_loc_y  =Map_->curr_max_loc[1];

        iteration_msg.generator_type =view_generate_->generator_type;

        //iteration_msg.utilities        = View_evaluate_->info_utilities_;
        iteration_msg.time_iteration   = timer.getLatestTime("[NBV)Loop]Iteration")/1000; // time in sec

        if (Map_->Maptype==MAP::COMBINED){
          iteration_msg.entropy_total_dl= View_evaluate_->info_dl_entropy_total_;
          iteration_msg.entropy_total_thermal = View_evaluate_->info_thermal_entropy_total_;
          iteration_msg.entropy_total_wireless = View_evaluate_->info_wireless_entropy_total_;

          iteration_msg.selected_utility_dl= View_evaluate_->info_dl_selected_utility_;
          iteration_msg.selected_utility_thermal = View_evaluate_->info_thermal_selected_utility_;
          iteration_msg.selected_utility_wireless = View_evaluate_->info_wireless_selected_utility_;

        }
        else
          {
              iteration_msg.entropy_total_dl=0;
              iteration_msg.entropy_total_thermal=0;
              iteration_msg.entropy_total_wireless=0;
              iteration_msg.selected_utility_dl=0;
              iteration_msg.selected_utility_thermal=0;
              iteration_msg.selected_utility_wireless=0;
          }

        pub_iteration_info.publish(iteration_msg);
        Occlusion_Map_->Convert2DMaptoOccupancyGrid(ros::Time::now());
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
  std::string navigation_method;
  if (view_generate_->nav_type==0)
    navigation_method="Straight Line";

  else
    navigation_method="Reactive Planner";


  if (state != NBVState::NAVIGATION)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to navigate to viewpoint out of order\n" << cc.reset;
    return;
  }

  std::cout << "[test_NBZ] " << cc.green << "Navigating to viewpoint using" << "[" << navigation_method << "]" << cc.reset;

  path_to_waypoint.clear();
  navigation_->setCurrentPose(drone_communicator_->GetPose());

  switch(view_generate_->nav_type)
  {
  default:
  case 2: // move to waypoint if discrete strigntline is used
    path_to_waypoint = navigation_->Path_discretizationtoPath(drone_communicator_->GetPose(),View_evaluate_->getTargetPose(),
                                                  0.02);

    std::cout << "desired pose is..." << View_evaluate_->getTargetPose() << std::endl;
    std::cout << "generated path is of size ....." << path_to_waypoint.size() << std::endl;
    for (int i=0; i<path_to_waypoint.size() ; i++)
    std::cout << path_to_waypoint[i]  << std::endl;

    if (!drone_communicator_->Execute_path(path_to_waypoint))
    {
        ROS_WARN("Drone Communicator is unable to set path to waypoint, terminating....");
    }
    break;


  case 1:  // move through path if reactive planner is used

           // initially check for a straight line.

     if (!view_generate_->isCollide(View_evaluate_->getTargetPose()))
     {
         if (!drone_communicator_->Execute_waypoint(View_evaluate_->getTargetPose()))
         {
             ROS_WARN("Drone Communicator is unable to set waypoint, terminating....");
         }
       history_->black_listed_poses.clear();
       break;
     }

     // if straight line did not work then try the reactive planner

    Occlusion_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
    navigation_->SetDynamicGridSize(grid_size_x,grid_size_y,0);

    Occlusion_Map_->GetActiveOrigin(grid_origin_x,grid_origin_y);
    navigation_->SetOriginPose(grid_origin_x,grid_origin_y,round(drone_communicator_->GetPose().position.z));

    if (navigation_->GeneratePath(View_evaluate_->getTargetPose(),path_to_waypoint)) {
      printf("path Found...\n");
      drone_communicator_->Execute_path(path_to_waypoint);
      history_->black_listed_poses.clear();
    }

    else {
      ROS_WARN("Unable to generate path, terminating....");
      if(view_generate_->generator_type==Generator::Frontier_Generator)
        history_->black_listed_poses.push_back(View_evaluate_->getTargetPose());
       }

    view_generate_->visualTools->deleteAllMarkers();
    view_generate_->visualTools->trigger();
    break;


  case 0:  // move through fixed setpoint

    if (!drone_communicator_->Execute_waypoint(View_evaluate_->getTargetPose()))
    {
        ROS_WARN("Drone Communicator is unable to set waypoint, terminating....");
    }

  break;
  }

 // ros::Time wait=ros::Time::now();
 // if (!drone_communicator_->GetStatus())
 // {
  std::cout << "[test_NBZ] " << cc.green << "Waiting for Vehicle to reach Viewpoint\n" << cc.reset;
  // wait for the drone commander node until it moves the drone to the viewpoint
// while ((ros::ok() && !drone_communicator_->GetStatus()) || ((ros::Time::now()-wait).toSec()<1.0))
  while ((ros::ok() && !drone_communicator_->GetStatus()))
   {
   ros::spinOnce();
   ros::Rate(5).sleep();
   }
  // }

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

  navigation_->reactivePlannerServer->visualTools->deleteAllMarkers();
  navigation_->reactivePlannerServer->visualTools->trigger();

  view_generate_->setCurrentPose(drone_communicator_->GetPose());
  view_generate_->setvictimmap(Map_->map,Map_->getlayer_name());

 // std::cout << "curretn pose: " << drone_communicator_->GetPose() <<" "<< std::endl;
  view_generate_->generateViews();

  if (view_generate_->generated_poses.size() == 0)
  {
    std::cout << "[test_NBZ] " << cc.red << "View generator created no poses. Terminating.\n" << cc.reset;
    state = NBVState::UPDATE_MAP_COMPLETE;
  }
  else
  {
    state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
  }
}


void TestNBZ::UpdateMap()
{
  if (state != NBVState::START_MAP_UPDATE)
  {
    std::cout << "[test_NBZ] " << cc.red << "ERROR: Attempt to update map out of order\n" << cc.reset;
    return;
  }

   std::cout << "[test_NBZ] " << cc.green << "Updateing [" << Map_->getlayer_name() << "]" << cc.reset;

  Map_->setCurrentPose(drone_communicator_->GetPose());

  Map_->Update();

  Occlusion_Map_->Convert2DMaptoOccupancyGrid(ros::Time::now());

  if (Map_->getMapResultStatus().victim_found)
  {
    std::cout << cc.magenta << "Victim Found at Location: " << "(x,y)=" << "(" <<
                 (Map_->getMapResultStatus().victim_loc)[0] << "," <<
                 (Map_->getMapResultStatus().victim_loc)[1] << ") terminating...\n" << cc.reset;

    std::cout << "time taken: " << (t_start-ros::Time::now()).toSec() << std::endl;
    std::cout << "entropy: " << View_evaluate_->info_entropy_total_ << std::endl;
    std::cout << "distance: " <<  View_evaluate_->info_distance_total_ << std::endl;
    std::cout << "iteration: " << history_->iteration<< std::endl;

    state = NBVState::TERMINATION_MET;
    return;
  }
    state = NBVState::UPDATE_MAP_COMPLETE;
}

void TestNBZ::runStateMachine()
{
  ROS_INFO("test_NBZ: Starting victim_localization node waiting for the drone to take off and rotate.");
 state = NBVState::STARTING_ROBOT;

 timer.start("NBV: Total Time");
 t_start = ros::Time::now();
   while (ros::ok())
   {
     switch(state)
     {   
     case NBVState::STARTING_ROBOT:
     if (!drone_communicator_->GetStatus()){
       break; // check if the drone is ready from drone commander node
     }
     Occlusion_Map_->Stop();
     std::cout << "drone is ready\n";
     state = NBVState::START_MAP_UPDATE;
       break;

       case NBVState::START_MAP_UPDATE:
       timer.start("[NBVLoop]Iteration");  // detect and update map
       ros::Rate(1).sleep();
       Occlusion_Map_->GetPointCloud();
       while (!Occlusion_Map_->GetPointCloudDone()) {ros::spinOnce() ; NBV_loop_rate.sleep();}
       UpdateMap();
       //if (state!=NBVState::TERMINATION_MET) state = NBVState::UPDATE_MAP_COMPLETE;
       break;

       case NBVState::UPDATE_MAP_COMPLETE:
         state = NBVState::VIEWPOINT_GENERATION ;
         generateViewpoints();
       break;

     case NBVState::VIEWPOINT_GENERATION_COMPLETE:
       state = NBVState::VIEWPOINT_EVALUATION;
       T1 = ros::Time::now();
       evaluateViewpoints();
        D= ros::Time::now()-T1;
       std::cout << "time taken for Viewpoint evaluator is..." << D.toSec() <<std::endl;     break;

     break;

     case NBVState::VIEWPOINT_EVALUATION_COMPLETE:
       state = NBVState::NAVIGATION;
       navigate();
       break;

     case NBVState::NAVIGATION_COMPLETE:
       updateHistory();
       timer.stop("[NBVLoop]Iteration");
       state = NBVState::START_MAP_UPDATE;
       break;

     case NBVState::TERMINATION_MET:

       for(int i =0; i< (history_->selected_poses.size()- 1) ;i+=1)
       {
         visualTools->publishLine(history_->selected_poses[i].position,history_->selected_poses[i+1].position, rviz_visual_tools::BLUE,rviz_visual_tools::XLARGE);
       }
       for(int i =0; i< (history_->selected_poses.size()- 1) ;i+=1)
       {
         visualTools->publishArrow(history_->selected_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
       }
       visualTools->trigger();
       Map_->publish_Map();

       ros::spinOnce();
       ros::Rate(5).sleep();
     break;
     }
     ros::spinOnce();
     NBV_loop_rate.sleep();
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
  test_->initVehicle();
  test_->initMap();
  test_->initOctomap();
  test_->initNavigation();
  test_->initViewGenerator();
  test_->initViewEvaluator();
  test_->initParameters();

  test_->runStateMachine();
  return 0;
}
