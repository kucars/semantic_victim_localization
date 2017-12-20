cmake_minimum_required(VERSION 2.8.3)
project(victim_localization)

SET ( CMAKE_CXX_FLAGS "-frounding-math" )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cmake_modules
    eigen_conversions
    cv_bridge
    image_transport
    message_generation
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    tf
    tf_conversions
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    grid_map_octomap
    grid_map_core
    octomap_world
    costmap_2d
    sspp
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL QUIET COMPONENTS)
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(
  FILES
  DL_msgs_box.msg
  DL_msgs_boxes.msg
)

add_service_files(
  FILES
  DL_box.srv
  rotate_action.srv
  waypoint_action.srv
  path_action.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs nav_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf sspp
  DEPENDS Eigen
  #LIBRARIES Victim_Localization
  #CATKIN_DEPENDS roscpp rospy
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include #Adds my own custom class headers
    include/vicitm_localization
    include/control
    include/keras
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${OCTOMAP_LIBRARIES})


## add executable

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${catkin_LIBRARIES})

add_executable(thermalVictim src/victim_thermal_detector_node.cpp )
add_dependencies(thermalVictim  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(thermalVictim ${catkin_LIBRARIES})

add_executable (navigation_test
    src/test_navigation.cpp

    src/control/vehicle_control_base.cpp
    src/control/vehicle_control_iris.cpp
    src/control/drone_communicator.cpp
    src/volumetric_map_manager.cpp
    src/raytracing.cpp

    src/navigation_base.cpp
    src/straightline.cpp
    src/reactive_path_planner.cpp
    src/reactive_planner_server.cpp
    )
add_dependencies(navigation_test ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigation_test
    ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CGAL_LIBRARY}
    ${CGAL_Core_LIBRARY}
    ${rviz_visual_tools_LIBRARIES}
    SSPathPlanner
)

add_executable (iris_commander
    src/control/iris_drone_commander.cpp
    src/control/drone_communicator.cpp

    src/control/vehicle_control_base.cpp
    src/control/vehicle_control_iris.cpp
    )

add_dependencies(iris_commander ${catkin_EXPORTED_TARGETS})

target_link_libraries(iris_commander
    ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

#add_executable(pointc src/Volumetric_Map_Manager.cpp )
#add_dependencies(pointc  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pointc ${catkin_LIBRARIES})

#add_executable(ff src/takeoff.cpp)
#target_link_libraries(takeoff ${catkin_LIBRARIES})



#add_executable(SSD_client src/SSD_client.cpp)
#target_link_libraries(SSD_client ${catkin_LIBRARIES} )
#add_dependencies(SSD_client ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#add_executable (cluster_ src/ssd_detection_with_ecludian_clustering.cpp)
#add_dependencies(cluster_  ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries (cluster_ ${PCL_LIBRARIES} ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Victim_Localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(signal_ src/signal_map.cpp)
#target_link_libraries(signal_ ${catkin_LIBRARIES} )
