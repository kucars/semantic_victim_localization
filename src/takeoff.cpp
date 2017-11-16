/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("iris/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("iris/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("iris/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("iris/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -8;
    pose.pose.position.y = -8;
    pose.pose.position.z = 0.5;


    pose.pose.orientation.x = 0.7071067811865476;
    pose.pose.orientation.y = 0.7071067811865475;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;



/* for face detection P test

 geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -8;
    pose.pose.position.y = -8;
    pose.pose.position.z = 1.8;

    pose.pose.orientation.x = 0.7071067811865476;
    pose.pose.orientation.y = 0.7071067811865475;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0;

*/

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !current_state.armed){
      std::cout << "upupup\n";

        if(current_state.mode != "OFFBOARD"){

          //send a few setpoints before starting
          for(int i = 150; ros::ok() && i > 0; --i){
              local_pos_pub.publish(pose);
              ros::spinOnce();
              rate.sleep();
          }
             set_mode_client.call(offb_set_mode);
    }
        else if( !current_state.armed) arming_client.call(arm_cmd);

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
            }
std::cout << "transition...........................................\n";

     while(ros::ok()){
       local_pos_pub.publish(pose);
       ros::spinOnce();
       rate.sleep();
     }


    return 0;
}
