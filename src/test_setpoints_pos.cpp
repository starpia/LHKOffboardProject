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
#include <mavros_msgs/Thrust.h>
#include "math.h"
//#include <iostream>
//#include <string>

/*
double r;
double theta;
double count=0.0;
double wn;
*/

double count=0.0;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher thr_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

	//nh.param("pub_setpoints_traj/wn", wn, 1.0);
	//nh.param("pub_setpoints_traj/r", r, 1.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::Thrust thr_cmd;
    thr_cmd.thrust = 0.2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        thr_pub.publish(thr_cmd);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // set custom mode OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // auto arming 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    
    double altitude = 0.0;
    bool isPeak = false;
    bool isLand = true;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(current_state.mode == "OFFBOARD" && 
           current_state.armed &&
           !isPeak)
        {
            ros::Duration dt = ros::Time::now() - last_request;
            if(ros::Duration(5.0) < dt && dt < ros::Duration(10.0))
            {
                altitude = 1.5; 
                isLand = false;
                //last_request = ros::Time::now();
            }
            else if(dt > ros::Duration(15.0))
            {
                isPeak = true;
                last_request = ros::Time::now();
            } 

            //local_pos_pub.publish(pose);

        }
        else if(current_state.mode == "OFFBOARD" &&
                current_state.armed &&
                isPeak && !isLand)
        {
            ros::Duration dt = ros::Time::now() - last_request;
            if(dt > ros::Duration(0.50))
            {
                altitude = altitude - 0.05;
                if(altitude <= 0.0)
                {
                    isLand = true;
                    //isPeak = false;
                    altitude = 0.0;
                    // Disarm 
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);
                }
                last_request = ros::Time::now();
            }
        }

     	pose.pose.position.x = 0;
    	pose.pose.position.y = 0;
    	pose.pose.position.z = altitude;

        thr_pub.publish(thr_cmd);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        //ros::Duration(5.0).sleep();
    }

    return 0;
}
