#include <cstdlib>
#include <stdio.h>

#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

// some important stuff above!

#define RATE 10 
// main function for simple ros application for ArduCopter
// will lift off and land automaticlaly; that is, the user has to enter in use input
// for x, y, and z velocity vectors there.

int main(int argc, char **argv){

    ros::init(argc, argv, "mavros_takeoff");

    ros::NodeHandle nh;
    ros::Rate r(RATE);




    // user input to be able to take off here.
    int input; 
    while(1){
        printf("Please Enter 1 to Takeoff, or 0 to Exit Application:");
        scanf("%d", &input);
        // get the user input as number
        if(!input){
            // 0
            printf("Exiting HAWK Controller....");
            return 0;

        }
        else if(input == 1){
            break;
        }
        printf("Enter a valid option.\n");
    }

    // getting into guided mode....
    ros::ServiceClient scl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // get the client 
    mavros_msgs::SetMode set_mode_service;
    set_mode_service.request.base_mode = 0;

    set_mode_service.request.custom_mode = "GUIDED";
    if(scl.call(set_mode_service)){
        ROS_INFO("changed mode successfully");
        //(set_mode_service.response);
        // do stuff
    }
    else{
        ROS_ERROR("failed to set mode!");
        return -1;
        // do stuff

    }

    ros::ServiceClient arm_scl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    mavros_msgs::CommandBool command_bool_service;
    command_bool_service.request.value = true;
    if(arm_scl.call(command_bool_service)){
        ROS_INFO("armed drone successfully!");
        //ROS_INFO(command_bool_service.response);
    }
    else{
        ROS_ERROR("failed to arm the drone!");
        return -1;

    }
    // hopefully have changed mode to guided mode here

    ros::ServiceClient takeoff_scl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL tol_service;
    tol_service.request.altitude = 20;
    if(takeoff_scl.call(tol_service)){
        ROS_INFO("takeoff command is good!");
        //ROS_INFO(tol_service.response);
    }
    else{
        ROS_ERROR("failed takeoff!");
        return -1;
    }

    int timer;
    // time to maintain setpoint velocity command
    float vx, vy, vz;

    printf("This is a test\n");
    printf("Udana HAWK\n");
    printf("Please enter x velocity: \n");
    scanf("%f", &vx);
    printf("Please enter y velocity: \n");
    scanf("%f", &vy);
    printf("Please enter z velocity: \n");
    scanf("%f", &vz);

    printf("Time to perform the action: \n");
    scanf("%d", &timer);
    // read in all of the values for the vectors that we want!

    // cmd vel message doesn't take into account any gps or location, and will keep going until told to stop.

    ros::Publisher simple_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 5);
    geometry_msgs::TwistStamped twister;
    geometry_msgs::TwistStamped finisher;
    
    twister.header.seq = 1;
    finisher.header.seq = 2;
    twister.header.stamp = ros::Time::now();
    finisher.header.stamp = ros::Time::now();
    twister.header.frame_id = "";
    finisher.header.frame_id = "";

    twister.twist.linear.x = vx;
    twister.twist.linear.y = vy;
    twister.twist.linear.z = vz;

    // message to stabilize drone 
    finisher.twist.linear.x = 0;
    finisher.twist.linear.y = 0;
    finisher.twist.linear.z = 0;

    ros::Time start = ros::Time::now();
    ros::Time end = start + ros::Duration(timer);
    // publish message for certain amount of time
    while(timer){
        simple_vel_pub.publish(twister);
        r.sleep();
        timer--;
        // sleep for one cycle
    }
    ros::Duration(timer).sleep();

    sleep(5);

    /*
    while(ros::Time::now() < end){
        simple_vel_pub.publish(twister);
        // publish the message, yay!
    }
    */

    start = ros::Time::now();
    end = start + ros::Duration(timer);
    simple_vel_pub.publish(finisher);
    /*
    while(ros::Time::now() < end){
        simple_vel_pub.publish(finisher);
    }
    */
    ros::Duration(timer).sleep();
    sleep(5);
    // intermittent sleeps spread throughout everything
    // vectors we want

    // do a bunch of stuff in this area.

    // setpoint 

    // based on user input of a vector, move the drone in a certain position for some amount of seconds.
    // now we can finally land the drone

    /*
            rostopic pub -r 10 /mavros/setpoint_velocity/cmd_vel geometry_msgs/TwistStamped "header:
        seq: 0
        stamp:
            secs: 0
            nsecs: 0
        frame_id: ''
        twist:
        linear:
            x: 0.0
            y: 0.0
            z: 10.0
        angular:
            x: 0.0
            y: 0.0
            z: 0.0"

    */
   // land below



    // get the client 
    mavros_msgs::SetMode set_mode_service_land;
    set_mode_service_land.request.base_mode = 0;

    set_mode_service_land.request.custom_mode = "LAND";
    if(scl.call(set_mode_service_land)){
        ROS_INFO("changed mode successfully. Going to land.");

        // do stuff

    }
    else{
        ROS_ERROR("failed to set mode!");
        return -1;
        // do stuff
    }
    while(nh.ok()){
        ros::spinOnce();
        r.sleep();
    }
    // keep waiting until user desires to end 
    return 0;






}

// options to control the drone in guided mode

// setpoint_velocity/cmd_vel for the velocity commands
// setpoint_raw/global for the gps navigation
// setpoint_raw/local for  