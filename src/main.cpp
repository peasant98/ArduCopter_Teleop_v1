#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>

#include <termios.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/ParamSet.h"
// v2 of the udana hawk flight controller

// allows the user to realtime control the drone via simple keyboard inputs
#define RATE 5

#define UP_Y 119
#define DOWN_Y 115
#define UP_X 100
#define DOWN_X 97
#define UP_Z 111
#define DOWN_Z 108
#define EXIT 120


// another thread for time

int timer;
// velocity vectors, as well as time
bool init = false;
double difference = 0.0f;
double starting_altitude = 0.0f;
void control_menu();
void leave(int sig);
int getch();

void stateCallback(const sensor_msgs::Imu::ConstPtr& msg){
    auto s = msg->linear_acceleration.z;
    //ROS_INFO("The linear acceleration is now: [%f]", s);
    // do stuff here

}


void heightCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if(!init){
        starting_altitude = msg->altitude;
        init = true;

    }
    else{
        difference  = msg->altitude - starting_altitude;
    }
    ROS_INFO("%lf is height", difference);
}



int main(int argc, char**argv){
    ros::init(argc, argv, "mavros_control");
    // steps for navio2 - set stream rate, and have override for everything there.

    ros::NodeHandle nh;
    ros::Rate r(RATE);
    signal(SIGINT,leave);
    // get the subscriber here to call the callback
    ros::Subscriber sb = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 100, stateCallback);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1, heightCallback);

    ros::ServiceClient stream_cl = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    ros::ServiceClient param_cl = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

    mavros_msgs::StreamRate stream_req;
    stream_req.request.stream_id = 0;
    stream_req.request.message_rate = 10;
    stream_req.request.on_off = 1;

    mavros_msgs::ParamSet param_set_req;
    mavros_msgs::ParamValue result;
    result.real = 0.0;
    result.integer = 16318;
    param_set_req.request.param_id = "ARMING_CHECK";
    param_set_req.request.value = result;


    if(stream_cl.call(stream_req) && param_cl.call(param_set_req)){
        ROS_INFO("Correct stream rate set and arm checks fixed.");
        //(set_mode_service.response);
        // do stuff
    }
    else{
        ROS_ERROR("Could not correct stream rate and/or change arm check.");
        return -1;
        // do stuff

    }
    
    // wait here some amount of time before arming and switching the modes
    // of the arducopter, need to pass all test cases before we can arm

    ros::Duration(20).sleep();
    // 20 seconds pause...
    int input;
    double height;

    while(1){
        printf("Please enter 1 to takeoff in guided mode, or 0 to exit.");
        printf("Everything else does nothing.\n");
        scanf("%d", &input);
        if(!input){
            printf("Exiting HAWK Controller v 2.0...");
            return 0;
        }
        else if(input == 1){
            getchar();
            break;
        }
        printf("Please enter a valid option.\n");
        getchar();
    }


    while(1){
        printf("Enter height of takeoff: ");
        scanf("%lf", &height);
        if(height>=0){
            getchar();
            break;
        }
        getchar();

    }
    
    // get desired height of takeoff.
    // getting into guided mode....
    // checks?

    ros::ServiceClient scl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // get the client // getting into guided mode....
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

    ros::Duration(2).sleep();

    // arm the drone, assuming that the pre arm checks passed.
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

    ros::Duration(2).sleep();
    // once in guided mode, take the previous user input
    // to get to the desired height

    ros::ServiceClient takeoff_scl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL tol_service;
    tol_service.request.altitude = height;
    if(takeoff_scl.call(tol_service)){
        ROS_INFO("takeoff command is good!");
        //ROS_INFO(tol_service.response);
    }
    else{
        ROS_ERROR("failed takeoff!");
        return -1;
    }

    ros::spinOnce();
    bool v = false;
    auto current_time = ros::Time::now();
    // polling for heights of the drone.
    // max is 25 seconds
    while(ros::Time::now() - current_time < ros::Duration(25)){
        ros::spinOnce();
        if(abs(height - difference) <= 0.125){
            // within acceptable height in the z axis.
            ROS_INFO("At the desired setpoint!");
            // exiting now that we are at the correct height

            ros::Duration(2).sleep();
            break;

        }
        r.sleep();
    }


    // take time to get to the stop.
    
    control_menu();

    ros::Publisher simple_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 50);
    geometry_msgs::TwistStamped twister;
    twister.header.stamp = ros::Time::now();
    // source code shows that the above line, if set to 0 - will break.
    twister.header.seq = 1;
    twister.header.frame_id = "";

    // let's see how queue size of 50 works....
    twister.twist.linear.x = 0;
    twister.twist.linear.y = 0;
    twister.twist.linear.z = 0;


    while(1){
        // show the menu
        int x = getch();
        // uncomment to see what is printed
        // putchar(x);
        
        if(x == EXIT){
            twister.twist.linear.x = 0;
            twister.twist.linear.y = 0;
            twister.twist.linear.z = 0;
            simple_vel_pub.publish(twister);
            ros::Duration(1).sleep();
            break;
        }
        switch(x){
            case UP_Y:
                //printf("y+");
                twister.twist.linear.y+=0.5;
                // increase y velocity
                break;
            case DOWN_Y:
                //printf("y-");
                twister.twist.linear.y-=0.5;
                // decrease y velocity
                break;
            case UP_X:
                //printf("x+");
                twister.twist.linear.x+=0.5;
                // increase x velocity
                break;
            case DOWN_X:
                //printf("x-");
                twister.twist.linear.x-=0.5;
                // decrease x velocity
                break;
            case UP_Z:
                //printf("z+");
                twister.twist.linear.z+=0.5;
                // increase z velocity
                break;
            case DOWN_Z:
                //printf("z-");
                twister.twist.linear.z-=0.5;
                // decrease z velocity
                break;
        }
        simple_vel_pub.publish(twister);
        ros::spinOnce();
        r.sleep();
        // get the user input
    }

    ROS_INFO("Udana HAWK about to land...");
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

    ros::Duration(10).sleep();


    auto _cur_time = ros::Time::now();
    command_bool_service.request.value = false;
  
    while(ros::Time::now() - _cur_time < ros::Duration(12)){

        if(arm_scl.call(command_bool_service)){
            ROS_INFO("Disarmed the Udana HAWK!");
            break;
        }
        ros::Duration(1).sleep();
    }
    while(nh.ok()){
        ros::spinOnce();
        r.sleep();
    }
    // keep waiting until user desires to end 
    return 0;





    // once the drone is at its spot
    // we'll need to use some kind of subscriber to check a drone's location here:

    // subscriber code:


    // now, drone is flying in the air and we can give it commands as we please.
}

void control_menu(){
    printf("Udana HAWK Main Controls:\n");
    printf("w - increase y velocity\n");
    printf("s - decrease y veloity\n");
    printf("d - increase x velocity\n");
    printf("a - decrease x velocity\n");
    printf("o - increase z velocity\n");
    printf("l - decrease z velocity\n");
    printf("x - go into landing mode, and end the mission. \n");
    printf("Enter: ");
}




int getch(void) {
      int c=0;

      struct termios org_opts, new_opts;
      int res=0;
          //-----  store old settings -----------
      res=tcgetattr(STDIN_FILENO, &org_opts);
      assert(res==0);
          //---- set new terminal parms --------
      memcpy(&new_opts, &org_opts, sizeof(new_opts));
      new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
      tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
      c= getchar();
          //------  restore old settings ---------
      res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
      assert(res==0);
      return(c);
}



void leave(int sig)
{
    printf("Shutting down node...");
    exit(0);
}
