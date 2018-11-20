/*
 * joy_subscriber.cpp
 *
 *  Created on: Mar 19, 2017
 *      Author: jlurobot
 */




#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <math.h>

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ros::Publisher * pub_ptr = NULL;

void callback_joy(const sensor_msgs::Joy& joy){
	geometry_msgs::Twist msg;
	if(fabs(joy.axes[0]) > fabs(joy.axes[1])){
		if(joy.axes[0] > 0.3){ //A
			msg.angular.z = 1;

		}else if(joy.axes[0] < -0.3){ //D
			msg.angular.z = -1;
		}
	}else if (fabs(joy.axes[0]) < fabs(joy.axes[1])){
		if(joy.axes[1] > 0.3){ //W
			msg.linear.x = 1;
		}else if(joy.axes[1] < -0.3){ //S

			msg.linear.x = -1;
		}
	}
	pub_ptr->publish(msg);
}

int main(int argc, char** argv){
        ros::init(argc, argv, "joy_listener");
        ros::NodeHandle n;
        ros::Subscriber joy_sub = n.subscribe("joy", 1000, callback_joy);
        ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        pub_ptr = &cmd_pub;
        ros::spin();
        return 0;
}


