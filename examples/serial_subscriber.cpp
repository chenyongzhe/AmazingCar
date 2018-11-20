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

serial::Serial my_serial("/dev/ttyUSB0", 19200, serial::Timeout::simpleTimeout(1000));
      
//ros::Publisher * pub_ptr = NULL;

void callback(const geometry_msgs::Twist& cmd_vel){  
	ROS_INFO("Received a /cmd_vel message!");  
  	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);  
     	ROS_INFO("Angular Component:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);  
     	int forward=cmd_vel.linear.x;
     	int swerve=cmd_vel.linear.z;
	string test_string;
	if(cmd_vel.linear.x > 0){
		test_string = "";
   		test_string = "w";
	}
	if(cmd_vel.linear.x < 0){
		test_string = "";
   		test_string = "s";
	}
	if(cmd_vel.angular.z > 0){
		test_string = "";
   		test_string = "a";
	}
	if(cmd_vel.angular.z < 0){
		test_string = "";
   		test_string = "d";
	}
	size_t bytes_wrote = my_serial.write(test_string);
}

/*
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
*/

int main(int argc, char** argv){
        ros::init(argc, argv, "cmd_vel_listener");  
        ros::NodeHandle n;  
        ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback); 
        //ros::Subscriber joy_sub = n.subscribe("joy", 1000, callback_joy);
        //ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        //pub_ptr = &cmd_pub;
        ros::spin();
        return 0;  
}  


