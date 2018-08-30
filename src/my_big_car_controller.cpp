#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using namespace std;

serial::Serial my_serial("/dev/ttyUSB1", 57600, serial::Timeout::simpleTimeout(1000));

int direction = 50;
int speed = 50;

void callback(const geometry_msgs::Twist& cmd_vel){
	float left = cmd_vel.linear.x;
	float right = cmd_vel.linear.y;
	if(left == 50 && right == 350){
		direction = 43;
		speed = 50;
	}else if(left == 350 && right == 50){
		direction = 57;
		speed = 50;
	}else if(left == 350 && right == 350){
		direction = 48;
		speed = 35;
	}else{  
		direction = 50;
		speed = 50;
	}

	/*
	if(left == right && left == 200){
		direction = 50;
		speed = 50;
	}else if(fabs(left - right) <= 40 && (left + right > 400)){
		direction = 50;
		speed = 40;
	}else if(fabs(left - right) <= 40 && (left + right < 400)){
		direction = 50;
		speed = 60;
	}else if(left - right > 40){ // turn right
		direction = 42;
		speed = 40;
	}else if(left - right < -40){ // turn left
		direction = 58;
		speed = 40;
	}else{
		direction = 50;
		speed = 50;
	}
	*/
}

std::string get_res(){
	char t[20];
	sprintf(t,"MAS %d %d\r\n",speed,direction);
	std::string res = t;
	return res;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_big_car_controller");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	ros::Rate rate(20);
	while(ros::ok()){
		std::string test_string = get_res();
	   	my_serial.write(test_string);
		cout<<test_string;
		usleep(40000);
		ros::spinOnce();
	}
	return 0;
}
