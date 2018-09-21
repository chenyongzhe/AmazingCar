#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "amazing_car/my_server_cmd.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include <fstream>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using namespace std;

int shutdown_cmd = 1;

void callback_server(const amazing_car::my_server_cmd cmd){
    if(cmd.controller_cmd == 0){
        shutdown_cmd = 0;
    }
}


void callback(const geometry_msgs::Twist& cmd_vel){
	float left = cmd_vel.linear.x;
	float right = cmd_vel.linear.y;
	if(left == 50 && right == 350){

	}else if(left == 350 && right == 50){

	}else if(left == 350 && right == 350){

	}else{  

	}
}

std::string get_res(){
	char t[20];
	sprintf(t,"MAS %d %d\r\n",speed,direction);
	std::string res = t;
	return res;
}

int main(int argc, char ** argv){

	std::ifstream controller_cfg("/home/jlurobot/catkin_ws/src/amazing_car/config/controller.cfg");
    int serial_num = 111;
    controller_cfg >> serial_num;
    char serial_num_str[20];
    memset(serial_num_str, 0, 20);
    sprintf(serial_num_str, "/dev/ttyUSB%d", serial_num);
    serial::Serial my_serial(serial_num_str, 57600, serial::Timeout::simpleTimeout(1000));

	ros::init(argc, argv, "my_big_car_controller");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Rate rate(20);
	while(ros::ok()){
		if(shutdown_cmd == 0){
			break;
		}
		std::string test_string = get_res();
	   	my_serial.write(test_string);
		cout<<test_string;
		usleep(40000);
		ros::spinOnce();
	}
	return 0;
}
