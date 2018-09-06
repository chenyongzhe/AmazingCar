#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "serial/serial.h"

using namespace std;

serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_server");
	ros::NodeHandle n;
	ros::Publisher location_pub = n.advertise<amazing_car::my_server_cmd>("server_cmd", 1000);
	ros::Rate rate(20);
	while(ros::ok()){
		//get cmd
		std::string test_string;
	   	my_serial.readline(test_string);
		cout<<test_string;
		//pub cmd
		amazing_car::my_server_cmd cmd;
		cmd.gnss_cmd = 2;
		cmd.ui_cmd = 2;
		cmd.algorithm_cmd = 2;
		cmd.controller_cmd = 2;
		cmd.vlp16_cmd = 2;
		cmd.beixing_cmd = 2;
		cmd.ins_cmd = 2;
		location_pub.publish(cmd);
		//circle
		usleep(40000);
		ros::spinOnce();
	}
	return 0;
}
