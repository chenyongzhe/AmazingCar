#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "serial/serial.h"

using namespace std;

serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));

void process_cmd(const ros::Publisher & cmd_pub, string cmd);

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_server");
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<amazing_car::my_server_cmd>("server_cmd", 1000);
	ros::Rate rate(20);
	while(ros::ok()){
		//get cmd
		std::string cmd_str;
	   	my_serial.readline(cmd_str);
		cout<<cmd_str;
		process_cmd(cmd_pub, cmd_str);
		//pub cmd
		
		//circle
		usleep(40000);
		ros::spinOnce();
	}
	return 0;
}


void process_cmd(const ros::Publisher & cmd_pub, string cmd){
	if(cmd.size() == 0){
		return;
	}else{
		cmd = cmd.substr(0, cmd.size() - 1);
	}

	if(cmd.size() != 30){
		printf("cmd_length:%d\n", cmd.size());
		return;
	}

	if(cmd.find("#GNSS_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/gnss.sh");
		return;
	}
	
	if(cmd.find("#ALGORITHM_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/algorithm.sh");
		return;
	}


	amazing_car::my_server_cmd ros_cmd;
	ros_cmd.gnss_cmd = 2;
	ros_cmd.ui_cmd = 2;
	ros_cmd.algorithm_cmd = 2;
	ros_cmd.controller_cmd = 2;
	ros_cmd.vlp16_cmd = 2;
	ros_cmd.beixing_cmd = 2;
	ros_cmd.ins_cmd = 2;
	cmd_pub.publish(ros_cmd);
}