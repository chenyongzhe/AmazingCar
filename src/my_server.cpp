#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"

#include <iostream>
#include <fstream>
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
	//auth
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/gnss.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/algorithm.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/controller.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/ui.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/vlp.sh");
	//main circle
	while(ros::ok()){
		//get cmd
		std::string cmd_str;
	   	my_serial.readline(cmd_str);
		cout<<cmd_str;
		process_cmd(cmd_pub, cmd_str);
		
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

	//#GNSS_OPEN_0$$$$$$$$$$$$$$$$$$

	if(cmd.find("#GNSS_OPEN") == 0){
		//get serial number
		cmd = cmd.substr(cmd.find("N_") + 2);
		cmd = cmd.substr(0, cmd.find("$"));
		int serial_num = atoi(cmd.c_str());
		//write file
		ofstream gnss_cfg_file("/home/jlurobot/catkin_ws/src/amazing_car/config/gnss_serial.cfg");
		gnss_cfg_file << serial_num;
		//start service
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/gnss.sh");
	}
	
	if(cmd.find("#ALGORITHM_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/algorithm.sh");
	}

	amazing_car::my_server_cmd ros_cmd;
	ros_cmd.gnss_cmd = 1;
	ros_cmd.ui_cmd = 1;
	ros_cmd.algorithm_cmd = 1;
	ros_cmd.controller_cmd = 1;
	ros_cmd.vlp16_cmd = 1;
	ros_cmd.beixing_cmd = 1;
	ros_cmd.ins_cmd = 1;

	if(cmd.find("#GNSS_CLOSE") == 0){
		ros_cmd.gnss_cmd = 0;
	}

	if(cmd.find("#UI_CLOSE") == 0){
		ros_cmd.ui_cmd = 0;
	}

	if(cmd.find("#ALGORITHM_CLOSE") == 0){
		ros_cmd.algorithm_cmd = 0;
	}

	if(cmd.find("#CONTROLLER_CLOSE") == 0){
		ros_cmd.controller_cmd = 0;
	}

	if(cmd.find("#CONTROLLER_SHUTDOWN_ON") == 0){
		ros_cmd.controller_cmd = 2;
	}

	if(cmd.find("#CONTROLLER_SHUTDOWN_OFF") == 0){
		ros_cmd.controller_cmd = 3;
	}

	cmd_pub.publish(ros_cmd);
}