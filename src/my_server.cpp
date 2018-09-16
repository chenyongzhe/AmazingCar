#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"
#include "amazing_car/my_checkpoints.h"
#include "amazing_car/my_car_state.h"

#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include "serial/serial.h"
#include <thread>

using namespace std;

struct CYCarPoint{
	CYCarPoint(float x, float y){
		this->x = x;
		this->y = y;
	}
	float x;
	float y;	
};

int ordered_cmd_count = 0;
int recieved_cmd_count = 0;
int cmd_flag[100] = {0};
ros::Publisher * p_tars_pub = nullptr;
vector<CYCarPoint> tars;
serial::Serial * p_my_serial = nullptr;
ros::Publisher * p_cmd_pub = nullptr;

void callback_state(const amazing_car::my_car_state state);
void SendCarData(float x,float y,float angle,int state);
void process_cmd(const ros::Publisher & cmd_pub, string cmd);
void init_tars_queue(vector<CYCarPoint> & tar_vector, string cmd);
void add_tar_to_queue(vector<CYCarPoint> & tar_vector, string cmd);

void gjm_data_thread(int);


int temp_test = 0;

void gjm_cmd_thread(int){
	//get cmd		
	while(true){
		std::string cmd_str;
		p_my_serial->readline(cmd_str);
		cout<<cmd_str;
		process_cmd(*p_cmd_pub, cmd_str);
	}	
}

int main(int argc, char ** argv){
	char temp[50];
	memset(temp, 0, 50);
	int serial_number = atoi(argv[1]);
	sprintf(temp, "/dev/ttyUSB%d", serial_number);
	serial::Serial my_serial(temp, 115200, serial::Timeout::simpleTimeout(1000));
	p_my_serial = &my_serial;
	ros::init(argc, argv, "my_server");
	ros::NodeHandle n;
	ros::Publisher cmd_pub = n.advertise<amazing_car::my_server_cmd>("server_cmd", 1000);
	p_cmd_pub = &cmd_pub;
	ros::Publisher tars_pub = n.advertise<amazing_car::my_checkpoints>("my_checkpoints", 1000);
	p_tars_pub = &tars_pub;
	ros::Subscriber car_state_sub = n.subscribe("my_car_state", 1000, callback_state);
	ros::Rate rate(50);
	//auth
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/gnss.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/algorithm.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/controller.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/ui.sh");
	system("chmod 777 /home/jlurobot/catkin_ws/src/amazing_car/shell/vlp.sh");
	auto t = thread(gjm_cmd_thread,0);
	//main circle
	while(ros::ok()){
		//circle
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

void callback_state(const amazing_car::my_car_state state){
	SendCarData(state.x, state.y, state.angle, state.state);
}

void process_cmd(const ros::Publisher & cmd_pub, string cmd){
	if(cmd.size() == 0){
		return;
	}else{
		cmd = cmd.substr(0, cmd.size() - 1);
	}

	if(cmd.size() != 50){
		printf("cmd_length:%d\n", cmd.size());
		return;
	}

	//#GNSS_OPEN_0$$$$$$$$$$$$$$$$$$

	if(cmd.find("#GNSS_OPEN") == 0){
		//get serial number
		cmd = cmd.substr(cmd.find("OPEN_") + 5);
		cmd = cmd.substr(0, cmd.find("$"));
		int serial_num = atoi(cmd.c_str());
		//write file
		ofstream gnss_cfg_file("/home/jlurobot/catkin_ws/src/amazing_car/config/gnss_serial.cfg");
		gnss_cfg_file << serial_num;
		//start service
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/gnss.sh");
	}

	if(cmd.find("#CONTROLLER_OPEN") == 0){
		//get serial number
		cmd = cmd.substr(cmd.find("OPEN_") + 5);
		cmd = cmd.substr(0, cmd.find("$"));
		int serial_num = atoi(cmd.c_str());
		//write file
		ofstream gnss_cfg_file("/home/jlurobot/catkin_ws/src/amazing_car/config/controller.cfg");
		gnss_cfg_file << serial_num;
		//start service
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/controller.sh");
	}
	
	if(cmd.find("#ALGORITHM_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/algorithm.sh");
	}

	if(cmd.find("#VLP_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/vlp.sh");
	}

	if(cmd.find("#UI_OPEN") == 0){
		system("gnome-terminal -e /home/jlurobot/catkin_ws/src/amazing_car/shell/ui.sh");
	}

	if(cmd.find("#TAR_COUNT") == 0){
		init_tars_queue(tars, cmd);
	}

	if(cmd.find("#TARS_") == 0){
		add_tar_to_queue(tars, cmd);
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

void SendCarData(float x,float y,float angle,int state){
	char t[100];
	memset(t, 0, 100);
	sprintf(t,"#CARSTATE_X%.3f_Y%.3f_A%.2f_S%d$",x, y, angle, 4);

	std::string res = t;
	//printf("send: %s\n", res.c_str());
	p_my_serial->write(res);
}

void init_tars_queue(vector<CYCarPoint> & tar_vector, string cmd){
	cmd = cmd.substr(cmd.find("COUNT_") + 6);
	cmd = cmd.substr(0, cmd.find("$"));
	ordered_cmd_count = atoi(cmd.c_str());
	memset(cmd_flag, 0, 100);
	tar_vector.clear();
	for(int i = 0;i < ordered_cmd_count;i++){
		tar_vector.push_back(CYCarPoint(0, 0));
	}
}

void add_tar_to_queue(vector<CYCarPoint> & tar_vector, string cmd){
	//printf("ordered_cmd_count is %d\n", ordered_cmd_count);
	if(ordered_cmd_count == 0){
		return;
	}
	//"#TAR_Np1_Xp2_Yp3$"
	int _N_index = cmd.find("_N");
	int _X_index = cmd.find("_X");
	int _Y_index = cmd.find("_Y");
	int _N = atoi(cmd.substr(_N_index + 2, _X_index - _N_index).c_str());
	float _X = atof(cmd.substr(_X_index + 2, _Y_index - _X_index).c_str());
	float _Y = atof(cmd.substr(_Y_index + 2, cmd.find("$") - _Y_index).c_str());
	
	//printf("_N %d _X %f _Y %f\n", _N, _X, _Y);
	if(_N >= ordered_cmd_count){
		
		return;
	}

	if(cmd_flag[_N] == 0){
		tar_vector[_N].x = _X;
		tar_vector[_N].y = _Y;
		cmd_flag[_N] = 1;
		recieved_cmd_count++;
		if(recieved_cmd_count == ordered_cmd_count){
			
			amazing_car::my_checkpoints checkpoints;
			checkpoints.count = recieved_cmd_count;
			checkpoints.X.resize(recieved_cmd_count);
			checkpoints.Y.resize(recieved_cmd_count);
			for(int i = 0;i<recieved_cmd_count;i++){
				checkpoints.X[i] = tar_vector[i].x;
				checkpoints.Y[i] = tar_vector[i].y;
			}
			p_tars_pub->publish(checkpoints);
			ordered_cmd_count = 0;
			recieved_cmd_count = 0;
			memset(cmd_flag, 0, 100);
			tar_vector.clear();
		}
	}else if(cmd_flag[_N] == 1){
		//wrong
		printf("tar_cmds_wrongn\n");
		ordered_cmd_count = 0;
		recieved_cmd_count = 0;
		memset(cmd_flag, 0, 100);
		tar_vector.clear();
	}
}

