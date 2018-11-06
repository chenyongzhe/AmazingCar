#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "amazing_car/my_server_cmd.h"
#include "amazing_car/my_node_state.h"
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

#define BYTE unsigned char

using namespace std;

int shutdown_cmd = 1;

struct ControllerState{
	int speed;
	int direction;
};

ControllerState controller_state;

//int 转 4字节 BYTE[],
void intToByte(int i, BYTE abyte[]) {
	abyte[3] = (BYTE)(0x000000ff & i);
	abyte[2] = (BYTE)((0x0000ff00 & i) >> 8);
	abyte[1] = (BYTE)((0x00ff0000 & i) >> 16);
	abyte[0] = (BYTE)((0xff000000 & i) >> 24);
}

//4字节 BYTE[] 转 int 
int bytesToInt(BYTE bytes[]) {
	int addr = bytes[3] & 0xFF;
	addr |= ((bytes[2] << 8) & 0xFF00);
	addr |= ((bytes[1] << 16) & 0xFF0000);
	addr |= ((bytes[0] << 24) & 0xFF000000);
	//long int result = (x[0] << 24) + (x[1] << 16) + (x[2] << 8) + x[3];   
	return addr;
}


void callback_server(const amazing_car::my_server_cmd cmd){
    if(cmd.controller_cmd == 0){
        shutdown_cmd = 0;
    }
}

int speed = 0; //0.01m/s 
int direction = 0; //0.001rad 


void callback(const geometry_msgs::Twist& cmd_vel){
	float left = cmd_vel.linear.x;
	float right = cmd_vel.linear.y;
	if(left == 50 && right == 350){
		speed = 300;
		direction = 314;
	}else if(left == 350 && right == 50){
		speed = 300;
		direction = -314;
	}else if(left == 350 && right == 350){
		speed = 300;
		direction = 0;
	}else{  
		speed = 0;
		direction = 0;
	}

	controller_state.speed = speed;
	controller_state.direction = direction;
}

int id = 0;
BYTE * temp = new BYTE[4];

void get_res(uint8_t* buffer, int buffer_size){
	int data_size = 0;
	if(buffer_size < 16){
		cout<<"The buffer doesn't have enough space\n"<<endl;
	}
	if(id >= 255){
		id = 0;
	}
	buffer[0] = 0xAA;
	buffer[1] = id;
	buffer[2] = 0x01;
	buffer[3] = 0;
	intToByte(speed, temp);
	buffer[4] = temp[2];
	buffer[5] = temp[3];
	buffer[6] = 0;
	buffer[7] = 0;
	intToByte(direction, temp);
	buffer[8] = temp[2];
	buffer[9] = temp[3];
	buffer[10] = 0;
	buffer[11] = 0;
	buffer[12] = 0;
	buffer[13] = 0;
	buffer[14] = 0;
	for(int i = 0;i<14;i++){
		buffer[14] += buffer[i];
	}
	buffer[15] = 0x55;
	id++;
	return;
}

void parking(){
	char temp_buffer[16];
	temp_buffer[0] = 0xAA;
	
	temp_buffer[2] = 0x01;
	temp_buffer[3] = 0;
	temp_buffer[4] = 0;
	temp_buffer[5] = 0;
	temp_buffer[6] = 0;
	temp_buffer[7] = 0;
	temp_buffer[8] = 0;
	temp_buffer[9] = 0;
	temp_buffer[10] = 0;
	temp_buffer[11] = 0;
	temp_buffer[12] = 0;
	temp_buffer[13] = 0;
	temp_buffer[14] = 0;
	if(id >= 255){
		id = 0;
	}
	temp_buffer[15] = 0x55;
	id++;
	//关闭串口前 发送50条停车指令
	for(int i = 0;i<50;i++){
		temp_buffer[1] = id;
		for(int i = 0;i<14;i++){
			temp_buffer[14] += temp_buffer[i];
		}
		//my_serial.write(buffer, 16);
		id++;
	}
}

int main(int argc, char ** argv){
	std::ifstream controller_cfg("/home/jlurobot/catkin_ws/src/amazing_car/config/controller.cfg");
    int serial_num = 111;
    controller_cfg >> serial_num;
    char serial_num_str[20];
    memset(serial_num_str, 0, 20);
    sprintf(serial_num_str, "/dev/ttyUSB%d", serial_num);
    serial::Serial my_serial(serial_num_str, 57600, serial::Timeout::simpleTimeout(1000));
	uint8_t* buffer = new uint8_t[16];
	ros::init(argc, argv, "my_wheeled_car_controller");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Publisher state_pub = n.advertise<amazing_car::my_node_state>("/my_nodes_state", 1000);
	ros::Rate rate(20);
	while(ros::ok()){
		if(shutdown_cmd == 0){
			parking();
			break;
		}

		amazing_car::my_node_state node_state_msg;
		node_state_msg.node_name = "my_wheeled_car_controller";
		node_state_msg.node_state = 1;
		node_state_msg.extra_info = "";
		
		node_state_msg.extra_info += to_string(controller_state.speed);
		node_state_msg.extra_info += " ";
		node_state_msg.extra_info += to_string(controller_state.direction);

		state_pub.publish(node_state_msg);

		get_res(buffer, 16);
		for(int i = 0;i<16;i++){
			printf("%d ", buffer[i]);
		}
		printf("\n");
		
	   	my_serial.write(buffer, 16);
		//cout<<"Speed: " << speed << " Direction: " << direction << endl;
		usleep(40000);
		ros::spinOnce();
	}
	return 0;
}
