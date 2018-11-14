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

int get_state(uint8_t* buffer, int buffer_size);
int set_state(uint8_t* buffer, int buffer_size);

int shutdown_cmd = 1;
int speed = 0; //0.01m/s 
int direction = 0; //0.001rad 
int turn_flag = 0; // 0 straight  -1 left  1 right

CarState state;


struct ControllerState{
	int speed;
	int direction;
};

typedef struct {
	float speed; //速度，单位是m/s
	float angle; //角度，左负右正，单位是°
	uint8_t gas;
	uint8_t direction;
	uint8_t turn;
	int drive_mode; //运行模式，0是限速安全模式，1是中速 2是高速 3是不限速
	float battery_percent; //电量百分比
	void print_car_state() {
		if (drive_mode == 1){
			printf("Gas:%d Turn:%d Mode:Mid-Speed Diretion:%d\r", gas, turn, battery_percent, direction);
		} else if (drive_mode == 2) {
			printf("Gas:%d Turn:%d Mode:High-Speed Diretion:%d\r", gas, turn, battery_percent, direction);
		} else if (drive_mode == 3) {
			printf("Gas:%d Turn:%d Mode:Unlimited-Speed Diretion:%d\r", gas, turn, battery_percent, direction);
		} else {
			printf("Gas:%d Turn:%d Mode:Low-Speed Diretion:%d\r", gas, turn, battery_percent, direction);
		}
	}
	void trans_state_data(uint8_t* ori_data) {
		if (ori_data[4] >= 0xF0) {
			//速度负值
			speed = ~ori_data[4] << 8 + ~ori_data[5] + 1;
		} else {
			//速度正值
			speed = ori_data[4] << 8 + ori_data[5];
		}
		if (ori_data[4] >= 0xF0) {
			//角度负值
			angle = ~ori_data[8] << 8 + ~ori_data[9] + 1;
		} else {
			//角度正值
			angle = ori_data[8] << 8 + ori_data[9];
		}
		drive_mode = ori_data[3];
		gas = ori_data[6];
		turn = ori_data[7];
		direction = ori_data[10];
		battery_percent = ori_data[11];
	}
	void print_ori_state(uint8_t* ori_data) {
		for (int i = 0; i < 24; i++) {
			printf("%d ", ori_data[i]);
			if (ori_data[i] == 0x55) {
				printf("\r");
			}
		}
	}
}CarState;

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
    //if( == 0){
    shutdown_cmd = cmd.controller_cmd;
}

void callback(const geometry_msgs::Twist& cmd_vel){
	float left = cmd_vel.linear.x;
	float right = cmd_vel.linear.y;
	float beta = cmd_vel.linear.z;
	
	if(left == 50 && right == 350){
		speed = 300;
		direction = 314;
		//left
		turn_flag = -1;

	}else if(left == 350 && right == 50){
		speed = 300;
		direction = -314;
		//right
		turn_flag = 1;
	}else if(left == 350 && right == 350){
		if(turn_flag != 0){
			//wait wheel to zero
			if(fabs(state.angle + 571) <= 20){
				//继续运动
				turn_flag = 0;		
			}
			speed = 0;
			direction = 0;
		}else{
			speed = 300;
			while(beta > 180){
				beta -= 360;
			}
			beta = beta * -1 * 3.1415926 * 1000 / 180;
			direction = beta;
		}
	}else{  
		speed = 0;
		direction = 0;
	}

	controller_state.speed = speed;
	controller_state.direction = direction;
}

int id = 0;
BYTE * temp = new BYTE[4];

serial::Serial * p_my_serial;

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
    serial::Serial my_serial(serial_num_str, 115200, serial::Timeout::simpleTimeout(1000));
	p_my_serial = &my_serial;
	uint8_t* buffer = new uint8_t[16];
	uint8_t* recv_buffer = new uint8_t[24];
	ros::init(argc, argv, "my_wheeled_car_controller");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Publisher state_pub = n.advertise<amazing_car::my_node_state>("/my_nodes_state", 1000);
	ros::Rate rate(20);
	while(ros::ok()){
		if(shutdown_cmd == 0){
			parking();
			//break;
		}else{
			amazing_car::my_node_state node_state_msg;
			node_state_msg.node_name = "my_wheeled_car_controller";
			node_state_msg.node_state = 1;
			node_state_msg.extra_info = "";
			
			node_state_msg.extra_info += to_string(controller_state.speed);
			node_state_msg.extra_info += " ";
			node_state_msg.extra_info += to_string(controller_state.direction);

			state_pub.publish(node_state_msg);
			set_state(buffer, 16);
			get_state(recv_buffer, 24, state);
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

int get_state(uint8_t* buffer, int buffer_size, CarState& state) {
	int data_size = 0;
	size_t recv_size = p_my_serial->read(buffer, buffer_size);
	for (int i = 0; i < recv_size; i++) {
		//读到了开始标记
		if (buffer[i] == 0xAA) {
			//开始读接下来23个字节
			data_buffer[0] = 0xAA;
			recv_flag = true;
			recv_bytes_count = 1;
		} else if (recv_flag) {
			data_buffer[recv_bytes_count++] = buffer[i];
		}
		//读满了24个字节
		if (recv_bytes_count == 24) {
			//检查开始标记，结束标记，校验和，之后进行处理
			//data_buffer[0] == 0xAA && data_buffer[23] == 0x55 && sum(data_buffer, 0, 21) == data_buffer[22]
			if (true) {
				state.trans_state_data(data_buffer);
				//输出state
				//state.print_ori_state(data_buffer);
				//state.print_car_state();
			}
		}

	}
	return data_size;
}

int set_state(uint8_t* buffer, int buffer_size) {
	int date_size = 0;
	if (buffer_size < 16) {
		cout << "The send buffer doesn't have enough space\n" << endl;
	}
	if (id >= 0xEF) {
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
	buffer[10] = mode;
	buffer[11] = 0;
	buffer[12] = 0;
	buffer[13] = 0;
	buffer[14] = 0;
	for (int i = 0; i<14; i++) {
		buffer[14] += buffer[i];
	}
	buffer[15] = 0x55;
	id++;
	p_my_serial->write(buffer, 16);
	return date_size;
}
