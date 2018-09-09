#include "ros/ros.h"
#include "amazing_car/my_car_state.h"
#include "amazing_car/my_server_cmd.h"
#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_gps_state.h"
#include "amazing_car/my_checkpoints.h"

#include "geometry_msgs/Twist.h"
#include "stdio.h"
#include "sensor_msgs/LaserScan.h"

#include <queue>

#include <stdio.h>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <vector>
#include <math.h>


struct CarData{
	float x,y,angle;
	int state;
};

struct CYCarPoint{
	CYCarPoint(float x, float y){
		this->x = x;
		this->y = y;
	}
	float x;
	float y;	
};

using namespace std;

float car_angle = 0.0f;
float car_location_x = 0;
float car_location_y = 0;
int location_state = 0;
ros::Publisher * tar_pub_ptr = NULL;

void update_tar(int count, const CYCarPoint* points);
void gjm_tar_thread(int);

float tar_location_x = 0;
float tar_location_y = 0;

float stop_distance = 1;

std::queue<CYCarPoint> tar_points;
int th_count = 0;

int shutdown_cmd = 1;

void callback_checkpoints(const amazing_car::my_checkpoints points){
	static bool bt = false;
	if(bt){
		t.join();
		bt = true;
	}
	queue_mtx.lock();
	tar_points = queue<CYCarPoint>();
	while(!tar_points.empty()){
		tar_points.pop();
	}
	for(int i = 0;i<points.count;i++){
		tar_points.push(CYCarPoint(points.X[i], points.Y[i]));
	}
	queue_mtx.unlock();
}

void callback_server(const amazing_car::my_server_cmd cmd){
    if(cmd.ui_cmd == 0){
        shutdown_cmd = 0;
    }
}

void callback_angle(const amazing_car::my_angle_msg msg){
	car_angle = msg.yaw;
}

//guojm
void callback_location(const amazing_car::my_location_msg msg){
	car_location_x = msg.x;
	car_location_y = msg.y;
}

void callback_state(const amazing_car::my_gps_state msg){
	location_state = msg.location_state;
}

int main(int argc, char ** argv){

	std::ifstream data_cfg("/home/jlurobot/catkin_ws/src/amazing_car/config/data_transer.cfg");
	int serial_num = 111;
	data_cfg >> serial_num;
	char serial_num_str[20];
	memset(serial_num_str, 0, 20);
	sprintf(serial_num_str, "/dev/ttyUSB%d", serial_num);
	serial::Serial my_serial(serial_num_str, 115200, serial::Timeout::simpleTimeout(1000));
	p_myserial = &my_serial;

	ros::init(argc, argv, "ui_transdata");
	ros::NodeHandle n;
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Subscriber car_angle_sub = n.subscribe("my_car_angle", 1000, callback_angle);
	ros::Subscriber car_location_sub = n.subscribe("my_car_location", 1000, callback_location);
	ros::Subscriber gps_state_sub = n.subscribe("my_gps_state", 1000, callback_state);
	ros::Subscriber checkpoints_sub = n.subscribe("my_checkpoints", 1000, callback_checkpoints);
	ros::Publisher tar_pub = n.advertise<amazing_car::my_location_msg>("/my_tar_location", 1000);
	ros::Publisher car_pub = n.advertise<amazing_car::my_car_state>("/my_car_state", 1000);
	
	tar_pub_ptr = &tar_pub;

	ros::Rate rate(15);

	auto t = thread(gjm_tar_thread,0);

	while(ros::ok()){
		if(shutdown_cmd == 0){
			break;
		}
		amazing_car::my_car_state msg;
		msg.x = car_location_x;
		msg.y = car_location_y;
		msg.angle = -90 - car_angle;
		msg.state = location_state;
		car_pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}


std::thread t;

void gjm_tar_thread(int){
	CYCarPoint tar = {0,0};
	while(true){
		queue_mtx.lock();
		fflush(stdout);
		if(!tar_points.empty()){
			printf("tar: %f %f\n",tar.x,tar.y);
			
			tar = tar_points.front();
			
			if(sqrt((tar.x - car_location_x) * (tar.x - car_location_x) + (tar.y - car_location_y) * (tar.y - car_location_y)) <= stop_distance){
				printf("pop\n");
				tar_points.pop();
			}else{
				queue_mtx.unlock();
				amazing_car::my_location_msg msg;
				msg.x = tar.x;
				msg.y = tar.y;
				printf("pub\n");
				tar_pub_ptr->publish(msg);
				queue_mtx.lock();
			}
		}
		queue_mtx.unlock();
		usleep(200000);	
	}
}

