#include "ros/ros.h"
#include "amazing_car/my_angle_msg.h"
#include "stdio.h"

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::string;

serial::Serial my_serial("/dev/ttyUSB1", 9600, serial::Timeout::simpleTimeout(1000));


float zeroAngle = 0.0;

float angle_translate(float yaw){
	float angle = yaw - zeroAngle;
	if(angle < 0){
		angle += 360;
	}
	return angle;
}

float calibration(){
	ROS_INFO("When setting is OK, press Y/y to finish calibration");
	char * choose = new char;
	scanf("%c", choose);
	while(*choose != 'y' && *choose != 'Y'){
		ROS_INFO("Please press Y/y to finish calibration");
		scanf("%c", choose);
	}
	float sum = 0.0;
	for(int i = 0;i<10;i++){
		std::string angle_str;
		my_serial.readline(angle_str);
		float yaw = atof(angle_str.c_str());
		sum += yaw;
	}
	zeroAngle = sum /= 10;
	ROS_INFO("OK, the zero angluar in this coordinate system is %f", zeroAngle);
	delete choose;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_car_angle_publisher");
	ros::NodeHandle n;
	ros::Publisher angle_pub = n.advertise<amazing_car::my_angle_msg>("my_car_angle", 1000);
	ros::Rate rate(50);
	double yaw;
	//calibration();
	while(ros::ok()){
		std::string angle_str;
		my_serial.readline(angle_str);
		float yaw = atof(angle_str.c_str());
		amazing_car::my_angle_msg msg;
		msg.yaw = angle_translate(yaw);
		printf("%f\n", msg.yaw);
		angle_pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
