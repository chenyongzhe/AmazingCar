#include "ros/ros.h"
#include "amazing_car/my_location_msg.h"
#include "stdio.h"
#include "pidClass.h"
#include "wheelSpeed.h"
#include "magicVector.h"
#include "magicPoint.h"
#include "unistd.h"

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_tar_location_publisher");
	ros::NodeHandle n;
	ros::Publisher location_pub = n.advertise<amazing_car::my_location_msg>("my_tar_location", 1000);
	ros::Rate rate(30);
	float x, y = 0;
	

	while(ros::ok()){

		scanf("%f %f",&x,&y);	
		amazing_car::my_location_msg msg;
		msg.x = x;
		msg.y = y;
		location_pub.publish(msg);
		

		/*
		scanf("%f %f",&x,&y);
		pathToTraverse.push_back(magicPoint(x,y));
		usleep(400000);
		*/

		ros::spinOnce();
		rate.sleep();
		
	}
	return 0;
}
