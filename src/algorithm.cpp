#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

#include <math.h>

#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_lidar_distance.h"
#include "geometry_msgs/Twist.h"
#include "pidClass.h"
#include "stdio.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"


//#include "rplidar.h"

//#ifndef _countof
//#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
//#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180/M_PI)

//lhw
using namespace std;

float car_angle = 0.0f;
float car_front_location_x = 0;
float car_front_location_y = 0;
float car_rear_location_x = 0;
float car_rear_location_y = 0;


float tar_location_x = 0;
float tar_location_y = 0;

bool stop_flag = false;

int shutdown_cmd = 1;

void callback_server(const amazing_car::my_server_cmd cmd){
    
	if(cmd.algorithm_cmd == 0){
       shutdown_cmd = 0;
    }
}

void scanCallbackRPLIDARA2(const sensor_msgs::LaserScan::ConstPtr& scan){
    int count = scan->scan_time / scan->time_increment;
    int min_count = 0;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    float min_range = 1.5;
    float min_degree = 1.5;
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	if(degree >= -135 && degree <= 135){
		continue;
	}
	if(scan->ranges[i] < 0.3){
		continue;
	}
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
	if(scan->ranges[i] < min_range){
		min_count ++;
		//min_range = scan->ranges[i];
		//min_degree = degree;
	}
    }
    //printf("%f %f\n", min_degree, min_range);
    printf("%d\n", min_count);
    if(min_count >= 10){
	stop_flag = true;
    }else{
	stop_flag = false;
    }
}

void scanCallbackVLP16(const sensor_msgs::PointCloud2 cloud){
	// [2018.07.01] PointCloud2 progress
	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(cloud, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
	float distance_max = 1.5;
	float distance_min = 0.3;
	float z_min = 0.3;
	float lidar_z = 1.6;
	int min_count = 0;
	for(int i = 0;i<temp_cloud->size();i++){
		pcl::PointXYZRGB point = (*temp_cloud)[i];
		if(point.x < fabs(point.y)){
			continue;
		}
		float d = sqrt(pow(point.x, 2) + pow(point.y, 2));
		//printf("%f\n",d);
		if(d >= distance_min && d <= distance_max){
			min_count++;
		} 
	}
	if(min_count >= 200){
		stop_flag = true;
	}else{
		stop_flag = false;
	}

	if(stop_flag){
		printf("can't move\n");
	}else{
		printf("can move\n");
	}
	//printf("count: %d\n", min_count);
	//printf("flag: %d\n", stop_flag);

	//getchar();
}

void scanCallbackBeiXing(const amazing_car::my_lidar_distance lidar_distance){
	if(lidar_distance.angle == -360){
		stop_flag = false;
	}else{
		if(lidar_distance.min_distance < 2){
			stop_flag = true;
		}else{
			stop_flag = false;
		}
	}

	if(stop_flag){
		printf("can moving!\n");
	}else{
		printf("can't moving!\n");
	}

}

void scanCallbackVlp16New(const amazing_car::my_lidar_distance lidar_distance){
	if(lidar_distance.angle == -360){
		stop_flag = false;
	}else{
		if(lidar_distance.min_distance < 2 && lidar_distance.angle >= -30 && lidar_distance.angle <= 30){
			stop_flag = true;
		}else{
			stop_flag = false;
		}
	}

	if(stop_flag){
		printf("can't moving!\n");
	}else{
		printf("can moving!\n");
	}

}


//guojm
void callback_angle(const amazing_car::my_angle_msg msg){
	car_angle = msg.yaw;
}
//guojm
void callback_front_location(const amazing_car::my_location_msg msg){
	car_front_location_x = msg.x;
	car_front_location_y = msg.y;
}
void callback_rear_location(const amazing_car::my_location_msg msg){
	car_rear_location_x = msg.x;
	car_rear_location_y = msg.y;
}

//guojm
void callback_tar_location(const amazing_car::my_location_msg msg){
	tar_location_x = msg.x;
	tar_location_y = msg.y;
}
//guojm
geometry_msgs::Twist algorithm(float car_angle, float car_front_location_x, float car_front_location_y, float car_rear_location_x, float car_rear_location_y, float tar_location_x, float tar_loaction_y, int stop_distance){
	geometry_msgs::Twist cmd;
	/////////////////////////////////////////
	magicPoint frontPoint(car_front_location_x, car_front_location_y);
	magicPoint rearPoint(car_rear_location_x, car_rear_location_y);
	magicPoint tarPoint(tar_location_x, tar_loaction_y);
	

    	wheelSpeed result = pidClass::simpleAlgorithm(frontPoint, tarPoint, car_angle, stop_distance);
	//wheelSpeed result = pidClass::traverse(frontPoint, car_angle, stop_distance);

	//printf("speed %f %f\n", result.left, result.right);
	cmd.linear.x = result.left;
	cmd.linear.y = result.right;
	/////////////////////////////////////////
	return cmd;
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "algorithm");
	ros::NodeHandle n;
	ros::Subscriber car_angle_sub = n.subscribe("my_car_angle", 1000, callback_angle);
	ros::Subscriber car_front_location_sub = n.subscribe("my_car_location", 1000, callback_front_location);
	//ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallbackRPLIDARA2);
	ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, scanCallbackVLP16);
	//ros::Subscriber sub3 = n.subscribe<amazing_car::my_lidar_distance>("/vlp16_lidar", 1000, scanCallbackVlp16New);
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Subscriber tar_location_sub = n.subscribe("my_tar_location", 1000, callback_tar_location);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Rate rate(80);
	while(ros::ok()){
		if(shutdown_cmd == 0){
			break;
		}
		if(!stop_flag){
			//printf("1\n");
			//printf("Angle:%f L_X:%f L_Y:%f T_X:%f T_Y:%f\n", car_angle, car_front_location_x, car_front_location_y, pathToTraverse.front().x, pathToTraverse.front().y);
			//printf("Angle:%f L_X:%f L_Y:%f T_X:%f T_Y:%f\n", car_angle, car_front_location_x, car_front_location_y, tar_location_x, tar_location_y);
			geometry_msgs::Twist cmd = algorithm(car_angle, car_front_location_x, car_front_location_y, car_rear_location_x, car_rear_location_y, tar_location_x, tar_location_y, 1);
		        //printf("cmd_l:%f cmd_r:%f\n", cmd.linear.x, cmd.linear.y);
			cmd_pub.publish(cmd);
		}else{
			//printf("2\n");
			geometry_msgs::Twist cmd;
			cmd.linear.x = 200;
			cmd.linear.y = 200;
			cmd.linear.z = 200;
			cmd_pub.publish(cmd);
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
