#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

#include <math.h>
#include <string>
#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_lidar_distance.h"
#include "amazing_car/my_node_state.h"

#include "std_msgs/Bool.h"

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

struct AlgorithmState{
	float left_speed;
	float right_speed;
	float beta;
	int stop_flag;
};


AlgorithmState algorithm_state;

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
		if(scan->ranges[i] < min_range){
			min_count ++;
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
		algorithm_state.stop_flag = 1;
	}else{
		stop_flag = false;
		algorithm_state.stop_flag = 0;
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

bool shutdown_flag = false;

void callback_shutdown_flag(const std_msgs::Bool msg){
	shutdown_flag = msg.data;
}

void callback_angle(const amazing_car::my_angle_msg msg){
	car_angle = msg.yaw;
}

void callback_front_location(const amazing_car::my_location_msg msg){
	car_front_location_x = msg.x;
	car_front_location_y = msg.y;
}

void callback_rear_location(const amazing_car::my_location_msg msg){
	car_rear_location_x = msg.x;
	car_rear_location_y = msg.y;
}

void callback_tar_location(const amazing_car::my_location_msg msg){
	tar_location_x = msg.x;
	tar_location_y = msg.y;
}

geometry_msgs::Twist algorithm(float car_angle, float car_front_location_x, float car_front_location_y, float car_rear_location_x, float car_rear_location_y, float tar_location_x, float tar_loaction_y, int stop_distance){
	geometry_msgs::Twist cmd;
	/////////////////////////////////////////
	magicPoint frontPoint(car_front_location_x, car_front_location_y);
	magicPoint rearPoint(car_rear_location_x, car_rear_location_y);
	magicPoint tarPoint(tar_location_x, tar_loaction_y);
	wheelSpeed result = pidClass::simpleAlgorithm(frontPoint, tarPoint, car_angle, stop_distance);
	//wheelSpeed result = pidClass::traverse(frontPoint, car_angle, stop_distance);

	if(shutdown_flag){
		result.left = 200;
		result.right = 200;
	}
	printf("speed %f %f\n", result.left, result.right);
	cmd.linear.x = result.left;
	cmd.linear.y = result.right;
	cmd.linear.z = result.beta;
	////////////////////
	algorithm_state.left_speed = result.left;
	algorithm_state.right_speed = result.right;
	algorithm_state.beta = result.beta;
	/////////////////////////////////////////
	return cmd;
}


int main(int argc, char ** argv){
	ros::init(argc, argv, "algorithm");
	ros::NodeHandle n;
	ros::Subscriber car_angle_sub = n.subscribe("my_car_angle", 1000, callback_angle);

	ros::Subscriber shutdown_sub = n.subscribe("my_shutdown_flag", 1000, callback_shutdown_flag);

	ros::Subscriber car_front_location_sub = n.subscribe("my_car_location", 1000, callback_front_location);
	//ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallbackRPLIDARA2);
	ros::Subscriber sub2 = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, scanCallbackVLP16);
	//ros::Subscriber sub3 = n.subscribe<amazing_car::my_lidar_distance>("/vlp16_lidar", 1000, scanCallbackVlp16New);
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Subscriber tar_location_sub = n.subscribe("my_tar_location", 1000, callback_tar_location);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	ros::Publisher state_pub = n.advertise<amazing_car::my_node_state>("/my_nodes_state", 1000);

	ros::Rate rate(20);

	while(ros::ok()){
		if(shutdown_cmd == 0){
			break;
		}

		amazing_car::my_node_state node_state_msg;
		node_state_msg.node_name = "algorithm";
		node_state_msg.node_state = 1;
		node_state_msg.extra_info = "";
		node_state_msg.extra_info += to_string(algorithm_state.left_speed);
		node_state_msg.extra_info += " ";
		node_state_msg.extra_info += to_string(algorithm_state.right_speed);
		node_state_msg.extra_info += " ";
		node_state_msg.extra_info += to_string(algorithm_state.beta);
		node_state_msg.extra_info += " ";
		node_state_msg.extra_info += to_string(algorithm_state.stop_flag);

		state_pub.publish(node_state_msg);

		if(!stop_flag){
			geometry_msgs::Twist cmd = algorithm(car_angle, car_front_location_x, car_front_location_y, car_rear_location_x, car_rear_location_y, tar_location_x, tar_location_y, 1);
			cmd_pub.publish(cmd);
		}else{
			geometry_msgs::Twist cmd;
			cmd.linear.x = 200;
			cmd.linear.y = 200;
			cmd.linear.z = 0;
			cmd_pub.publish(cmd);
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
