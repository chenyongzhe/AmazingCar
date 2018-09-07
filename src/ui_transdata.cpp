#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"
#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_gps_state.h"
#include "geometry_msgs/Twist.h"
#include "stdio.h"
#include "sensor_msgs/LaserScan.h"

#include <queue>

#include <stdio.h>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdlib.h>
#include <stddef.h>
#include <netinet/in.h>
#include <vector>
#include <math.h>
#include <mutex>

void Connect(const char* ipaddr,unsigned int port = 1888);

void SendCarData(float x,float y,float angle,int state);

std::mutex queue_mtx;

volatile bool bDataSignal = false;

volatile bool mtx = true;

volatile bool server_alive = false;

volatile bool server_connected = false;

volatile bool wtkey = true;

#define wait(b) do { while(!b);b = false;}while(0)

extern int errno;

struct CarData{
	float x,y,angle;
	int state;
};

struct CYCarPoint{
	float x;
	float y;	
};
void SetCallback(void(*)(int,const CYCarPoint*));


std::vector<CarData> v;
std::thread thrd;

void (*pcb)(int, const CYCarPoint*) = NULL;

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

void callback_server(const amazing_car::my_server_cmd cmd){
    if(cmd.ui_cmd == 0){
        system("exit");
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
	ros::init(argc, argv, "ui_transdata");
	ros::NodeHandle n;
	Connect("192.168.188.53");
	//Connect("192.168.0.124");
	//Connect("169.254.1.145");
	ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Subscriber car_angle_sub = n.subscribe("my_car_angle", 1000, callback_angle);
	ros::Subscriber car_location_sub = n.subscribe("my_car_location", 1000, callback_location);
	ros::Subscriber gps_state_sub = n.subscribe("my_gps_state", 1000, callback_state);
	ros::Publisher tar_pub = n.advertise<amazing_car::my_location_msg>("/my_tar_location", 1000);
	tar_pub_ptr = &tar_pub;
	SetCallback(update_tar);
	ros::Rate rate(15);

	auto t = thread(gjm_tar_thread,0);

	while(ros::ok()){
		//printf("%f, %f, %f\n", car_location_x, car_location_y, car_angle);
		SendCarData(car_location_x, car_location_y, (-90 - car_angle),location_state);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}


void NetServer(const char* ipAddr,unsigned int port){	
re_connect:
	int server = socket(AF_INET,SOCK_STREAM,0);
	sockaddr_in addr = {0};
    	addr.sin_family = AF_INET;
    	addr.sin_addr.s_addr = inet_addr(ipAddr);
   	 addr.sin_port = htons(port);
	CYCarPoint* CP = (CYCarPoint*)malloc(sizeof(CYCarPoint)*8);
	int CarPointCount = 8;
    	while( -1 == connect(server,(sockaddr*)&addr,sizeof(addr))){
    		printf("%d\n",errno);
    		usleep(1000000);
   	}
    	server_connected = true;
    	while(true){
    		while(!bDataSignal){
	    		fd_set st;
	    		FD_ZERO(&st);
	    		FD_SET(server,&st);
	    		timeval tm;
	    		tm.tv_sec = 0;
	    		tm.tv_usec = 10;
    			if( 0 < select(server+1,&st,NULL,NULL,&tm)){
    				printf("select get !\n");
    				if(FD_ISSET(server,&st)){
    					printf("GetData\n");
					int N;
    					int c = recv(server,&N,4,MSG_WAITALL);
					if (c > 0){
						if (N > CarPointCount) {
							CP = (CYCarPoint*)realloc(CP, sizeof(CYCarPoint) * N);
							CarPointCount = N;
						}
						c = recv(server, CP, N * sizeof(CYCarPoint), MSG_WAITALL);
					}
    					if(c <= 0){
	    					printf("server error\n");
	    					server_connected = false;
	    					close(server);
	    					goto re_connect;
					}
    					if(pcb){
						pcb(N, CP);
					}

    				}
    				fflush(stdout);
    			}
    		}
    		bDataSignal = false;
    	
    		wait(wtkey);
    		wait(mtx);
	    	wtkey = true;
	    	int result = 0;
	    	if(v.size() > 0){
	    		result = send(server,&v[0],sizeof(CarData) * v.size(),0);
	    		v.clear();
	    	}
	    	mtx = true;

	    	if(result <= 0){
	    		server_connected = false;
	    		close(server);
	    		goto re_connect;
	    	}

    	}
    	server_alive = false;
}

void Connect(const char* ipaddr,unsigned int port){
	if(server_alive)return;
	server_alive = true;
	thrd = std::thread(NetServer,ipaddr,port);
}

void SendCarData(float x,float y,float angle,int state){
	printf("x: %f y: %f angle: %f state %d\n",x,y,angle,state);

	wait(wtkey);
	wait(mtx);
	wtkey = true;
	v.push_back({x,y,angle,state});
	bDataSignal = true;
	while(v.size() > 1000){
		mtx = true;
		while(!server_connected);
		wait(wtkey);
		wait(mtx);
		wtkey = true;
	}
	mtx = true;
	
}

void SetCallback(void(*p)(int,const CYCarPoint * points)){
	pcb = p;
}

std::thread t;

void update_tar(int count, const CYCarPoint* points){
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
	for(int i = 0;i<count;i++){
		tar_points.push(points[i]);
	}
	queue_mtx.unlock();
}

void gjm_tar_thread(int){
	CYCarPoint tar = {0,0};
	while(true){
		queue_mtx.lock();
		printf("1\n");
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


