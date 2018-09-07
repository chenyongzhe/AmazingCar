#include "ros/ros.h"
#include "amazing_car/my_server_cmd.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_gps_state.h"
#include "stdio.h"
#include <fstream>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include <math.h>

using std::string;

int gps_angle_state = 0;

float zero_angle = 246.38;
float diff_angle = 180;

float tar_location_x = 0;
float tar_location_y = 0;

typedef struct{
    //jingdu
    double lon;
    std::string lon_dir;  // E  or  W
    //weidu
    double lat;
    std::string lat_dir;  // S  or  N

    int state;
}GPGGA_Data;

typedef struct{
    //distance
    double dis;
    //x_distance
    double x_dis;   //  East
    //y_distance
    double y_dis;   //  North
}GPNTR_Data;

typedef struct{
    //yaw
    double yaw;
    //roll
    double roll;
    //pitch
    double pitch;
    //
    int state;
}GPTRA_Data;

typedef struct{
    //yaw
    double yaw;

    int state;
}GPHDT_Data;


struct Vec2d{
    double x;
    double y;
};

struct DMS{
    DMS(double dm){
        dd = (int)dm / 100;
        mm = (int)dm - 100 * dd;
        ss = (dm - 100 * dd - mm) * 60.0f;
    }
    int dd;
    int mm;
    double ss;
};

void callback_tar_location(const amazing_car::my_location_msg msg){
	tar_location_x = msg.x;
	tar_location_y = msg.y;
}

void callback_server(const amazing_car::my_server_cmd cmd){
    if(cmd.gnss_cmd == 0){
        system("exit");
    }
}


GPGGA_Data decodeGPGGA(std::string gpgga_msg);
GPNTR_Data decodeGPNTR(std::string gpntr_msg);
GPTRA_Data decodeGPTRA(std::string gptra_msg);
GPHDT_Data decodeGPHDT(std::string gphdt_msg);
double DeltaLat(const DMS & base, const DMS & dest);
double DeltaLon(const DMS & base, const DMS & dest);
Vec2d get_distance1(double latDest, double lngDest, double latOrg, double lngOrg);


GPGGA_Data ori_data;

int main(int argc, char ** argv){
	//ori_data.lat = 4349.15060000;
	//ori_data.lon = 12516.63578100;
	ori_data.lat = 4349.15050130;
	ori_data.lon = 12516.63578153;

    std::ifstream gnss_cfg("/home/jlurobot/catkin_ws/src/amazing_car/config/gnss_serial.cfg");
    int serial_num = 0;
    gnss_cfg >> serial_num;
    char serial_num_str[20];
    memset(serial_num_str, 0, 20);
    sprintf(serial_num_str, "/dev/ttyUSB%d", serial_num);
    serial::Serial my_serial(serial_num_str, 115200, serial::Timeout::simpleTimeout(1000));


	ros::init(argc, argv, "gprs_location_publisher");
	ros::NodeHandle n;
	string location_str;
	ros::Publisher location_pub = n.advertise<amazing_car::my_location_msg>("my_car_location", 1000);
	ros::Publisher angle_pub = n.advertise<amazing_car::my_angle_msg>("my_car_angle", 1000);
	ros::Publisher gps_state_pub = n.advertise<amazing_car::my_gps_state>("my_gps_state", 1000);
	ros::Subscriber tar_location_sub = n.subscribe("my_tar_location", 1000, callback_tar_location);
    ros::Subscriber server_cmd_sub = n.subscribe("server_cmd", 1000, callback_server);
	ros::Rate rate(120);
	while(ros::ok()){
		location_str.clear();
		amazing_car::my_location_msg location_msg;
		amazing_car::my_angle_msg angle_msg;
		amazing_car::my_gps_state state_msg;
		try{
		    //get data
		    //usleep(5000);
		    std::string data;
	            my_serial.readline(data);
	            printf("%s", data.c_str());
		    //decode data and update data
		    GPGGA_Data gpgga_data;
		    GPNTR_Data gpntr_data;
		    GPTRA_Data gptra_data;
		    GPHDT_Data gphdt_data;
		    if(data.c_str() == NULL || strlen(data.c_str()) < 10){
		    //if(strlen(data.c_str()) < 10){
		        continue;
		    }
		    if(data.c_str()[3] == 'N'){
		        gpntr_data = decodeGPNTR(data);
		    }else if(data.c_str()[3] == 'G'){
		        gpgga_data = decodeGPGGA(data);
			state_msg.location_state = gpgga_data.state;
			state_msg.angle_state = gpgga_data.state;
			gps_angle_state = gpgga_data.state;
			gps_state_pub.publish(state_msg);
			if(gpgga_data.state != 1 && gpgga_data.state != 2 && gpgga_data.state != 4 && gpgga_data.state != 5){
				location_msg.x = tar_location_x;
				location_msg.y = tar_location_y;
				printf("%d\n", gpgga_data.state);
				location_pub.publish(location_msg);
				continue;
			}
			//translate gpgga to cordinate
			//---------------------------
			Vec2d location = get_distance1(gpgga_data.lat,gpgga_data.lon,ori_data.lat,ori_data.lon);
			
			double ori_x_data = location.y;
			double ori_y_data = -location.x;
			float alpha = zero_angle - diff_angle < 0 ? zero_angle - diff_angle + 360 : zero_angle - diff_angle;
			float beta = gptra_data.yaw - diff_angle < 0 ? gptra_data.yaw - diff_angle + 360 : gptra_data.yaw - diff_angle;
			float gama = atan(-ori_y_data / ori_x_data);
			float axis_diff = -(90 - (zero_angle - diff_angle));
			axis_diff = axis_diff * M_PI / 180;			

			location_msg.y = ori_x_data * cos(axis_diff) - ori_y_data * sin(axis_diff);
			location_msg.x = - (ori_x_data * sin(axis_diff) + ori_y_data * cos(axis_diff));			

			
			//printf("%d %3f %3f\n", gpgga_data.state, location_msg.x , location_msg.y);
			//---------------------------
			location_pub.publish(location_msg);
		    }else if(data.c_str()[3] == 'T'){
		        gptra_data = decodeGPTRA(data);
			
			float alpha = zero_angle - diff_angle < 0 ? zero_angle - diff_angle + 360 : zero_angle - diff_angle;
			float beta = gptra_data.yaw - diff_angle < 0 ? gptra_data.yaw - diff_angle + 360 : gptra_data.yaw - diff_angle;

			if(beta >= alpha){
				angle_msg.yaw = beta - alpha;
			}else {
				angle_msg.yaw = 360 + beta - alpha;
			}
			if(!(gptra_data.yaw == 0 || gptra_data.state != 4)){
				angle_pub.publish(angle_msg);
			}

			//printf("%3f %d %3f\n", gptra_data.yaw, gptra_data.state, angle_msg.yaw);
			
		    }else if(data.c_str()[3] == 'H'){
		        gphdt_data = decodeGPHDT(data);

			float alpha = zero_angle < 0 ? zero_angle + 360 : zero_angle;
			float beta = gphdt_data.yaw < 0 ? gptra_data.yaw + 360 : gphdt_data.yaw;

			if(beta >= alpha){
				angle_msg.yaw = beta - alpha - diff_angle;
			}else {
				angle_msg.yaw = 360 + beta - alpha - diff_angle;
			}
			if(angle_msg.yaw < 0){
				angle_msg.yaw += 360;
			}

			if(!gphdt_data.yaw == 0){
				angle_pub.publish(angle_msg);
			}

			//printf("%3f %d %3f\n", gptra_data.yaw, gptra_data.state, angle_msg.yaw);
			
		    }
		}catch(...){
		    //continue;
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

void split(std::string& s, std::string& delim, std::vector<std::string>* ret){
	size_t last = 0;
	size_t index = s.find_first_of(delim, last);
	while (index != std::string::npos){
		ret->push_back(s.substr(last, index - last));
		last = index + 1;
		index = s.find_first_of(delim, last);
	}
	if (index - last>0){
		ret->push_back(s.substr(last, index - last));
	}
}

GPGGA_Data decodeGPGGA(std::string gpgga_msg){
    GPGGA_Data data;
    std::vector<std::string> vec;
    vec.clear();
    std::string delim = ",";
    split(gpgga_msg, delim, &vec);
    if(vec.size() < 6){
        data.lat = 0;
        data.lon = 0;
    } else if(strcmp(vec[0].c_str(),"$GPGGA") == 0 || strcmp(vec[0].c_str(),"$GNGGA") == 0){
        data.lat = atof(vec[2].c_str());
        data.lat_dir = vec[3].c_str();
        data.lon = atof(vec[4].c_str());
        data.lon_dir = vec[5].c_str();
	data.state = atoi(vec[6].c_str());
    }else{

    }
    return data;
}

GPTRA_Data decodeGPTRA(std::string gptra_msg){
    GPTRA_Data data;
    std::vector<std::string> vec;
    vec.clear();
    std::string delim = ",";
    split(gptra_msg, delim, &vec);
    if(vec.size() < 3){
        data.yaw = 0;
    } else if(strcmp(vec[0].c_str(),"$GPTRA") == 0){
        data.yaw = atof(vec[2].c_str());
	data.state = atoi(vec[5].c_str());
    }else{

    }
    return data;
}

GPHDT_Data decodeGPHDT(std::string gphdt_msg){
    GPHDT_Data data;
    std::vector<std::string> vec;
    vec.clear();
    std::string delim = ",";
    split(gphdt_msg, delim, &vec);
    if(vec.size() < 3){
        data.yaw = 0;
    } 
    //else if(strcmp(vec[0].c_str(),"$GPHDT") == 0 || strcmp(vec[0].c_str(),"$GNHDT")){
        data.yaw = atof(vec[1].c_str());
    //}else{

    //}
    return data;
}

GPNTR_Data decodeGPNTR(std::string gpntr_msg){
    GPNTR_Data data;
    std::vector<std::string> vec;
    vec.clear();
    std::string delim = ",";
    split(gpntr_msg, delim, &vec);
    if(vec.size() < 6){
        data.dis = 0;
        data.x_dis = 0;
        data.y_dis = 0;
    } else if(strcmp(vec[0].c_str(),"$GPNTR") == 0){
        data.dis = atof(vec[3].c_str());
        if(vec[4].c_str()[0] == '+'){
            data.x_dis = atof(vec[4].substr(1).c_str());
        }else{
            data.x_dis = atof(vec[4].c_str());
        }
        if(vec[5].c_str()[0] == '+'){
            data.y_dis = atof(vec[5].substr(1).c_str());
        }else{
            data.y_dis = atof(vec[5].c_str());
        }
    }else{

    }
    return data;
}

double DeltaLat(const DMS & base, const DMS & dest)
{
    double distance = (dest.dd - base.dd) * 111000.0f;
    distance += (dest.mm - base.mm) * 1850.0f;
    distance += (dest.ss - base.ss) * 30.9f;
    return distance;
}

double DeltaLon(const DMS & base, const DMS & dest)
{
    double distance = (dest.dd - base.dd) * 85390.0f;
    distance += (dest.mm - base.mm) * 1420.0f;
    distance += (dest.ss - base.ss) * 23.6f;
    return distance;
}

Vec2d get_distance1(double latDest, double lngDest, double latOrg, double lngOrg)
{
    Vec2d vec;
    DMS latDestDms(latDest);
    DMS lngDestDms(lngDest);
    DMS latOrgDms(latOrg);
    DMS lngOrgDms(lngOrg);

    vec.x = DeltaLon(lngOrgDms, lngDestDms);
    vec.y = DeltaLat(latOrgDms, latDestDms);
    return vec;
}


