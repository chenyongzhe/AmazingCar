#include "ros/ros.h"
#include "amazing_car/my_location_msg.h"
#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_gps_state.h"
#include "stdio.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "amazing_car/my_angle_msg.h"
#include "amazing_car/my_gps_state.h"

#include "serial/serial.h"
#include <math.h>

using std::string;

float gps_angle = 0;
float diff1 = 0;
int gps_state;
ros::Publisher * angle_pub_ptr = NULL;
unsigned char temp[11] = {0};
int gyroscope_flag = 0;

void DecodeIMUData();
double a[3],w[3],Angle[3],T;
int cmd_count = 0;
int sum_count = 0;

void callback_car_angle(amazing_car::my_angle_msg msg){
	gps_angle = msg.yaw;
}

void callback_gps_state(amazing_car::my_gps_state msg){
	gps_state = msg.location_state;
}

serial::Serial my_serial("/dev/ttyUSB3", 9600, serial::Timeout::simpleTimeout(1000));


unsigned char hex2char(std::string const &s);
std::string char2hex(std::string const &s);
void decode(std::string const &s);

int main(int argc, char ** argv){
	ros::init(argc, argv, "gyroscope");
	ros::NodeHandle n;
	ros::Publisher angle_pub = n.advertise<amazing_car::my_angle_msg>("my_car_angle", 1000);
	angle_pub_ptr = &angle_pub;
	ros::Subscriber angle_sub = n.subscribe("my_car_angle", 1000, callback_car_angle);
	ros::Subscriber gps_state_sub = n.subscribe("my_gps_state", 1000, callback_gps_state);
	ros::Rate rate(200);

	while(ros::ok()){
		
		amazing_car::my_gps_state state_msg;
	   	//get data
	   	std::string data = "";
		//usleep(2000);		
		//read_data(fd, read_buffer);
		//my_serial.available()
		data = my_serial.read(1);
		decode(data);
		sum_count++;
		//DecodeIMUData(read_buffer);
		//data = char2hex(data);

                //printf("read: %s\n", data.c_str());
		
		//my_serial1.readline(data);
           	//printf("%s\n", data.c_str());	
		ros::spinOnce();
		//rate.sleep();
	}
	return 0;
}


void DecodeIMUData()
{
	switch(temp[1])
	{
	
	case 0x51:
		a[0] = (short(temp[3]<<8|temp[2]))/32768.0*16;
		a[1] = (short(temp[5]<<8|temp[4]))/32768.0*16;
		a[2] = (short(temp[7]<<8|temp[6]))/32768.0*16;
		//T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		//printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n",a[0],a[1],a[2]);
		break;
	case 0x52:
		w[0] = (short(temp[3]<<8|temp[2]))/32768.0*2000;
		w[1] = (short(temp[5]<<8|temp[4]))/32768.0*2000;
		w[2] = (short(temp[7]<<8|temp[6]))/32768.0*2000;
		//T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		//printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n",w[0],w[1],w[2]);
		break;
	
	case 0x53:
		Angle[0] = (short(temp[3]<<8|temp[2]))/32768.0*180;
		Angle[1] = (short(temp[5]<<8|temp[4]))/32768.0*180;
		Angle[2] = (short(temp[7]<<8|temp[6]))/32768.0*180;


		float transed_angle = 0;
		if(Angle[2] < 0){
			transed_angle = -1 * Angle[2];
		}else{
			transed_angle = 360 - Angle[2];
		}


		if(gps_state != 4){
			amazing_car::my_angle_msg angle_msg;
			transed_angle += diff1;
			if(transed_angle >= 360){
				transed_angle -= 360;
			}
			if(transed_angle < 0){
				transed_angle += 360;
			}
			angle_msg.yaw = transed_angle;
			angle_pub_ptr->publish(angle_msg);
			printf("AAA gps = %4.2f ori = %4.2f diff = %4.2f Angle = %4.2f\r\n",gps_angle, Angle[2], diff1, transed_angle);
		}else{
			diff1 = gps_angle - transed_angle;
			printf("gps = %4.2f ori = %4.2f diff = %4.2f Angle = %4.2f\r\n",gps_angle, Angle[2], diff1, transed_angle + diff1);
		}
		
		//T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
		//printf("Angle = %4.2f\t%4.2f\t%4.2f\tT=%4.2f\r\n",Angle[0],Angle[1],Angle[2],T);
		
		break;
	}
}

void decode(std::string const &data){
	std::string str = char2hex(data);
	//printf("no:%d  %s\n", sum_count, str.c_str());
	if(strcmp(str.c_str(),"00") == 0){
		//return;
	}
	if(strcmp(str.c_str(),"0x55") == 0){
		cmd_count = 0;
		for(int i = 0;i<11;i++){
			temp[i] = 0;
		}
		gyroscope_flag = 1;
		temp[0] = hex2char("0x55");
		//printf("sum:%d cmd:%d temp[%d]:%s\n",sum_count,cmd_count,cmd_count,str.c_str());
		return;
	}
	if(gyroscope_flag == 1){
		//printf("%d\n", cmd_count);
		
		cmd_count ++;
		temp[cmd_count] = hex2char(str);
		//printf("sum:%d cmd:%d temp[%d]:%s\n",sum_count,cmd_count,cmd_count,str.c_str());
		//if(cmd_count == 1 && (temp[cmd_count] == 0x51 || temp[cmd_count] == 0x52)){
		//	gyroscope_flag = 0;
		//	return;
		//}

		//printf("%d\n", hex2char(char2hex(str)));
		if(cmd_count == 10){
			if(temp[1] == 0x53){
				int sum_check = 0;
				for(int i = 0;i<11;i++){
					//printf(" %d", temp[i]);
					//sum_check += temp[i];
				}
				//printf("  check: %d", sum_check);
				//printf("\n");
			}
			
			DecodeIMUData();
			for(int i = 0;i<11;i++){
				temp[i] = 0;
			}
			cmd_count = 0;
		}
	}
}


unsigned char hex2char(std::string const &s)
{
    	short high = 0;
    	short low = 0;
	unsigned char res = 0;
	//printf("a : %s\n",s.c_str());
	switch(s.c_str()[2]){
		case '0':high = 0;break;
		case '1':high = 16;break;
		case '2':high = 32;break;
		case '3':high = 48;break;
		case '4':high = 64;break;
		case '5':high = 80;break;
		case '6':high = 96;break;
		case '7':high = 112;break;
		case '8':high = 128;break;
		case '9':high = 144;break;
		case 'a':high = 160;break;
		case 'b':high = 176;break;
		case 'c':high = 192;break;
		case 'd':high = 208;break;
		case 'e':high = 224;break;
		case 'f':high = 240;break;
		default:high = 0;break;
	}

	switch(s.c_str()[3]){
		case '0':low = 0;break;
		case '1':low = 1;break;
		case '2':low = 2;break;
		case '3':low = 3;break;
		case '4':low = 4;break;
		case '5':low = 5;break;
		case '6':low = 6;break;
		case '7':low = 7;break;
		case '8':low = 8;break;
		case '9':low = 9;break;		
		case 'a':low = 10;break;
		case 'b':low = 11;break;
		case 'c':low = 12;break;
		case 'd':low = 13;break;
		case 'e':low = 14;break;
		case 'f':low = 15;break;
		default:low = 0;break;
	}
	res = low + high;
   	return res;
}


std::string char2hex(std::string const &s)
{
    std::string ret;
    //printf("s:%s\n",s.c_str());
    for (unsigned i = 0; i != s.size(); ++i)
    {
        char hex[5];
        sprintf(hex, "%#.2x", (unsigned char)s[i]);
        ret += hex;
    }
    //printf("ret:%s\n",ret.c_str());
    return ret;
}
