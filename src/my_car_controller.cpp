#include "ros/ros.h"
//#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>


// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


#define TRUE 0
#define FALSE -1
#define DEFAULT_DEV "/dev/ttyACM0"

#define DEFAULT_BAUND 115200
#define DEFAULT_DATABIT CS8
#define DEFAULT_PAR ~PARENB
#define DEFAULT_INP ~INPCK
#define DEFAULT_STOPBIT ~CSTOPB
#define DEFAULT_INTERVAL 11

#define STR_NUM 9

#define DEBUG

#ifdef DEBUG
#define READ_BUFFER_SIZE 32
#endif

#ifndef FROM_ARGV
//static const char *pstr[] = {0, "AT", "AT", "AT+CMGF=1", "AT+CMGS=", "\"", "13474669578", "\"", ";\r", (char *)26};
#else
#define pstr argv
#endif

int open_dev(const char *dev_name);
int set_port(const int fd);
int send_data(const int fd, const char *buffer, const int buffer_len);
#ifdef DEBUG
int read_data(const int fd, char *read_buffer);
#endif


using std::string;

//serial::Serial my_serial("/dev/ttyUSB0", 256000, serial::Timeout::simpleTimeout(1000));

//std::string test_string;
//char * test_string = new char[10];

int fd;
int i;
char *dev_name = DEFAULT_DEV;
ros::Rate * rate;
int left = 200;
int right = 200;
void callback(const geometry_msgs::Twist& cmd_vel){
	left = cmd_vel.linear.x;
	right = cmd_vel.linear.y;
	//printf("%s\n", test_string.c_str());
	//char * test_string = new char[4];
	//sprintf(test_string,"%d@%d#\0",200,200);
	//sprintf(test_string,"%d@%d#",left,right);
	//printf("%s\n",test_string);
	
}

std::string get_res(){

	if(left > 344 && left <= 350 && right >= 200 && right < 206){
		return "a";
	}else if(left > 338 && left <= 344 && right >= 206 && right < 212){
		return "b";
	}else if(left > 332 && left <= 338 && right >= 212 && right < 218){
		return "c";
	}else if(left > 326 && left <= 332 && right >= 218 && right < 224){
		return "d";
	}else if(left > 320 && left <= 326 && right >= 224 && right < 230){
		return "e";
	}else if(left > 314 && left <= 320 && right >= 230 && right < 236){
		return "f";
	}else if(left > 308 && left <= 314 && right >= 236 && right < 242){
		return "g";
	}else if(left > 302 && left <= 308 && right >= 242 && right < 248){
		return "h";
	}else if(left > 296 && left <= 302 && right >= 248 && right < 254){
		return "i";
	}else if(left > 290 && left <= 296 && right >= 254 && right < 260){
		return "j";
	}else if(left > 284 && left <= 290 && right >= 260 && right < 266){
		return "k";
	}else if(left > 278 && left <= 284 && right >= 266 && right < 272){
		return "l";
	}else if(left > 272 && left <= 278 && right >= 272 && right < 278){
		return "m";
	}else if(left > 266 && left <= 272 && right >= 278 && right < 284){
		return "n";
	}else if(left > 260 && left <= 266 && right >= 284 && right < 290){
		return "o";
	}else if(left > 254 && left <= 260 && right >= 290 && right < 296){
		return "p";
	}else if(left > 248 && left <= 254 && right >= 296 && right < 302){
		return "q";
	}else if(left > 242 && left <= 248 && right >= 302 && right < 308){
		return "r";
	}else if(left > 236 && left <= 242 && right >= 308 && right < 314){
		return "s";
	}else if(left > 230 && left <= 236 && right >= 314 && right < 320){
		return "t";
	}else if(left > 224 && left <= 230 && right >= 320 && right < 326){
		return "u";
	}else if(left > 218 && left <= 224 && right >= 326 && right < 332){
		return "v";
	}else if(left > 212 && left <= 218 && right >= 332 && right < 338){
		return "w";
	}else if(left > 206 && left <= 212 && right >= 338 && right < 344){
		return "x";
	}else if(left > 200 && left <= 206 && right >= 344 && right < 350){
		return "y";
	}else if(left == 200 && right == 350){
		return "z";
	}

	else if(left == 50 && right == 350){
		//left
		return "!";
	}else if(left == 350 && right == 50){
		//right
		return "@";
	}else if(left == 350 && right == 350){
		//straight
		return "#";
	}

}

int main(int argc, char ** argv){
	ros::init(argc, argv, "my_car_contoller");
	ros::NodeHandle n;
	rate = new ros::Rate(25);
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);


	if((fd = open_dev(dev_name)) == FALSE){
		perror("open error!");
		return -1;
	}

	if(set_port(fd) == FALSE){
		perror("set error!");
		return -1;
	}
	sleep(2);


	char read_buffer[READ_BUFFER_SIZE];
		
	//read_data(fd,read_buffer);
	//printf("%s\n", read_buffer);
	while(ros::ok()){
		//std::ostringstream stream;
		//stream << left << "a" << right << "#";
		std::string test_string = get_res();
		//std::string test_string = stream.str();
		//printf("%s\n", test_string.c_str());
		//test_string = "a";
	   	send_data(fd, test_string.c_str(), strlen(test_string.c_str()));
		usleep(20000);
                
                read_data(fd, read_buffer);
                printf("read: %s", read_buffer);
		usleep(20000);
		ros::spinOnce();
	}

	return 0;
}



int open_dev(const char *dev_name){
	return open(dev_name, O_RDWR);
}

int set_port(const int fd){
	struct termios opt;
	if(tcgetattr(fd, &opt) != 0){
		return FALSE;
	}
	cfsetispeed(&opt, DEFAULT_BAUND);
	cfsetospeed(&opt,DEFAULT_BAUND);
	tcsetattr(fd,TCSANOW,&opt);
	opt.c_cflag &= ~CSIZE;
	opt.c_cflag |= DEFAULT_DATABIT;
	opt.c_cflag &= DEFAULT_PAR;
	opt.c_iflag &= DEFAULT_INP;
	opt.c_cflag &= DEFAULT_STOPBIT;

	tcflush(fd, TCIFLUSH);
	opt.c_cc[VTIME] = DEFAULT_INTERVAL;
	opt.c_cc[VMIN] = 0;
	if(tcsetattr(fd, TCSANOW, &opt) != 0){
		return FALSE;
	}
	return TRUE;
}

int send_data(const int fd, const char *buffer, const int buffer_len){
	return write(fd, buffer, buffer_len);
}

#ifdef DEBUG
int read_data(const int fd, char *read_buffer){
	return read(fd, read_buffer, READ_BUFFER_SIZE);
}
#endif
