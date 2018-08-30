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

volatile bool bDataSignal = false;

volatile bool mtx = true;

volatile bool server_alive = false;

volatile bool server_connected = false;

volatile bool wtkey = true;

#define wait(b) do { while(!b);b = false;}while(0)

extern int errno;

struct CarData
{
	float x,y,angle;
};

std::vector<CarData> v;
std::thread thrd;

void NetServer(const char* ipAddr,unsigned int port)
{
re_connect:

	int server = socket(AF_INET,SOCK_STREAM,0);

	sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ipAddr);
    addr.sin_port = htons(port);

    while( -1 == connect(server,(sockaddr*)&addr,sizeof(addr))){
    	printf("%d\n",errno);
    	usleep(1000000);
    }
    server_connected = true;

    while(true)
    {
    	wait(bDataSignal);
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

void Connect(const char* ipaddr,unsigned int port)
{
	if(server_alive)return;
	server_alive = true;
	thrd = std::thread(NetServer,ipaddr,port);
}

void SendCarData(float x,float y,float angle)
{
	wait(wtkey);
	wait(mtx);
	wtkey = true;
	v.push_back({x,y,angle});
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

/*

int main(int argc, char const *argv[])
{
	float X = 0.0f,Y = 0.0f;
	Connect("127.0.0.1");
	while(true)
	{
		float dx = (float)rand()/(float)RAND_MAX - 0.5f;
		float dy = (float)rand()/(float)RAND_MAX - 0.5f;

		X += dx * 0.1f;
		Y += dy * 0.1f;
		printf("(%f,%f)\n",X,Y);
		fflush(stdout);
		SendCarData(X,Y,acos(dy / sqrtf(dx * dx + dy * dy)) * (dx > 0.0f?-1.0f:1.0f));
		usleep(30000);
	}
	return 0;
}

*/
