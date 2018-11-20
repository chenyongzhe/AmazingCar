#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <pcap.h>
#include <vector>
#include <stack>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h> 


#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#include<Windows.h>
#pragma comment(lib, "Ws2_32.lib")

#define CRS_SOCKET SOCKET
#define CRS_INVALID_SOCKET INVALID_SOCKET
#define CRS_SOCKET_ERROR SOCKET_ERROR
#elif __linux__
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "ros/ros.h"
#include "amazing_car/my_lidar_distance.h"


#define CRS_SOCKET int
#define CRS_INVALID_SOCKET -1
#define CRS_SOCKET_ERROR -1
#endif

#define PI 3.14159265357

class UdpClient
{
public:
	UdpClient() : m_sockFd(CRS_INVALID_SOCKET) {}
	~UdpClient() { Release(); }
	bool Init();
	bool Release();
	bool SendTo(const char * data, const size_t & length, const char * serverIp, const int & port);
	bool RecvFrom(char * data, size_t & length, char ** ppaddr, unsigned short * pport);

private:
	CRS_SOCKET m_sockFd;
	struct sockaddr_in m_servAddr;
	struct sockaddr_in m_sendAddr;
	char m_buffer[512];
	size_t m_bufferLength;
};

bool UdpClient::Init()
{
#ifdef _WIN32
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0) return false;
	m_sockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#elif __linux__
	m_sockFd = socket(AF_INET, SOCK_DGRAM, 0);
#endif
	if (m_sockFd == CRS_INVALID_SOCKET)
	{
		std::cout << "Create socket error." << std::endl;
		return false;
	}
	return true;
}

bool UdpClient::Release()
{
	if (m_sockFd != CRS_INVALID_SOCKET)
	{
#ifdef _WIN32
		closesocket(m_sockFd);
		WSACleanup();
#elif __linux__
		close(m_sockFd);
#endif
		m_sockFd = CRS_INVALID_SOCKET;
	}
	return true;
}

bool UdpClient::SendTo(const char * data, const size_t & length, const char * clientIp, const int & port)
{
	m_sendAddr.sin_family = AF_INET;
	m_sendAddr.sin_port = htons(port);
#ifdef _WIN32
	m_sendAddr.sin_addr.S_un.S_addr = inet_addr(clientIp);
#elif __linux__
	m_sendAddr.sin_addr.s_addr = inet_addr(clientIp);
	bzero(&(m_sendAddr.sin_zero), 8);
#endif

	int ret = sendto(m_sockFd, data, length, 0, (struct sockaddr*)&m_sendAddr, sizeof(m_sendAddr));
	if (ret == 0)
	{
		std::cout << "Connection closed." << std::endl;
		return false;
	}
	if (ret < 0)
	{
		std::cout << "Error." << std::endl;
		return false;
	}
	if (ret != static_cast<int>(length))
	{
		std::cout << "Send length error." << std::endl;
		return false;
	}
	return true;
}

bool UdpClient::RecvFrom(char * data, size_t & length, char ** ppaddr, unsigned short * pport)
{
	int addr_len = sizeof(m_servAddr);

	length = recvfrom(m_sockFd, data, length, 0/*MSG_DONTWAIT*/, (struct sockaddr*)&m_sendAddr,
#ifdef _WIN32
		/*(socklen_t*)*/&addr_len
#elif __linux__
	(socklen_t*)&addr_len
#endif
	);
	if (length == SIZE_MAX)
	{
		return false;
	}

	if (ppaddr)
	{
		//TODO : add support to get addr
#ifdef _WIN32
		inet_ntop(AF_INET, &m_sendAddr, *ppaddr, 0);
#elif __linux__
		*ppaddr = inet_ntoa(m_sendAddr.sin_addr);
#endif
	}

	if (pport)
		*pport = ntohs(m_sendAddr.sin_port);
	return true;
}

pcap_t * get_pcap_dev_handle(int ethernet_number) {
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	pcap_if_t *alldevs = NULL;
	pcap_if_t *d = NULL;
	int inum = 0;
	int i = 0;
	pcap_t *adhandle;
	char errbuf[PCAP_ERRBUF_SIZE];
	/* Retrieve the device list */
	if (pcap_findalldevs(&alldevs, errbuf) == -1) {
		fprintf(stderr, "Error in pcap_findalldevs: %s\n", errbuf);
		exit(1);
	}
	/* Print the list */
	for (d = alldevs; d; d = d->next) {
		printf("%d. %s", ++i, d->name);
		if (d->description)
			printf(" (%s)\n", d->description);
		else
			printf(" (No description available)\n");
	}
	if (i == 0) {
		printf("\nNo interfaces found! Make sure WinPcap is installed.\n");
		return nullptr;
	}
	printf("Enter the interface number (1-%d):", i);
	//scanf("%d", &inum);
	inum = ethernet_number;
	if (inum < 1 || inum > i) {
		printf("\nIntrface number out of range.\n");
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	/* Jump to the selected adapter */
	for (d = alldevs, i = 0; i< inum - 1; d = d->next, i++);
	/* Open the device */
	/* Open the adapter */
	if ((adhandle = pcap_open_live(d->name,   // 
		65536,         // 
					   // 65536 grants that the whole packet will be captured on all the MACs.
		1,             // 
		1000,          // 
		errbuf         // 
	)) == NULL) {
		fprintf(stderr, "\nUnable to open the adapter. %s is not supported by WinPcap\n", d->name);
		fprintf(stderr, "\n%s\n", errbuf);
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	printf("\nlistening on %s...\n\n", d->description);
	/* At this point, we don't need any more the device list. Free it */
	pcap_freealldevs(alldevs);
	return adhandle;
}

double *banewakeCE30D_vertical_angles = new double[20]{
	1.9,
	1.7,
	1.5,
	1.3,
	1.1,
	0.9,
	0.7,
	0.5,
	0.3,
	0.1,
	-0.1,
	-0.3,
	-0.5,
	-0.7,
	-0.9,
	-1.1,
	-1.3,
	-1.5,
	-1.7,
	-1.9
};

struct Grid_data
{
	int num;
	float grid_points[20][3];
	float min_distance;
	pcl::PointXYZRGBA clost_point;
	float max_distance;
	float total_distance;
	float total_horizontal_angle;
	float total_vertical_angle;
};

void grid_search(std::vector<std::vector<bool>> &visited, Grid_data grid[21][118], std::stack<std::pair<int, int>> &extend,
	float threshold, std::vector<std::pair<int, int>> &obstacle_grid_group, float &grids_avr_sum, float &grids_heri_angle_sum, float &grids_vert_angle_sum) {

	while (!extend.empty())
	{
		std::pair<int, int> cur = extend.top();
		extend.pop();
		//上
		if (cur.first - 1 >= 0 && cur.first - 1 < 21 && grid[cur.first - 1][cur.second].num >= 1 && visited[cur.first - 1][cur.second] == 0 &&
			fabs(grid[cur.first][cur.second].total_distance / grid[cur.first][cur.second].num - grid[cur.first - 1][cur.second].total_distance / grid[cur.first - 1][cur.second].num) < threshold) {
			extend.push(std::make_pair(cur.first - 1, cur.second));
			visited[cur.first - 1][cur.second] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first - 1][cur.second].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first - 1, cur.second));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first - 1, cur.second));
			grids_avr_sum += grid[cur.first - 1][cur.second].total_distance / grid[cur.first - 1][cur.second].num;
			grids_heri_angle_sum += grid[cur.first - 1][cur.second].total_horizontal_angle / grid[cur.first - 1][cur.second].num;
			grids_vert_angle_sum += grid[cur.first - 1][cur.second].total_vertical_angle / grid[cur.first - 1][cur.second].num;
		}
		//下
		if (cur.first + 1 >= 0 && cur.first + 1 < 21 && grid[cur.first + 1][cur.second].num >= 1 && visited[cur.first + 1][cur.second] == 0 &&
			fabs(grid[cur.first][cur.second].total_distance / grid[cur.first][cur.second].num - grid[cur.first + 1][cur.second].total_distance / grid[cur.first + 1][cur.second].num) < threshold) {
			extend.push(std::make_pair(cur.first + 1, cur.second));
			visited[cur.first + 1][cur.second] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first + 1][cur.second].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first + 1, cur.second));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first + 1, cur.second));
			grids_avr_sum += grid[cur.first + 1][cur.second].total_distance / grid[cur.first + 1][cur.second].num;
			grids_heri_angle_sum += grid[cur.first + 1][cur.second].total_horizontal_angle / grid[cur.first + 1][cur.second].num;
			grids_vert_angle_sum += grid[cur.first + 1][cur.second].total_vertical_angle / grid[cur.first + 1][cur.second].num;
		}
		//左
		if (cur.second - 1 >= 0 && cur.second - 1 < 118 && grid[cur.first][cur.second - 1].num >= 1 && visited[cur.first][cur.second - 1] == 0 &&
			fabs(grid[cur.first][cur.second].total_distance / grid[cur.first][cur.second].num - grid[cur.first][cur.second - 1].total_distance / grid[cur.first][cur.second - 1].num) < threshold) {
			extend.push(std::make_pair(cur.first, cur.second - 1));
			visited[cur.first][cur.second - 1] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first][cur.second - 1].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first, cur.second - 1));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first, cur.second - 1));
			grids_avr_sum += grid[cur.first][cur.second - 1].total_distance / grid[cur.first][cur.second - 1].num;
			grids_heri_angle_sum += grid[cur.first][cur.second - 1].total_horizontal_angle / grid[cur.first][cur.second - 1].num;
			grids_vert_angle_sum += grid[cur.first][cur.second - 1].total_vertical_angle / grid[cur.first][cur.second - 1].num;
		}
		//右
		if (cur.second + 1 >= 0 && cur.second + 1 < 118 && grid[cur.first][cur.second + 1].num >= 1 && visited[cur.first][cur.second + 1] == 0 &&
			fabs(grid[cur.first][cur.second].total_distance / grid[cur.first][cur.second].num - grid[cur.first][cur.second + 1].total_distance / grid[cur.first][cur.second + 1].num) < threshold) {
			extend.push(std::make_pair(cur.first, cur.second + 1));
			visited[cur.first][cur.second + 1] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first][cur.second + 1].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first, cur.second + 1));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first, cur.second + 1));
			grids_avr_sum += grid[cur.first][cur.second + 1].total_distance / grid[cur.first][cur.second + 1].num;
			grids_heri_angle_sum += grid[cur.first][cur.second + 1].total_horizontal_angle / grid[cur.first][cur.second + 1].num;
			grids_vert_angle_sum += grid[cur.first][cur.second + 1].total_vertical_angle / grid[cur.first][cur.second + 1].num;
		}
	}
}

std::vector<std::vector<float>> get_current_frame_CE30D( pcap_t * cur_device) {
	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	int count = 0;
	float maxDistance = 0.0;
	int maxFlectivity = 0;
	int block_count = 12;
	int block_size = 64;
	int head_size = 84;
	int channel_count = 20;
	int idcode_size = 2;
	int angle_size = 2;
	int unit_distance_size = 2;
	int unit_reflectivity_size = 1;
	int channel_size = unit_distance_size + unit_reflectivity_size;

	float *angles = new float[block_count];
	int *distance_mm = new int[channel_count*block_count];
	int *flectivity = new int[channel_count*block_count];

	const int grid_row = 21;
	const int grid_col = 118;
	float grid_left = -297.5, grid_right = 292.5;//cm
	float grid_bottom = -21, grid_top = 21;//cm
	bool check = false;
	int packet_loss_num = 0;

	//栅格显示分辨率(倍数相同)
	int display_hori = 590 * 6;  //590的倍数
	int display_vert = 42 * 6;    //42的倍数
	bool abnormal_signal = false;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;//异常点云保存
	cloud.is_dense = false;

	Grid_data grid[grid_row][grid_col] = { 0 };

	char buffer[858];
	bool bFirstPkt = true;
	int timept = 0;
	while (pcap_next_ex(cur_device, &pkthdr, &pktdata) >= 0) {
		if (pkthdr->caplen == 858) {

			if (pktdata == nullptr) {
				continue;
			}
			int* ptr = (int*)(pktdata + 64 * 12 + 84);

			if (bFirstPkt)
			{
				bFirstPkt = false;
			}
			else
			{
				if (timept == *ptr) {
					continue;
				}

			}
			timept = *ptr;
			memset(angles, 0, sizeof(float)*block_count);
			memset(distance_mm, 0, sizeof(int)*channel_count*block_count);
			memset(flectivity, 0, sizeof(int)*channel_count*block_count);

			for (int i = 0; i < block_count; ++i) {

				for (int k = angle_size - 1; k >= 0; --k) {
					int index = head_size + i*block_size + idcode_size + k;
					float data = pktdata[index];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100 - 30;

				for (int j = 0; j < channel_count; ++j) {

					float distance = 0;

					for (int k = unit_distance_size - 1; k >= 0; --k) {
						int index = head_size + i*block_size + idcode_size + angle_size + j*channel_size + k;
						float data = pktdata[index];
						distance_mm[channel_count*i + j] = distance_mm[channel_count*i + j] * 256 + data;
					}

					flectivity[channel_count*i + j] = pktdata[head_size + i*block_size + idcode_size + angle_size + j*channel_size + unit_distance_size];
					if (maxFlectivity < flectivity[channel_count*i + j]) {
						maxFlectivity = flectivity[channel_count*i + j];
					}

					distance = distance_mm[i*channel_count + j] * 2.0 / 1000;
					if (distance > maxDistance)
						maxDistance = distance;
					int flectivity_value = flectivity[i*channel_count + j];
					float horizontal_angle = angles[i] * PI / 180;
					float vertical_angle = banewakeCE30D_vertical_angles[j%channel_count] * PI / 180;

					pcl::PointXYZRGBA pclPoint;
					pclPoint.z = distance * sin(vertical_angle);
					pclPoint.y = distance * cos(vertical_angle) * sin(-horizontal_angle);
					pclPoint.x = distance * cos(vertical_angle) * cos(horizontal_angle);
					pclPoint.r = 0;
					pclPoint.b = 255;
					pclPoint.g = 0;

					if (pclPoint.x <= 5.0f&&distance>0.0f) {

						pcl::PointXYZRGBA new_point;
						float distance_expend = 5.0f / cos(horizontal_angle) / cos(vertical_angle);
						new_point.z = distance_expend*sin(vertical_angle);
						new_point.y = distance_expend*cos(vertical_angle)*sin(-horizontal_angle);
						new_point.x = 5.0f;

						int grid_cur_row = (new_point.z * 100 - grid_bottom) / 2;
						int grid_cur_col = (new_point.y * 100 - grid_left) / 5;

						grid[grid_cur_row][grid_cur_col].grid_points[grid[grid_cur_row][grid_cur_col].num][0] = pclPoint.x;
						grid[grid_cur_row][grid_cur_col].grid_points[grid[grid_cur_row][grid_cur_col].num][1] = pclPoint.y;
						grid[grid_cur_row][grid_cur_col].grid_points[grid[grid_cur_row][grid_cur_col].num][2] = pclPoint.z;

						if (grid[grid_cur_row][grid_cur_col].num == 0) {
							grid[grid_cur_row][grid_cur_col].min_distance = distance;
							grid[grid_cur_row][grid_cur_col].clost_point = pclPoint;
							grid[grid_cur_row][grid_cur_col].max_distance = distance;
						}
						else {
							if (grid[grid_cur_row][grid_cur_col].max_distance < distance)
								grid[grid_cur_row][grid_cur_col].max_distance = distance;
							if (grid[grid_cur_row][grid_cur_col].min_distance > distance) {

								grid[grid_cur_row][grid_cur_col].min_distance = distance;
								grid[grid_cur_row][grid_cur_col].clost_point = pclPoint;

							}

						}
						grid[grid_cur_row][grid_cur_col].total_distance += distance;
						grid[grid_cur_row][grid_cur_col].total_horizontal_angle += angles[i];
						grid[grid_cur_row][grid_cur_col].total_vertical_angle += banewakeCE30D_vertical_angles[j%channel_count];
						grid[grid_cur_row][grid_cur_col].num += 1;

					}
				}
			}
			count++;

			if (count == 27) {
	
				std::stack<std::pair<int, int>> extend;
				float threshold = 0.15;
				float grids_avr_sum = 0;
				float grids_heri_angle_sum = 0;
				float grids_vert_angle_sum = 0;

				std::vector<std::vector<bool>> visited(21, std::vector<bool>(118, 0));
				std::vector<std::vector<std::pair<int, int>>> obstacle_grid;
				std::vector<std::pair<int, int>> obstacle_grid_group;
				std::vector<std::vector<float>> grids_obstacle_datas;
				std::vector<float> grids_obstacle_data;

				for (int i = 0; i < grid_row; ++i) {
					for (int j = 0; j < grid_col; ++j) {
						if (grid[i][j].num < 1 || visited[i][j])
							continue;

						grids_avr_sum = 0.0f;
						grids_heri_angle_sum = 0.0f;
						grids_vert_angle_sum = 0.0f;
						extend.push(std::make_pair(i, j));
						obstacle_grid_group.push_back(std::make_pair(i, j));
						grids_avr_sum = grid[i][j].total_distance / grid[i][j].num;
						grids_heri_angle_sum = grid[i][j].total_horizontal_angle / grid[i][j].num;
						grids_vert_angle_sum = grid[i][j].total_vertical_angle / grid[i][j].num;
						visited[i][j] = 1;

						grid_search(visited, grid, extend, threshold, obstacle_grid_group, grids_avr_sum, grids_heri_angle_sum, grids_vert_angle_sum);
						if (obstacle_grid_group.size() >= 3) {
							obstacle_grid.push_back(obstacle_grid_group);
							grids_obstacle_data.push_back(grids_avr_sum / obstacle_grid_group.size());
							grids_obstacle_data.push_back(grids_heri_angle_sum / obstacle_grid_group.size());
							grids_obstacle_data.push_back(grids_vert_angle_sum / obstacle_grid_group.size());
							grids_obstacle_datas.push_back(grids_obstacle_data);
							grids_obstacle_data.clear();
						}

						obstacle_grid_group.clear();
					}
				}

				delete[] angles;
				delete[] distance_mm;
				delete[] flectivity;
				//break;

				return grids_obstacle_datas;
			}
		}
	}
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "lidar");
	ros::NodeHandle n;
	ros::Publisher lidar_pub = n.advertise<amazing_car::my_lidar_distance>("beixing_lidar", 1000);	
	ros::Rate rate(120);

	int count = 0;
	
 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
 	pcap_t * device = get_pcap_dev_handle(1);
	

	UdpClient udpclient;
	if (!udpclient.Init())
	{
		printf("Init error.\n");
	}
 
 	char buffer[50] = "version";
 	char dest_ip[] = "192.168.1.80";
	char* pdest_ip = dest_ip;
 	unsigned short dest_port = 2368;
 
	int temp = udpclient.SendTo(buffer, 50, dest_ip, dest_port);
	char version_str[7] = { 0 };
	size_t length = 6;
	udpclient.RecvFrom(version_str,length,&pdest_ip,&dest_port);
	
	printf("Version : %s\n", version_str);

 	strcpy(buffer, "getDistanceAndAmplitudeSorted");
	udpclient.SendTo(buffer, 50, "192.168.1.80", 2368);
	memset(buffer, 0, sizeof(buffer));
	strcpy(buffer, "updateTimeStamp 00 00 00 00");
	udpclient.SendTo(buffer, 50, "192.168.1.80", 2368);
	
	while (ros::ok())
	{

		if (count % 3000 == 0) {
			memset(buffer, 0, sizeof(buffer));
			strcpy(buffer, "updateTimeStamp 00 00 00 00");
			udpclient.SendTo(buffer, 50, "192.168.1.80", 2368);
		}	

		std::vector<std::vector<float>> datas=get_current_frame_CE30D(device);//return vector<vector<float>>  distance,hori_angle,vert_angle
		float distance = 100000;
		float angle = -360;
		for(int i=0;i<datas.size();++i){
		   	printf("no.%d  distance:%f, hori_angle:%f, vert_angle:%f\n",i,datas[i][0],datas[i][1],datas[i][2]);
		   	if(datas[i][0] < distance){
				distance = datas[i][0];
				angle = datas[i][1];
			}
		}
		count++;
		amazing_car::my_lidar_distance lidar_msg;
		lidar_msg.min_distance = distance;
		lidar_msg.angle = angle;
		lidar_pub.publish(lidar_msg);
	}

	return 1;
}
