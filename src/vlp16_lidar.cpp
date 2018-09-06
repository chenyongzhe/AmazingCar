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

#include "ros/ros.h"
#include "amazing_car/my_lidar_distance.h"


#define PI 3.14159265357

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
	scanf("%d", &inum);
	//inum = ethernet_number;
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
		/* Free the device list */
		pcap_freealldevs(alldevs);
		return nullptr;
	}
	printf("\nlistening on %s...\n\n", d->description);
	/* At this point, we don't need any more the device list. Free it */
	pcap_freealldevs(alldevs);
	return adhandle;
}

double *vld16_vertical_angles = new double[16]{
	-15,
	1,
	-13,
	3,
	-11,
	5,
	-9,
	7,
	-7,
	9,
	-5,
	11,
	-3,
	13,
	-1,
	15
};

struct Point
{
	float x, y, z;
	unsigned char b, g, r, a;
};

struct Grid_data
{
	struct Point3F {
		float x, y, z;
		Point3F() {};
		template<typename T>
		Point3F(const T& v) :x(v.x), y(v.y), z(v.z) {}
		Point3F(float _x, float _y, float _z) :x(_x), y(_y), z(_z) { }
		float& operator[](int n) {
			switch (n)
			{
			case 0:return x;
			case 1:return y;
			case 2:return z;
			default:return x;
			}
		}
	};
	std::vector<Point3F> grid_points;
	float min_distance = 0.0f;
	Point clost_point = { 0 };
	float max_distance = 0.0f;
	float total_distance = 0.0f;
	float total_z = 0.0f;
	float total_horizontal_angle = 0.0f;
	float total_vertical_angle = 0.0f;
};

template<size_t Mx, size_t Ny>
void grid_search(std::vector<std::vector<bool>> &visited, Grid_data(&grid)[Mx][Ny], std::stack<std::pair<int, int>> &extend,
	float threshold, std::vector<std::pair<int, int>> &obstacle_grid_group, float &grids_avr_sum, float &grids_heri_angle_sum, float &grids_vert_angle_sum, int &points_sum) {

	while (!extend.empty())
	{
		std::pair<int, int> cur = extend.top();
		extend.pop();
		//上
		//fabs(grid[cur.first][cur.second].total_z / grid[cur.first][cur.second].grid_points.size() - grid[cur.first + 1][cur.second].total_z / grid[cur.first + 1][cur.second].grid_points.size()) < threshold
		if (cur.first - 1 >= 0 && cur.first - 1 < Mx && grid[cur.first - 1][cur.second].grid_points.size() >= 1 && visited[cur.first - 1][cur.second] == 0 ) {
			extend.push(std::make_pair(cur.first - 1, cur.second));
			visited[cur.first - 1][cur.second] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first - 1][cur.second].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first - 1, cur.second));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first - 1, cur.second));
			grids_avr_sum += grid[cur.first - 1][cur.second].total_distance;
			grids_heri_angle_sum += grid[cur.first - 1][cur.second].total_horizontal_angle;
			grids_vert_angle_sum += grid[cur.first - 1][cur.second].total_vertical_angle;
			points_sum += grid[cur.first - 1][cur.second].grid_points.size();
		}
		//下
		if (cur.first + 1 >= 0 && cur.first + 1 < Mx && grid[cur.first + 1][cur.second].grid_points.size() >= 1 && visited[cur.first + 1][cur.second] == 0 ) {
			extend.push(std::make_pair(cur.first + 1, cur.second));
			visited[cur.first + 1][cur.second] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first + 1][cur.second].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first + 1, cur.second));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first + 1, cur.second));
			grids_avr_sum += grid[cur.first + 1][cur.second].total_distance;
			grids_heri_angle_sum += grid[cur.first + 1][cur.second].total_horizontal_angle;
			grids_vert_angle_sum += grid[cur.first + 1][cur.second].total_vertical_angle;
			points_sum += grid[cur.first + 1][cur.second].grid_points.size();
		}
		//左
		if (cur.second - 1 >= 0 && cur.second - 1 < Ny && grid[cur.first][cur.second - 1].grid_points.size() >= 1 && visited[cur.first][cur.second - 1] == 0 ) {
			extend.push(std::make_pair(cur.first, cur.second - 1));
			visited[cur.first][cur.second - 1] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first][cur.second - 1].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first, cur.second - 1));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first, cur.second - 1));
			grids_avr_sum += grid[cur.first][cur.second - 1].total_distance;
			grids_heri_angle_sum += grid[cur.first][cur.second - 1].total_horizontal_angle;
			grids_vert_angle_sum += grid[cur.first][cur.second - 1].total_vertical_angle;
			points_sum += grid[cur.first][cur.second - 1].grid_points.size();
		}
		//右
		if (cur.second + 1 >= 0 && cur.second + 1 < Ny && grid[cur.first][cur.second + 1].grid_points.size() >= 1 && visited[cur.first][cur.second + 1] == 0) {
			extend.push(std::make_pair(cur.first, cur.second + 1));
			visited[cur.first][cur.second + 1] = 1;
			if (grid[obstacle_grid_group[0].first][obstacle_grid_group[0].second].min_distance > grid[cur.first][cur.second + 1].min_distance)
				obstacle_grid_group.insert(obstacle_grid_group.begin(), std::make_pair(cur.first, cur.second + 1));
			else
				obstacle_grid_group.push_back(std::make_pair(cur.first, cur.second + 1));
			grids_avr_sum += grid[cur.first][cur.second + 1].total_distance;
			grids_heri_angle_sum += grid[cur.first][cur.second + 1].total_horizontal_angle;
			grids_vert_angle_sum += grid[cur.first][cur.second + 1].total_vertical_angle;
			points_sum += grid[cur.first][cur.second + 1].grid_points.size();
		}
	}
}

std::vector<std::vector<float>> get_current_frame_vld16( pcap_t * cur_device, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & scene) {
	scene->clear();

	pcap_pkthdr *pkthdr = 0;
	const u_char *pktdata = 0;
	int count = 0;
	float maxDistance = 0.0;
	int maxFlectivity = 0;
	int block_count = 12;
	int block_size = 100;
	int head_size = 42;
	int channel_count = 32;
	int idcode_size = 2;
	int angle_size = 2;
	int unit_distance_size = 2;
	int unit_reflectivity_size = 1;
	int channel_size = unit_distance_size + unit_reflectivity_size;

	float *angles = new float[block_count*2];
	int *distance_mm = new int[channel_count*block_count];
	int *flectivity = new int[channel_count*block_count];

	const int grid_row = 100;
	const int grid_col = 200;
	float grid_left = -500.0, grid_right = 500.0;//cm
	float grid_bottom = 0.0, grid_top = 500.0;//cm
	bool check = false;

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;//异常点云保存
	cloud.is_dense = false;

	struct Grid_data_init
	{
		Grid_data grid[grid_row][grid_col];

	};
	static Grid_data_init *tmp = new Grid_data_init();
	auto& grid = tmp->grid;
	for (int i = 0; i < grid_row; i++)
		for (int j = 0; j < grid_col; j++)
		{
			grid[i][j].clost_point = { 0 };
			grid[i][j].max_distance = 0.0f;
			grid[i][j].min_distance = 0.0f;
			grid[i][j].grid_points.clear();
			grid[i][j].total_distance = 0.0f;
			grid[i][j].total_horizontal_angle = 0.0f;
			grid[i][j].total_vertical_angle = 0.0f;
			grid[i][j].total_z = 0.0f;
		}

	while (pcap_next_ex(cur_device, &pkthdr, &pktdata) >= 0) {
		if (pkthdr->caplen == 1248) {

			if (pktdata == nullptr) {
				continue;
			}

			memset(angles, 0, sizeof(float)*block_count*2);
			memset(distance_mm, 0, sizeof(int)*channel_count*block_count);
			memset(flectivity, 0, sizeof(int)*channel_count*block_count);

			for (int i = 0; i < block_count * 2; i += 2) {

				for (int k = angle_size - 1; k >= 0; --k) {
					int index = head_size + i / 2 * block_size + idcode_size + k;
					float data = pktdata[index];
					angles[i] = angles[i] * 256 + data;
				}
				angles[i] = angles[i] / 100;

			}

			for (int i = 1; i < block_count * 2 - 1; i += 2) {

				if (angles[i - 1] > angles[i + 1])
					angles[i] = (angles[i - 1] + angles[i + 1] + 360) / 2;
				else
					angles[i] = (angles[i - 1] + angles[i + 1]) / 2;
				if (angles[i] > 360)
					angles[i] -= 360;
			}

			int last_angle_index = block_count * 2 - 1;
			angles[last_angle_index] = angles[last_angle_index - 1] * 2 - angles[last_angle_index - 2];
			if (angles[last_angle_index] > 360)
				angles[last_angle_index] -= 360;

			for (int i = 0; i < block_count; ++i) {
				for (int j = 0; j < channel_count; ++j) {

					float distance = 0.0f;

					for (int k = unit_distance_size - 1; k >= 0; --k) {
						int index = head_size + i*block_size + idcode_size + angle_size + j*channel_size + k;
						float data = pktdata[index];
						distance_mm[channel_count*i + j] = distance_mm[channel_count*i + j] * 256 + data;
					}

					flectivity[channel_count*i + j] = pktdata[head_size + i*block_size + idcode_size + angle_size + j*channel_size + unit_distance_size];
					if (maxFlectivity < flectivity[channel_count*i + j]) {
						maxFlectivity = flectivity[channel_count*i + j];
					}

					distance = distance_mm[i*channel_count + j] * 2/ 1000.0f;
					if (distance > maxDistance)
						maxDistance = distance;
					int flectivity_value = flectivity[i*channel_count + j];
					float horizontal_angle = angles[i * 2 + j / 16] * PI / 180;
					float vertical_angle = vld16_vertical_angles[j % 16] * PI / 180;

					Point pclPoint;
					pclPoint.z = distance * sin(vertical_angle);
					pclPoint.y = distance * cos(vertical_angle) * cos(horizontal_angle);
					pclPoint.x = distance * cos(vertical_angle) * sin(horizontal_angle);
					pclPoint.r = 0;
					pclPoint.b = 255;
					pclPoint.g = 0;
					//######################################################################################################################################
					if (pclPoint.y >= 0.0f&&pclPoint.y <= 5.0f&&distance > 0.0f && pclPoint.x >= -5.0f&&pclPoint.x <= 5.0f&&pclPoint.z >= -0.7f) {

						int grid_cur_row = (pclPoint.y * 100 - grid_bottom) / 5;
						int grid_cur_col = (pclPoint.x * 100 - grid_left) / 5;

						grid[grid_cur_row][grid_cur_col].grid_points.push_back({ pclPoint.x, pclPoint.y, pclPoint.z });

						if (grid[grid_cur_row][grid_cur_col].grid_points.size() == 1) {
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
						grid[grid_cur_row][grid_cur_col].total_z += pclPoint.z;
						grid[grid_cur_row][grid_cur_col].total_distance += distance;
						if (angles[i] >= 270)
							grid[grid_cur_row][grid_cur_col].total_horizontal_angle += angles[i] - 360;
						else
							grid[grid_cur_row][grid_cur_col].total_horizontal_angle += angles[i];
						grid[grid_cur_row][grid_cur_col].total_vertical_angle += vld16_vertical_angles[j%16];

					}
				}
			}
			count++;

			if (count >= 76) {
	
				std::stack<std::pair<int, int>> extend;
				float threshold = 0.2;
				float grids_avr_sum = 0;
				float grids_heri_angle_sum = 0;
				float grids_vert_angle_sum = 0;
				int points_sum = 0;

				std::vector<std::vector<bool>> visited(grid_row, std::vector<bool>(grid_col, 0));
				std::vector<std::vector<std::pair<int, int>>> obstacle_grid;
				std::vector<std::pair<int, int>> obstacle_grid_group;
				std::vector<std::vector<float>> grids_obstacle_datas;
				std::vector<float> grids_obstacle_data;

				for (int i = 0; i < grid_row; ++i) {
					for (int j = 0; j < grid_col; ++j) {
						if (grid[i][j].grid_points.size() < 1 || visited[i][j])
							continue;

						grids_avr_sum = 0.0f;
						grids_heri_angle_sum = 0.0f;
						grids_vert_angle_sum = 0.0f;
						points_sum = 0;
						extend.push(std::make_pair(i, j));
						obstacle_grid_group.push_back(std::make_pair(i, j));
						grids_avr_sum = grid[i][j].total_distance;
						grids_heri_angle_sum = grid[i][j].total_horizontal_angle;
						grids_vert_angle_sum = grid[i][j].total_vertical_angle;
						points_sum = grid[i][j].grid_points.size();
						visited[i][j] = 1;

						grid_search(visited, grid, extend, threshold, obstacle_grid_group, grids_avr_sum, grids_heri_angle_sum, grids_vert_angle_sum, points_sum);
						if (points_sum >= 30) {
							obstacle_grid.push_back(obstacle_grid_group);
							grids_obstacle_data.push_back(grids_avr_sum / points_sum);
							grids_obstacle_data.push_back(grids_heri_angle_sum / points_sum);
							grids_obstacle_data.push_back(grids_vert_angle_sum / points_sum);
							grids_obstacle_datas.push_back(grids_obstacle_data);
							grids_obstacle_data.clear();
						}

						obstacle_grid_group.clear();
					}
				}

				srand((unsigned)time(NULL));
				pcl::PointXYZRGBA point_view;
				for (int i = 0; i < obstacle_grid.size(); ++i) {

					int r = rand() % 255;
					int g = rand() % 255;
					int b = rand() % 255;
					for (int j = 0; j < obstacle_grid[i].size(); ++j) {
						for (int k = 0; k < grid[obstacle_grid[i][j].first][obstacle_grid[i][j].second].grid_points.size(); ++k) {
							point_view.x = grid[obstacle_grid[i][j].first][obstacle_grid[i][j].second].grid_points[k][0];
							point_view.y = grid[obstacle_grid[i][j].first][obstacle_grid[i][j].second].grid_points[k][1];
							point_view.z = grid[obstacle_grid[i][j].first][obstacle_grid[i][j].second].grid_points[k][2];
							point_view.r = r;
							point_view.g = g;
							point_view.b = b;
						//	scene->push_back(point_view);
						}
					}
				}

				for (int i = 0; i < grids_obstacle_datas.size(); ++i) {

					printf("no.%d's horizontal angle is %f, vertical angle is %f, distance is %f.\n", i + 1, grids_obstacle_datas[i][1], grids_obstacle_datas[i][2], grids_obstacle_datas[i][0]);
				}
				printf("\n\n");

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

	ros::init(argc, argv, "vlp16_lidar");
	ros::NodeHandle n;
	ros::Publisher lidar_pub = n.advertise<amazing_car::my_lidar_distance>("vlp16_lidar", 1000);	
	ros::Rate rate(120);
 	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
 	pcap_t * device = get_pcap_dev_handle(6);
	while (ros::ok())
	{
		std::vector<std::vector<float>> datas=get_current_frame_vld16(device, cloud);//return vector<vector<float>>  distance,hori_angle,vert_angle
		float distance = 100000;
		float angle = -360;
		for(int i=0;i<datas.size();++i){
		   	printf("no.%d  distance:%f, hori_angle:%f, vert_angle:%f\n",i,datas[i][0],datas[i][1],datas[i][2]);
		   	
			if(datas[i][1] <= -30 || datas[i][1]>=30){
				continue;
			}
			if(datas[i][0] < distance){
				distance = datas[i][0];
				angle = datas[i][1];
			}
		}
		amazing_car::my_lidar_distance lidar_msg;
		lidar_msg.min_distance = distance;
		lidar_msg.angle = angle;
		lidar_pub.publish(lidar_msg);
	}
	return 1;
}
