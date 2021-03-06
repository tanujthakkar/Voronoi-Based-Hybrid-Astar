#include "../include/hybrid_astar/node4d.h"

using namespace std;

bool Node4D::check_path_collision(bool** bin_map) {

	for (int i = 0; i < xlist.capacity(); ++i)
	{
		if(bin_map[(int)round(xlist[i]/XY_RESOLUTION)][(int)round(ylist[i]/XY_RESOLUTION)]) {
			ROS_INFO("IN COLLISION!");
			return true;
		}
	}

	ROS_INFO("NO COLLISION!");
	return false; // NO collision
}

bool Node4D::check_collision(nav_msgs::OccupancyGrid::Ptr grid,bool** bin_map, int** acc_obs_map) {

	int max_x;
	int min_x;
	int max_y;
	int min_y;

	float deltal;
	float cx;
	float cy;
	
	// Robot/Tractor Collision Check
	deltal = (RF - RB) / 2.0;
	for (int i = 0; i < xlist.capacity(); ++i) {

		// ROS_INFO("Collision check for X : %f || Y : %f ", xlist[i], ylist[i]);
		cx = xlist[i] + deltal * cos(yawlist[i]);
		cy = ylist[i] + deltal * sin(yawlist[i]);

		if(xlist[i] >= grid->info.width || xlist[i]<0 || ylist[i] >= grid->info.height || ylist[i] < 0) {
			ROS_INFO("OUT OF BOUNDS");
			return true; // OUT OF BOUNDS
		}
		
		max_x =  (cx + RL * abs(cos(yawlist[i]))/2 + RW * abs(sin(yawlist[i]))/2) + MIN_SAFE_DIST;
		min_x =  (cx - RL * abs(cos(yawlist[i]))/2 - RW * abs(sin(yawlist[i]))/2) - MIN_SAFE_DIST;

		max_y =  (cy + RL * abs(sin(yawlist[i]))/2 + RW * abs(cos(yawlist[i]))/2) + MIN_SAFE_DIST;
		min_y =  (cy - RL * abs(sin(yawlist[i]))/2 - RW * abs(cos(yawlist[i]))/2) - MIN_SAFE_DIST;
		
		if(max_x >= grid->info.width || min_x < 0 || max_y >= grid->info.height || min_y < 0) {
			ROS_INFO("OUT OF BOUNDS");
			return true; // OUT OF BOUNDS
		}

		// Converting coordinates to indices
		max_x /= XY_RESOLUTION;
		max_y /= XY_RESOLUTION;
		min_y /= XY_RESOLUTION;
		min_x /= XY_RESOLUTION;

		// if(acc_obs_map[max_x][max_y] + acc_obs_map[min_x][min_y] == acc_obs_map[max_x][min_y] + acc_obs_map[min_x][max_y])
		// 	ROS_INFO("ACC SAFE");
		// 	return false; // NO COLLISION

		// Checking the robot polygon
		int s;
		int t;

		for(float k = -RL/2 + MIN_SAFE_DIST; k <= RL/2 + MIN_SAFE_DIST; k += 0.25) {
			for(float j = -RW/2 + MIN_SAFE_DIST; j <= RW/2 + MIN_SAFE_DIST; j += 0.25) {

				s = (cx + k * cos(yawlist[i]) + j * sin(yawlist[i]))/XY_RESOLUTION + 0.001;
				t = (cy + k * sin(yawlist[i]) + j * cos(yawlist[i]))/XY_RESOLUTION + 0.001;
	     		
	     		if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
	     			ROS_INFO("POLYGON COLLISION");
					return true; // IN COLLISION
	     		}
			}
		}
	}

	ROS_INFO("NO COLLISION - SAFE");
	return false; // NO COLLISION
}