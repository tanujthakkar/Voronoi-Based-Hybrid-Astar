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