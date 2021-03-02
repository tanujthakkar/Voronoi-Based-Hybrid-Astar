#ifndef NODE4D
#define NODE4D

#include<cmath>
#include "constants.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

const int n = ceil(WHEELBASE/MOVE_STEP);

class Node4D {

public:

	Node4D(float x, float y, float yaw, float yawt) {

		this->x = x;
		this->y = y;
		this->yaw = yaw;
		this->yawt = yawt;
	}

	Node4D(std::vector<float> xlist, std::vector<float> ylist, std::vector<float> yawlist, 
		std::vector<float> yawtlist, float steer, int dir) {

		this->xlist.resize(n);
		this->ylist.resize(n);
		this->yawlist.resize(n);
		this->yawtlist.resize(n);
		this->directions.resize(n);

		this->xlist = xlist;
		this->ylist = ylist;
		this->yawlist = yawlist;
		this->yawtlist = yawtlist;
		this->steer = steer;
		this->direction = dir;
	}

	// Node4D(int xind, int yind, int yawind, bool direction,
	// 	   float steer, float cost, int pind, const Node4D* parent) {

	// 	this->xind = xind;
	// 	this->yind = yind;
	// 	this-> yawind = yawind;
	// 	this-> direction = direction;
	// 	this-> steer = steer;
	// 	this-> cost = cost;
	// 	this-> pind = pind;
	// 	this-> parent = parent;
	// }

	float x;
	float y;
	float yaw;
	float yawt;

	float get_x(int i) const { return xlist[i]; }

	float get_y(int i) const { return ylist[i]; }

	float get_yaw(int i) const { return yawlist[i]; }

	float get_yawt(int i) const { return yawtlist[i]; }

	float get_steer() const { return steer; }

	int get_size() const { return xlist.capacity(); }

private:
	int xind;
	int yind;
	int yawind;
	int direction;
	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> yawlist;
	std::vector<float> yawtlist;
	std::vector<bool> directions;
	float steer;
	float cost;
	int pind;
	const Node4D* parent;
};

#endif