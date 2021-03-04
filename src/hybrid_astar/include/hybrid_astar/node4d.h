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

	// Constructor for start and goal nodes
	Node4D(float x, float y, float yaw, float yawt) {

		this->x = x;
		this->y = y;
		this->yaw = yaw;
		this->yawt = yawt;
		xlist.push_back(x);
		ylist.push_back(y);
		yawlist.push_back(yaw);
		yawtlist.push_back(yawt);
		this->direction = 1;
		this->steer = 0.0;
		this->cost = 0.0;
		this->parent = nullptr;
	}

	// Constructor for successor nodes
	Node4D(std::vector<float> xlist, std::vector<float> ylist, std::vector<float> yawlist, 
		std::vector<float> yawtlist, int dir, float steer, float cost, Node4D* parent) {

		this->xlist.resize(n);
		this->ylist.resize(n);
		this->yawlist.resize(n);
		this->yawtlist.resize(n);
		this->directions.resize(n);

		this->xlist = xlist;
		this->ylist = ylist;
		this->yawlist = yawlist;
		this->yawtlist = yawtlist;
		this->direction = dir;
		this->steer = steer;
		this->cost = cost;
		this->parent = parent;
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

	int get_dir() const { return direction; }

	float get_steer() const { return steer; }

	float get_cost() const { return cost; }

	Node4D* get_parent() { return parent; }

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
	Node4D* parent;
};

#endif