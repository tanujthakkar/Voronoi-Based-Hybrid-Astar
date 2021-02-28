#ifndef NODE4D
#define NODE4D

#include<cmath>
#include "constants.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

static const int W = 1;

class Node4D {

public:

	Node4D(float xind, float yind, float yawind, bool direction,
		   std::vector<float> x, std::vector<float> y, std::vector<float> yaw,
		   std::vector<float> yawt, std::vector<bool> directions, 
		   float steer, float cost, float pind, const Node4D* parent) {

		this->xind = xind;
		this->yind = yind;
		this-> yawind = yawind;
		this-> direction = direction;
		this-> x = x;
		this-> y = y;
		this-> yaw = yaw;
		this-> yawt = yawt;
		this-> directions = directions;
		this-> steer = steer;
		this-> cost = cost;
		this-> pind = pind;
		this-> parent = parent;
	}

private:

	float xind;
	float yind;
	float yawind;
	bool direction;
	std::vector<float> x
	std::vector<float> y;
	std::vector<float> yaw;
	std::vector<float> yawt;
	std::vector<bool> directions;
	float steer;
	float cost;
	const Node4D* parent;
};

#endif