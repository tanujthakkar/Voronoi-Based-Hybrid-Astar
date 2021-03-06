#ifndef NODE4D
#define NODE4D

#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "constants.h"
#include "helper.h"

using namespace std;

const int n = ceil(PATH_LENGTH/MOVE_STEP);

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

	float x;
	float y;
	float yaw;
	float yawt;

	// Get functions to retrieve class data
	float get_x(int i) const { return xlist[i]; }

	float get_y(int i) const { return ylist[i]; }

	float get_yaw(int i) const { return yawlist[i]; }

	float get_yawt(int i) const { return yawtlist[i]; }

	int get_dir() const { return direction; }

	float get_steer() const { return steer; }

	float get_cost() const { return cost; }

	Node4D* get_parent() { return parent; }

	int get_size() const { return xlist.capacity(); }


	// Fucntion to check path collision
	bool check_path_collision(bool** bin_map);

	// Function to check tractor-trailer collision
	bool check_collision(bool** bin_map);

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