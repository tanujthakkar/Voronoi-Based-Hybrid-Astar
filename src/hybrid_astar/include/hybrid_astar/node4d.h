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
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "constants.h"
#include "helper.h"
#include "hybrid_astar.h"

using namespace std;

class Node4D {

public:

	// Default Constructor
	Node4D() {};

	// Constructor for start and goal nodes
	Node4D(float x, float y, float yaw, float yawt, float yaw_t, int ind) {

		xlist.push_back(x);
		ylist.push_back(y);
		yawlist.push_back(yaw);
		yawtlist.push_back(yawt);
		this->yawt.push_back(yaw_t);
		this->direction = 1;
		this->steer = 0.0;
		this->cost = 0.0;
		this->ind = ind;
		this->parent_ind = NULL;
	}

	// Constructor for successor nodes
	Node4D(std::vector<float> xlist, std::vector<float> ylist, std::vector<float> yawlist, 
		std::vector<float> yawtlist, std::vector<float> yawt, int dir, float steer, float cost, int ind, int parent_ind) {

		int n = xlist.size();
		this->xlist.resize(n);
		this->ylist.resize(n);
		this->yawlist.resize(n);
		this->yawtlist.resize(n);
		this->yawt.resize(n);

		this->xlist = xlist;
		this->ylist = ylist;
		this->yawlist = yawlist;
		this->yawtlist = yawtlist;
		this->yawt = yawt;
		this->direction = dir;
		this->steer = steer;
		this->cost = cost;
		this->ind = ind;
		this->parent_ind = parent_ind;
	}
	

	// Get functions to retrieve class data
	float get_x(int i) const { return xlist[i]; }

	float get_y(int i) const { return ylist[i]; }

	float get_yaw(int i) const { return yawlist[i]; }

	float get_yawt(int i) const { return yawtlist[i]; }

	float get_yaw_t(int i) const { return yawt[i]; }

	int get_dir() const { return direction; }

	float get_steer() const { return steer; }

	float get_cost() const { return cost; }

	Node4D* get_parent() { return parent; }

	Node4D* get_child() { return child; }

	int get_ind() { return ind; }

	int get_parent_ind() { return parent_ind; }

	int get_child_ind() { return child_ind; }

	int get_size() const { return xlist.size(); }

	// Set functions to set class data
	void set_cost(float cost) { this->cost + cost; }

	void set_child(Node4D* next) { child = next; } 

	void set_ind(int ind) { this->ind = ind; }

	void set_child_ind(int child_ind) { this->child_ind = child_ind; }

	// Fucntion to check path collision
	bool check_path_collision(bool** bin_map);

	// Function to check tractor-trailer collision
	bool check_collision(nav_msgs::OccupancyGrid::Ptr grid, bool** bin_map,  int** acc_obs_map);

private:
	// Private node variables
	std::vector<float> xlist; // x coords of path points
	std::vector<float> ylist; // y coords of path points
	std::vector<float> yawlist; // yaw of path points
	std::vector<float> yawtlist; // hitch-angle of path points
	std::vector<float> yawt; // Trailer yaw for collision check
	int direction; // Direction of motion
	float steer; // Node steering angle
	float cost; // Node cost
	Node4D* parent; // Pointer to the parent node
	Node4D* child; // Pointer to the next/child node in the final path
	int ind; // Node index
	int parent_ind; // Parent node index
	int child_ind; // Child node index
};

// Function to create a polygon for the robot and the trailer
geometry_msgs::PolygonStamped create_polygon(float l, float w, float cx, float cy, float yaw);

#endif