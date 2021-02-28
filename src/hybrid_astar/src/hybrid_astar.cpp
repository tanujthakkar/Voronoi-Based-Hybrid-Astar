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

#include "../include/hybrid_astar/constants.h"
#include "../include/hybrid_astar/helper.h"
#include "../include/hybrid_astar/node4d.h"

using namespace std;
using namespace std::chrono;

// Constants

// const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
// const float YAW_RESOLUTION = (15 * (M_PI / 180)); // [rad]
// const float MOVE_STEP = 0.1; // [m] Path interpolate resolution

// // Vehicle Configuration
// const float WHEELBASE = 0.8; // [m] Wheelbase of the tractor, i.e., distance from front axle to rear axle
// const float RTR = 0.8; // [m] Distance from the rear axle (hitch position) of the tractor to rear axle of the trailer


// Global Publisher Variables
ros::Publisher pub;
ros::Publisher path_pub;
ros::Publisher visualize_nodes_pub;


nav_msgs::Path path;
geometry_msgs::PoseStamped start_pose;
visualization_msgs::Marker nodes;

float x;
float y;
float yaw;

void generate_path(float x, float y, float yaw, float yawt, float nyaw) {

	int d = 1; // direction of motion
	int nlist = ceil(WHEELBASE / MOVE_STEP);

	float xlist[nlist];
	float ylist[nlist];
	float yawlist[nlist];
	float yawtlist[nlist];

	xlist[0] = x + (d * MOVE_STEP) * cos(yaw);
	ylist[0] = y + (d * MOVE_STEP) * sin(yaw);
	yawlist[0] = pi_2_pi(yaw + (d * MOVE_STEP / WHEELBASE) * tan(to_rad(nyaw)));
	yawtlist[0] = pi_2_pi(yawt + (d * MOVE_STEP / RTR) * sin(yawt - yaw));
	// ROS_INFO("x[%d] : %f || y[%d] : %f || yaw[%d] : %f || yawt[%d] : %f", 0, xlist[0], 0, ylist[0], 0, yawlist[0], 0, yawtlist[0]);

	for(int i=1;i<nlist;i++) {
		xlist[i] = xlist[i-1] + (d * WHEELBASE * MOVE_STEP) * cos(yawlist[i-1]);
		ylist[i] = ylist[i-1] + (d * WHEELBASE * MOVE_STEP) * sin(yawlist[i-1]);
		yawlist[i] = pi_2_pi(yawlist[i-1] + (d * MOVE_STEP / WHEELBASE) * tan(to_rad(nyaw)));
		yawtlist[i] = pi_2_pi(yawtlist[i-1] + (d * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawtlist[i-1]));
		// ROS_INFO("x[%d] : %f || y[%d] : %f || yaw[%d] : %f || yawt[%d] : %f", i, xlist[i], i, ylist[i], i, yawlist[i], i, yawtlist[i]);
	}

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
	path.poses.clear();

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.header.frame_id = "map";
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;
	pose_stamped.pose.position.z = 0;
	pose_stamped.pose.orientation.w = yaw;
	path.poses.push_back(pose_stamped);

	for(int i=0;i<nlist;i++) {
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.header.frame_id = "map";
		pose_stamped.pose.position.x = xlist[i];
		pose_stamped.pose.position.y = ylist[i];
		pose_stamped.pose.position.z = 0;
		pose_stamped.pose.orientation.w = yawlist[i];
		path.poses.push_back(pose_stamped);
	}

	path_pub.publish(path);
}

void generate_path_vector(float x, float y, float yaw, float yawt, float nyaw) {

	int d = 1; // direction of motion
	int nlist = ceil(WHEELBASE / MOVE_STEP);

	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> yawlist;
	std::vector<float> yawtlist;

	xlist.reserve(nlist);
	ylist.reserve(nlist);
	yawlist.reserve(nlist);
	yawtlist.reserve(nlist);

	xlist[0] = x + (d * MOVE_STEP) * cos(yaw);
	ylist[0] = y + (d * MOVE_STEP) * sin(yaw);
	yawlist[0] = pi_2_pi(yaw + (d * MOVE_STEP / WHEELBASE) * tan(to_rad(nyaw)));
	yawtlist[0] = pi_2_pi(yawt + (d * MOVE_STEP / RTR) * sin(yawt - yaw));

	for(int i=1;i<nlist;i++) {
		xlist[i] = xlist[i-1] + (d * WHEELBASE * MOVE_STEP) * cos(yawlist[i-1]);
		ylist[i] = ylist[i-1] + (d * WHEELBASE * MOVE_STEP) * sin(yawlist[i-1]);
		yawlist[i] = pi_2_pi(yawlist[i-1] + (d * MOVE_STEP / WHEELBASE) * tan(to_rad(nyaw)));
		yawtlist[i] = pi_2_pi(yawtlist[i-1] + (d * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawtlist[i-1]));
		// ROS_INFO("x[%d] : %f || y[%d] : %f || yaw[%d] : %f || yawt[%d] : %f", i, xlist[i], i, ylist[i], i, yawlist[i], i, yawtlist[i]);
	}

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
	path.poses.clear();

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.header.frame_id = "map";
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;
	pose_stamped.pose.position.z = 0;
	pose_stamped.pose.orientation.w = yaw;
	path.poses.push_back(pose_stamped);

	for(int i=0;i<nlist;i++) {
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.header.frame_id = "map";
		pose_stamped.pose.position.x = xlist[i];
		pose_stamped.pose.position.y = ylist[i];
		pose_stamped.pose.position.z = 0;
		pose_stamped.pose.orientation.w = yawlist[i];
		path.poses.push_back(pose_stamped);
	}

	path_pub.publish(path);
}

void visualize_nodes() {
	
	nodes.header.stamp = ros::Time::now();
	nodes.header.frame_id = "/map";
	nodes.ns = "v_nodes";
	nodes.action = visualization_msgs::Marker::ADD;
	nodes.id = 0;
	nodes.type = visualization_msgs::Marker::LINE_LIST;
	nodes.scale.x = 0.02;
	nodes.color.r = 1.0;
	nodes.color.a = 1.0;

	geometry_msgs::Point p;
	for(int i=0;i<8;i++) {
		p.x = path.poses[i].pose.position.x;
		p.y = path.poses[i].pose.position.y;
		p.z = 0;
		nodes.points.push_back(p);
	}
	// for(int i=0;i<10;i++) {
	// 	p.x = i;
	// 	p.y = i;
	// 	p.z = 0;
	// 	nodes.points.push_back(p);
	// }
}

void callback_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p) {

	start_pose.header.stamp = ros::Time::now();
	start_pose.header.frame_id = "map";
	start_pose.pose.position = p->pose.pose.position;
	start_pose.pose.orientation = p->pose.pose.orientation;
	// geometry_msgs::Quaternion quat;
	// quat.x = p->pose.pose.orientation.x;
	// quat.y = p->pose.pose.orientation.y;
	// quat.z = p->pose.pose.orientation.z;
	// quat.w = p->pose.pose.orientation.w;
	yaw = tf::getYaw(start_pose.pose.orientation);
	// start_pose->pose.orientation = p->pose.pose.orientation;

	ROS_INFO("X: %f \t Y: %f \t YAW: %f", start_pose.pose.position.x, start_pose.pose.position.y, yaw);
	pub.publish(start_pose);
	// generate_path(start_pose.pose.position.x, start_pose.pose.position.y, yaw, 0, 30.0);
	// generate_path(start_pose.pose.position.x, start_pose.pose.position.y, yaw, 0, 0.0);
	
	std::vector<float> t;
	auto start = high_resolution_clock::now();
	generate_path_vector(start_pose.pose.position.x, start_pose.pose.position.y, yaw, 0, -30.0);
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop-start);
	std::cout << "Execution Time : " << duration.count() << endl;
	// visualize_nodes();
	// visualize_nodes_pub.publish(nodes);
}

// void callback_map(const nav_msgs::OccupancyGrid::Ptr map) {

// 	grid = map;
// }

int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	// Publishers
	pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
	path_pub = nh.advertise<nav_msgs::Path>("path", 10);
	visualize_nodes_pub = nh.advertise<visualization_msgs::Marker>("nodes", 10);

	// Subscribers
	ros::Subscriber sub_start_pose = nh.subscribe("initialpose", 10, callback_start_pose);
	// ros::Subscriber sub_map = nh.subscribe('map', 1, callback_map);
	
	ros::Rate loop_rate(10);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}