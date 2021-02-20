#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>

using namespace std;

// Global Publisher Variables
ros::Publisher pub;
ros::Publisher path_pub;


nav_msgs::Path path;
geometry_msgs::PoseStamped start_pose;

float x;
float y;
float yaw;

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
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1000);
	path_pub = nh.advertise<nav_msgs::Path>("path", 1000);
	ros::Subscriber sub_start_pose = nh.subscribe("initialpose", 1000, callback_start_pose);

	ros::Rate loop_rate(10);

	while(ros::ok()) {

		path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        path.poses.clear();

		for(int i=0;i<6;i++) {
			geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = start_pose.pose.position.x + i;
            pose_stamped.pose.position.y = start_pose.pose.position.y;
            pose_stamped.pose.position.z = 0;
            pose_stamped.pose.orientation.w = yaw;
            path.poses.push_back(pose_stamped);
		}
		path_pub.publish(path);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}