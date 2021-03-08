#ifndef HYBRID_ASTAR
#define HYBRID_ASTAR

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
#include "node4d.h"

using namespace std;

extern ros::Publisher start_pose_pub;
extern ros::Publisher path_pub;
extern ros::Publisher visualize_nodes_pub;
extern ros::Publisher robot_polygon_pub;
extern ros::Publisher trailer_polygon_pub;
extern ros::Publisher robot_center_pub;
extern ros::Publisher trailer_center_pub;
extern ros::Publisher robot_collision_check_pub;
extern ros::Publisher trailer_collision_check_pub;

#endif