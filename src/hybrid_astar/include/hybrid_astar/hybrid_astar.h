#ifndef HYBRID_ASTAR
#define HYBRID_ASTAR

#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <queue>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include "constants.h"
#include "helper.h"
#include "node4d.h"
#include "dubins.h"
#include "reeds_shepp.h"
#include "pure_pursuit.h"

#include "hybrid_astar/GlobalPath.h"
#include "hybrid_astar/MonteCarloSim.h"

using namespace std;

extern ros::Publisher start_pose_pub;
extern ros::Publisher goal_pose_pub;
extern ros::Publisher hybrid_path_pub;
extern ros::Publisher global_path_pub;
extern ros::Publisher dubins_path_pub;
extern ros::Publisher reeds_shepp_path_pub;
extern ros::Publisher visualize_nodes_pub;
extern ros::Publisher robot_polygon_pub;
extern ros::Publisher trailer_polygon_pub;
extern ros::Publisher robot_center_pub;
extern ros::Publisher trailer_center_pub;
extern ros::Publisher robot_collision_check_pub;
extern ros::Publisher trailer_collision_check_pub;
extern geometry_msgs::PoseStamped start_pose;
extern nav_msgs::Path path;
extern bool visualization;

#endif