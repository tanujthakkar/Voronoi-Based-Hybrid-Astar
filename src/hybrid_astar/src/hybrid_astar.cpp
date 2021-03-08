#include "../include/hybrid_astar/hybrid_astar.h"

using namespace std;
using namespace std::chrono;


// Global Variables
ros::Publisher start_pose_pub;
ros::Publisher path_pub;
ros::Publisher visualize_nodes_pub;
ros::Publisher robot_polygon_pub;
ros::Publisher trailer_polygon_pub;
ros::Publisher robot_center_pub;
ros::Publisher trailer_center_pub;
ros::Publisher robot_collision_check_pub;
ros::Publisher trailer_collision_check_pub;

nav_msgs::Path path;
nav_msgs::OccupancyGrid::Ptr grid;
bool** bin_map;
int** acc_obs_map;
geometry_msgs::PoseStamped start_pose;
visualization_msgs::Marker nodes;
geometry_msgs::PolygonStamped robot_polygon;

Node4D* current_node;
Node4D* new_node;

float sx;
float sy;
float syaw;

std::vector<int> steer = {30, 0, -30};


/*
	Function to create a successor node

	node: input node to create successor for
	steer: steering input of the new node
	dir: direction of the new node

	Returns a pointer to the new node
*/
Node4D* create_successor(Node4D* node, float steer,int dir) {

	int nlist = ceil(PATH_LENGTH/MOVE_STEP);
	int n = node->get_size();

	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> yawlist;
	std::vector<float> yawtlist;
	std::vector<float > yawt;

	xlist.resize(nlist);
	ylist.resize(nlist);
	yawlist.resize(nlist);
	yawtlist.resize(nlist);
	yawt.resize(nlist);

	xlist[0] = node->get_x(n-1) + (dir * MOVE_STEP) * cos(node->get_yaw(n-1));
	ylist[0] = node->get_y(n-1) + (dir * MOVE_STEP) * sin(node->get_yaw(n-1));
	yawlist[0] = pi_2_pi(node->get_yaw(n-1) + (dir * MOVE_STEP / WHEELBASE) * tan(to_rad(steer)));
	yawtlist[0] = pi_2_pi(node->get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(node->get_yawt(n-1) - node->get_yaw(n-1)));
	yawt[0] = pi_2_pi(node->get_yaw_t(0));

	for(int i=1;i<nlist;i++) {
		xlist[i] = xlist[i-1] + (dir * MOVE_STEP) * cos(yawlist[i-1]);
		ylist[i] = ylist[i-1] + (dir * MOVE_STEP) * sin(yawlist[i-1]);
		yawlist[i] = pi_2_pi(yawlist[i-1] + (dir * MOVE_STEP / WHEELBASE) * tan(to_rad(steer)));
		yawtlist[i] = pi_2_pi(yawtlist[i-1] + (dir * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawtlist[i-1]));
		yawt[i] = pi_2_pi(yawt[i-1] + (dir * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawtlist[i-1]));
	}

	float cost = 0.0;

	// Calculating g(n) cost
	if (dir > 0) {
		cost = nlist * MOVE_STEP;
		// cout << "Cost : " << cost << endl;
	} else {
		cost = nlist * MOVE_STEP + BACKWARD_COST;
		// cout << "Cost : " << cost << endl;
	}

	cost = cost + abs(steer) * STEER_ANGLE_COST;
	// cout << "Cost : " << cost << endl;
	cost = cost + abs(node->get_steer() - steer) * STEER_CHANGE_COST;
	// cout << "Cost : " << cost << endl;

	cost = cost + node->get_cost();
	// cout << "Cost : " << cost << endl;

	return new Node4D(xlist, ylist, yawlist, yawtlist, yawt, dir, steer, cost, node);
}


/*
	Publishes the final path using the current_node pointer as a marker array
*/
void visualize_final_path() {

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
	// cout << " Current Node : " << current_node << endl;
	while(current_node->get_parent() != nullptr) {
		// cout << "Node : " << current_node << " Next Node : " << current_node->get_parent() << endl;
		for(int i=0;i<ceil(PATH_LENGTH/MOVE_STEP);i++) {
			p.x = current_node->get_x(i);
			p.y = current_node->get_y(i);
			p.z = 0;
			nodes.points.push_back(p);
		}
		current_node = current_node->get_parent();
	}
}


/*
	Subcribes/callback: /initial_pose
	Publishes: /start_pose

	Callback function to retrieve the initial_pose and display it as a pose in rviz
*/
void callback_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {

	start_pose.header.stamp = ros::Time::now();
	start_pose.header.frame_id = "map";
	start_pose.pose.position = pose->pose.pose.position;
	start_pose.pose.orientation = pose->pose.pose.orientation;

	float deltar = (RF - RB) / 2.0;
	sx = start_pose.pose.position.x - deltar * cos(syaw);
	sy = start_pose.pose.position.y - deltar * sin(syaw);
	syaw = tf::getYaw(start_pose.pose.orientation);

	ROS_INFO("X: %f \t Y: %f \t YAW: %f", start_pose.pose.position.x, start_pose.pose.position.y, syaw);
	if (grid->info.height >= start_pose.pose.position.y && start_pose.pose.position.y >= 0 && 
		grid->info.width >= start_pose.pose.position.x && start_pose.pose.position.x >= 0) {
		ROS_INFO("VALID START!");
    } else  {
    	ROS_INFO("INVALID START!");
    }

	Node4D start_node = Node4D(sx, sy, syaw, 0);
	current_node = &start_node;

	auto start = high_resolution_clock::now();
	new_node = create_successor(current_node, 30, 1);
	current_node = new_node;

	// current_node->check_path_collision(bin_map);
	auto stop = high_resolution_clock::now();
	current_node->check_collision(grid, bin_map, acc_obs_map);
	auto duration = duration_cast<microseconds>(stop-start);
	std::cout << "Execution Time : " << duration.count() << " microseconds" << endl;

    start_pose_pub.publish(start_pose);

	// for (int j = 0; j < 5; ++j)
	// {
	// 	for (int i = 0; i < 3; ++i)
	// 	{
	// 		cout << "Current : " << current_node << endl;
	// 		new_node = create_successor(current_node, steer[i], 1);
	// 		cout << "New : " << new_node << endl;
	// 	}
	// 	current_node = new_node;
	// }
	
	visualize_final_path();

	visualize_nodes_pub.publish(nodes);
}


/*
	Subscribes/Callback: /map
	Publishes: None

	Callback function to retrieve the occupancy grid and construct a 2D binary obstacle map 
*/
void callback_map(const nav_msgs::OccupancyGrid::Ptr map) {

	grid = map;
	ROS_INFO("Recieved the occupancy grid map");

	int height = map->info.height;
	int width = map->info.width;
	bin_map = new bool*[width];

	for (int x = 0; x < width; ++x) { bin_map[x] = new bool[height]; }

	for (int x = 0; x < width; ++x) {
		for (int y = 0; y < height; ++y) {
			bin_map[x][y] = map->data[y * width + x] ? true : false;
		}
	}

	acc_obs_map = new int* [width];

	for (int x = 0; x < width; x++) {
		acc_obs_map[x] = new int[height];
		for (int y = 0; y < height; y++) {
			acc_obs_map[x][y] = (bin_map[x][y] > 0);
		}
	}

	for (int x = 0; x < width; x++) {
		for (int y = 1; y < height; y++) {
			acc_obs_map[x][y] = acc_obs_map[x][y-1] + acc_obs_map[x][y];
		}
	}

	for (int y = 0; y < height; y++) {
		for (int x = 1; x < width; x++) {
			acc_obs_map[x][y] = acc_obs_map[x-1][y] + acc_obs_map[x][y];
		}
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
	path_pub = nh.advertise<nav_msgs::Path>("path", 10);
	visualize_nodes_pub = nh.advertise<visualization_msgs::Marker>("nodes", 10);
	robot_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_polygon", 10);
	trailer_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("trailer_polygon", 10);
	robot_center_pub = nh.advertise<geometry_msgs::PointStamped>("robot_center", 10);
	trailer_center_pub = nh.advertise<geometry_msgs::PointStamped>("trailer_center", 10);
	robot_collision_check_pub = nh.advertise<visualization_msgs::Marker>("robot_collision_check", 10);
	trailer_collision_check_pub = nh.advertise<visualization_msgs::Marker>("trailer_collision_check", 10);


	// Subscribers
	ros::Subscriber sub_start_pose = nh.subscribe("initialpose", 10, callback_start_pose);
	ros::Subscriber sub_map = nh.subscribe("map", 1, callback_map);
	
	ros::Rate loop_rate(10);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}