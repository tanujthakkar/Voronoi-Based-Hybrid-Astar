#include "../include/hybrid_astar/hybrid_astar.h"

using namespace std;
using namespace std::chrono;


// Global Variables

// Publishers
ros::Publisher start_pose_pub;
ros::Publisher goal_pose_pub;
ros::Publisher hybrid_path_pub;
ros::Publisher hybrid_tractor_path_pub;
ros::Publisher hybrid_trailer_path_pub;
ros::Publisher dubins_path_pub;
ros::Publisher reeds_shepp_path_pub;
ros::Publisher visualize_nodes_pub;
ros::Publisher robot_polygon_pub;
ros::Publisher robot_polyogn_array_pub;
ros::Publisher trailer_polygon_pub;
ros::Publisher trailer_polyogn_array_pub;
ros::Publisher robot_center_pub;
ros::Publisher trailer_center_pub;
ros::Publisher robot_collision_check_pub;
ros::Publisher trailer_collision_check_pub;
ros::Publisher voronoi_path_pub;
ros::Publisher voronoi_sub_goals_pub;
ros::Publisher astar_path_pub;
ros::Publisher cmd_pub;
ros::Publisher target_point_pub;

// Subscribers
ros::Subscriber voronoi_graph_sub;

bool visualization = false; // Visualization toggle
bool visualization_final_node = true; // Final path visualization toggle

geometry_msgs::PoseStamped start_pose; // Start pose msg
bool valid_start; // Start pose validity check
geometry_msgs::PoseStamped goal_pose; // Goal pose msg
bool valid_goal; // Goal pose validity check

nav_msgs::OccupancyGrid::Ptr grid; // Pointer to the occupancy grid msg
int grid_height;
int grid_width;
bool** bin_map; // 2D Binary map of the grid 
int** acc_obs_map;

bool voronoi_graph_received = false; // Voronoi graph availability flag
tuw_multi_robot_msgs::Graph voronoi_graph; // Voronoi graph representing the voronoi nodes

typedef pair<float, int> pi; // Pair definition for maps

nav_msgs::Path path; // Hybrid A* path
nav_msgs::Path tractor_path; // Hybrid A* path for tractor
nav_msgs::Path trailer_path; // Hybrid A* path for trailer
std::vector<int> dirs; // Vector of directions for global path
bool path_found; // Hybrid A* availability flag
nav_msgs::Path dubins_path; // Dubins Path
visualization_msgs::Marker nodes; // Visualize all nodes

jsk_recognition_msgs::PolygonArray robot_polygon_array; // Tractor polygon visualization
jsk_recognition_msgs::PolygonArray trailer_polygon_array; // Trailer polygon visualization

float sx; // x coordinate of current pose of the rear axle/hitch point of the tractor
float sy; // y coordinate of current pose of the rear axle/hitch point of the tractor
float syaw; // yaw coordinate of current pose of the rear axle/hitch point of the tractor
float syaw_t; // yaw coordinate of current pose of the trailer
int s_ind; // start node index

float gx; // x coordinate of goal pose of the rear axle/hitch point of the trailer
float gy; // y coordinate of goal pose of the rear axle/hitch point of the trailer
float gyaw; // yaw coordinate of goal pose of the rear axle/hitch point of the trailer
float gyaw_t; // yaw coordinate of goal pose of the trailer
int g_ind; // goal node index

Node4D* start_node_ptr; // Pointer to the start node
Node4D* goal_node_ptr; // Pointer to the goal node

unsigned int iterations; // total iterations of hybrid a* planning
bool iteration_limit_flag; // iteration limit flag
unsigned int total_nodes; // total nodes of hybrid a* planning
unsigned int execution_time; // execution time of hybrid a* planning

std::vector<int> steer; // Steering inputs for which the nodes have to be created
std::vector<int> direction = {-1, 1}; // Direction inputs for which the nodes have to be created


/*
	Function to create a successor node

	node: input node to create successor for
	steer: steering input of the new node
	dir: direction of the new node

	Returns: new node object
*/
Node4D create_successor(Node4D &node, float steer,int dir) {

	int nlist = ceil(PATH_LENGTH/MOVE_STEP);
	int n = node.get_size();
	int jacknife_sum = 0;

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

	xlist[0] = node.get_x(n-1) + (dir * MOVE_STEP) * cos(node.get_yaw(n-1));
	ylist[0] = node.get_y(n-1) + (dir * MOVE_STEP) * sin(node.get_yaw(n-1));
	yawlist[0] = pi_2_pi(node.get_yaw(n-1) + (dir * MOVE_STEP / WHEELBASE) * tan(to_rad(steer)));
	yawtlist[0] = pi_2_pi(node.get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(node.get_yaw(n-1) - node.get_yawt(n-1)));
	yawt[0] = pi_2_pi(node.get_yaw_t(n-1) + (dir * MOVE_STEP / RTR) * sin(node.get_yaw(0) - node.get_yaw_t(0)));
	jacknife_sum = jacknife_sum + abs(pi_2_pi(yawlist[0] - yawtlist[0]));

	for(int i=1;i<nlist;i++) {
		xlist[i] = xlist[i-1] + (dir * MOVE_STEP) * cos(yawlist[i-1]);
		ylist[i] = ylist[i-1] + (dir * MOVE_STEP) * sin(yawlist[i-1]);
		yawlist[i] = pi_2_pi(yawlist[i-1] + (dir * MOVE_STEP / WHEELBASE) * tan(to_rad(steer)));
		yawtlist[i] = pi_2_pi(yawtlist[i-1] + (dir * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawtlist[i-1]));
		yawt[i] = pi_2_pi(yawt[i-1] + (dir * MOVE_STEP / RTR) * sin(yawlist[i-1] - yawt[i-1]));
		jacknife_sum = jacknife_sum + abs(pi_2_pi(yawlist[i] - yawtlist[i]));
	}

	float cost = 0.0;

	// Calculating g(n) cost
	if (dir > 0) {
		cost = nlist * MOVE_STEP;
	} else {
		cost = nlist * MOVE_STEP + BACKWARD_COST; // Penalizing backward motion
	}

	if(dir != node.get_dir()) {
		cost = cost + DIRECTION_CHANGE_COST; // Penalizing change of direction
	}

	cost = cost + abs(steer) * STEER_ANGLE_COST; // Penalizing steering angle
	cost = cost + abs(node.get_steer() - steer) * STEER_CHANGE_COST; // Penalizing change in steering angle
	cost = cost + jacknife_sum * JACKNIFE_COST; // Penalizing jacknifing

	cost = cost + node.get_cost();

	int ind = (int)(round(yawlist[nlist-1]/YAW_RESOLUTION) * grid_width * grid_height + round(ylist[nlist-1]/XY_RESOLUTION) * grid_width + round(xlist[nlist-1]/XY_RESOLUTION)); // Calculating node index

	return Node4D(xlist, ylist, yawlist, yawtlist, yawt, dir, steer, cost, ind, node.get_ind());
}


/*
	Function to create a dubins node

	start: start node to start planning from
	goal: goal node

	Returns: dubins node object
*/
Node4D create_dubins_node(Node4D &start, Node4D &goal) {

	int n = start.get_size();
	int dir = 1;

	double q0[] = { start.get_x(n-1), start.get_y(n-1), start.get_yaw(n-1) }; // start
	double q1[] = { goal.get_x(0), goal.get_y(0), goal.get_yaw(0) }; // goal

	DubinsPath path; // initialize the path
	dubins_init(q0, q1, 1, &path); // calculate the path

	int i = 1;
	float x = 0.f;
	float length = dubins_path_length(&path);

	std::vector<float> dubins_xlist;
	std::vector<float> dubins_ylist;
	std::vector<float> dubins_yawlist;
	std::vector<float> dubins_yawtlist;
	std::vector<float> dubins_yawt;

	// dubins_path.header.stamp = ros::Time::now();
	// dubins_path.header.frame_id = "/map";
	// dubins_path.poses.clear();
	// geometry_msgs::PoseStamped pose_stamped;

	// Sampling the dubins path with the MOVE_STEP
	double q[3];
	dubins_yawtlist.push_back(pi_2_pi(start.get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(start.get_yaw(n-1) - start.get_yawt(n-1))));
	dubins_yawt.push_back(pi_2_pi(start.get_yaw_t(n-1) + (dir * MOVE_STEP / RTR) * sin(start.get_yaw(0) - start.get_yaw_t(0))));

	while (x <  length) {
		dubins_path_sample(&path, x, q);
		dubins_xlist.push_back(q[0]);
		dubins_ylist.push_back(q[1]);
		dubins_yawlist.push_back(pi_2_pi(q[2]));
		dubins_yawtlist.push_back(pi_2_pi(dubins_yawtlist[i-1] + (dir * MOVE_STEP / RTR) * sin(dubins_yawlist[i-1] - dubins_yawtlist[i-1])));
		dubins_yawt.push_back(pi_2_pi(dubins_yawt[i-1] + (dir * MOVE_STEP / RTR) * sin(dubins_yawlist[i-1] - dubins_yawt[i-1])));

		// pose_stamped.header.stamp = ros::Time::now();
		// pose_stamped.header.frame_id = "map";
		// pose_stamped.pose.position.x = q[0];
		// pose_stamped.pose.position.y = q[1];
		// pose_stamped.pose.position.z = 0;
		// pose_stamped.pose.orientation.w = pi_2_pi(q[2]);
		// dubins_path.poses.push_back(pose_stamped);

		x += MOVE_STEP;
		i++;
	}

	dubins_yawtlist.erase(dubins_yawtlist.begin());
	dubins_yawt.erase(dubins_yawt.begin());

	int nlist = dubins_xlist.size();
	int ind = (int)(round(dubins_yawlist[nlist-1]/YAW_RESOLUTION) * grid_width * grid_height + round(dubins_ylist[nlist-1]/XY_RESOLUTION) * grid_width + round(dubins_xlist[nlist-1]/XY_RESOLUTION));
	// dubins_path_pub.publish(dubins_path);
	// std::cout << "Dubins Node created" << "\n";

	// robot_polygon_array.header.stamp = ros::Time::now();
	// robot_polygon_array.header.frame_id = "map";
	// robot_polygon_array.polygons.clear();
	// robot_polygon_array.polygons.push_back(create_polygon(RL, RW, dubins_xlist[0], dubins_ylist[0], dubins_yawlist[0]));
	// robot_polyogn_array_pub.publish(robot_polygon_array);

	// robot_polygon_array.header.stamp = ros::Time::now();
	// robot_polygon_array.header.frame_id = "map";
	// robot_polygon_array.polygons.push_back(create_polygon(RL, RW, dubins_xlist[nlist-1], dubins_ylist[nlist-1], dubins_yawlist[nlist-1]));
	// robot_polyogn_array_pub.publish(robot_polygon_array);

	// float deltar = (RF - RB) / 2.0;
	// float deltat = (RTF - RTB) / 2.0;
	// float ctx;
	// float cty;

	// ctx = (dubins_xlist[0] - deltar * cos(dubins_yawlist[0])) + deltat * cos(dubins_yawt[0]);
	// cty = (dubins_ylist[0] - deltar * sin(dubins_yawlist[0])) + deltat * sin(dubins_yawt[0]);

	// trailer_polygon_array.header.stamp = ros::Time::now();
	// trailer_polygon_array.header.frame_id = "map";
	// trailer_polygon_array.polygons.clear();
	// trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, dubins_yawt[0]));
	// trailer_polyogn_array_pub.publish(trailer_polygon_array);

	// ctx = (dubins_xlist[nlist-1] - deltar * cos(dubins_yawlist[nlist-1])) + deltat * cos(dubins_yawt[nlist-1]);
	// cty = (dubins_ylist[nlist-1] - deltar * sin(dubins_yawlist[nlist-1])) + deltat * sin(dubins_yawt[nlist-1]);

	// trailer_polygon_array.header.stamp = ros::Time::now();
	// trailer_polygon_array.header.frame_id = "map";
	// trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, dubins_yawt[nlist-1]));
	// trailer_polyogn_array_pub.publish(trailer_polygon_array);

	return Node4D(dubins_xlist, dubins_ylist, dubins_yawlist, dubins_yawtlist, dubins_yawt, 1, -1, 0.0, ind, start.get_ind());
}


/*
	Function to create a list of steering inputs

	max_steer: maximum steering input in either direction

	Updates the global steer vector with steering inputs
*/
void create_steer_inputs(float max_steer) {

	steer.resize(STEER_STEP);
	for (int i = 0; i < STEER_STEP + 1; ++i)
	{
		steer[i] = max_steer - i * (2 * max_steer / STEER_STEP);
	}
}


/*
	Function to calculate the heuristic cost of the node

	n: pointer to the node

	Returns the total cost (g(n) + h(n))
*/
float calc_heuristic_cost(Node4D n) {
	int size = n.get_size();
	return n.get_cost() + (sqrt(pow((n.get_x(size-1) - gx),2) + pow((n.get_y(size-1) - gy), 2) + pow((n.get_yaw(size-1) - gyaw), 2)) * H_COST);
}


/*
	Function to display contents of a map

	m: map to be displayed
*/
void display_map(map<int, Node4D> m) {

	map<int, Node4D>::iterator itr;
	if(m.empty()) {
		cout << "List EMPTY" << endl;
	}
	for (itr = m.begin(); itr != m.end(); ++itr) {
		cout << "\tIndex: " << itr->first << endl;
	}
	cout << endl;
}


/*
	Function to display contents of the priority queue

	gq: priority queue to be displayed
*/
void display_pq(priority_queue<pi, vector<pi>, greater<pi>> gq)
{ 
	priority_queue<pi, vector<pi>, greater<pi>> g = gq;

	cout << "Priority Queue - " << endl;
	if(g.empty()) {
		cout << "Priority Queue EMPTY" << endl;
	}
	while (!g.empty()) {
		cout << "\tCost: " << g.top().first << " Index: " << g.top().second << endl;
		g.pop();
	}
	cout << endl;
}


/*
	Publishes the hybrid a* path

	current_node: address of current node of the Hybrid A* search
	closed_list: address of closed list of the Hybrid A* search
*/
void visualize_final_path(Node4D &current_node, std::map<int, Node4D> &closed_list) {

	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> yawlist;
	std::vector<float > yawt;

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "/map";
	path.poses.clear();

	nav_msgs::Path sub_path;
	sub_path.header.stamp = ros::Time::now();
	sub_path.header.frame_id = "/map";
	sub_path.poses.clear();

	tractor_path.header.stamp = ros::Time::now();
	tractor_path.header.frame_id = "/map";
	tractor_path.poses.clear();

	trailer_path.header.stamp = ros::Time::now();
	trailer_path.header.frame_id = "/map";
	trailer_path.poses.clear();

	int n;
	geometry_msgs::PoseStamped pose_stamped;

	while(current_node.get_parent_ind() != NULL) {
		n = current_node.get_size();
		for (int i = 0; i < n; i++) {
			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.header.frame_id = "map";
			pose_stamped.pose.position.x = current_node.get_x(n-i-1);
			pose_stamped.pose.position.y = current_node.get_y(n-i-1);
			pose_stamped.pose.position.z = 0;
			tf::Quaternion quat = tf::createQuaternionFromYaw(current_node.get_yaw(n-i-1));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();
			sub_path.poses.push_back(pose_stamped);

			if(visualization_final_node) {
				xlist.push_back(pose_stamped.pose.position.x);
				ylist.push_back(pose_stamped.pose.position.y);
				yawlist.push_back(current_node.get_yaw(n-i-1));
				yawt.push_back(current_node.get_yaw_t(n-i-1));
			}

			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.header.frame_id = "map";
			pose_stamped.pose.position.x = current_node.get_x(n-i-1) + DELTAR * cos(current_node.get_yaw(n-i-1));
			pose_stamped.pose.position.y = current_node.get_y(n-i-1) + DELTAR * sin(current_node.get_yaw(n-i-1));
			pose_stamped.pose.position.z = 0;
			quat = tf::createQuaternionFromYaw(current_node.get_yaw(n-i-1));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();
			tractor_path.poses.push_back(pose_stamped);

			pose_stamped.pose.position.x = current_node.get_x(n-i-1) - ((0.4) - DELTAT) * cos(current_node.get_yaw_t(n-i-1));
			pose_stamped.pose.position.y = current_node.get_y(n-i-1)  - ((0.4) - DELTAT) * sin(current_node.get_yaw_t(n-i-1));
			pose_stamped.pose.position.z = 0;
			quat = tf::createQuaternionFromYaw(current_node.get_yaw_t(n-i-1));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();
			trailer_path.poses.push_back(pose_stamped);

			dirs.push_back(current_node.get_dir());
		}
		current_node = closed_list[current_node.get_parent_ind()];
	}

	// Reversing all vectors for ascending order (start to goal)
	reverse(sub_path.poses.begin(), sub_path.poses.end());
	reverse(dirs.begin(), dirs.end());
	reverse(tractor_path.poses.begin(), tractor_path.poses.end());
	reverse(trailer_path.poses.begin(), trailer_path.poses.end());

	reverse(xlist.begin(), xlist.end());
	reverse(ylist.begin(), ylist.end());
	reverse(yawlist.begin(), yawlist.end());
	reverse(yawt.begin(), yawt.end());

	for (int i = 0; i < sub_path.poses.size(); ++i) {
		path.poses.push_back(sub_path.poses[i]);
	}

	hybrid_path_pub.publish(path);
	hybrid_tractor_path_pub.publish(tractor_path);
	hybrid_trailer_path_pub.publish(trailer_path);

	int nlist = xlist.size();
	float cx;
	float cy;
	float ctx;
	float cty;

	cx = (xlist[0] + DELTAR * cos(yawlist[0]));
	cy = (ylist[0] + DELTAR * sin(yawlist[0]));

	robot_polygon_array.header.stamp = ros::Time::now();
	robot_polygon_array.header.frame_id = "map";
	robot_polygon_array.polygons.clear();
	robot_polygon_array.polygons.push_back(create_polygon(RL, RW, cx, cy, yawlist[0]));
	robot_polyogn_array_pub.publish(robot_polygon_array);

	cx = (xlist[nlist-1] + DELTAR * cos(yawlist[nlist-1]));
	cy = (ylist[nlist-1] + DELTAR * sin(yawlist[nlist-1]));

	robot_polygon_array.header.stamp = ros::Time::now();
	robot_polygon_array.header.frame_id = "map";
	robot_polygon_array.polygons.push_back(create_polygon(RL, RW, cx, cy, yawlist[nlist-1]));
	robot_polyogn_array_pub.publish(robot_polygon_array);

	ctx = (xlist[0] + DELTAT * cos(yawt[0]));
	cty = (ylist[0] + DELTAT * sin(yawt[0]));

	trailer_polygon_array.header.stamp = ros::Time::now();
	trailer_polygon_array.header.frame_id = "map";
	trailer_polygon_array.polygons.clear();
	trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, yawt[0]));
	trailer_polyogn_array_pub.publish(trailer_polygon_array);

	ctx = (xlist[nlist-1] + DELTAT * cos(yawt[nlist-1]));
	cty = (ylist[nlist-1] + DELTAT * sin(yawt[nlist-1]));

	trailer_polygon_array.header.stamp = ros::Time::now();
	trailer_polygon_array.header.frame_id = "map";
	trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, yawt[nlist-1]));
	trailer_polyogn_array_pub.publish(trailer_polygon_array);

	if(visualization_final_node) {
		
		float cx;
		float cy;
		float ctx;
		float cty;

		unsigned int sleep = 125000;

		for (unsigned int i = 0; i < xlist.size(); ++i) {	
			usleep(sleep);

			cx = (xlist[i] + DELTAR * cos(yawlist[i]));
			cy = (ylist[i] + DELTAR * sin(yawlist[i]));

			robot_polygon_array.header.stamp = ros::Time::now();
			robot_polygon_array.header.frame_id = "map";
			robot_polygon_array.polygons.clear();
			robot_polygon_array.polygons.push_back(create_polygon(RL, RW, cx, cy, yawlist[i]));
			robot_polyogn_array_pub.publish(robot_polygon_array);

			ctx = (xlist[i] + DELTAT * cos(yawt[i]));
			cty = (ylist[i] + DELTAT * sin(yawt[i]));

			trailer_polygon_array.header.stamp = ros::Time::now();
			trailer_polygon_array.header.frame_id = "map";
			trailer_polygon_array.polygons.clear();
			trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, yawt[i]));
			trailer_polyogn_array_pub.publish(trailer_polygon_array);
		}
	}
}


/*
	Computes the Hybrid A* path
*/
bool hybrid_astar_plan() {

	nodes.header.stamp = ros::Time::now();
	nodes.header.frame_id = "/map";
	nodes.ns = "v_nodes";
	nodes.action = visualization_msgs::Marker::ADD;
	nodes.id = 0;
	nodes.type = visualization_msgs::Marker::LINE_LIST;
	nodes.scale.x = 0.02;
	nodes.color.r = 1.0;
	nodes.color.g = 0.26;
	nodes.color.b = 0.20;
	nodes.color.a = 1.0;
	nodes.points.clear();

	geometry_msgs::Point p;

	// Computing rear-axle/hitch-point pose for start node
	float cx;
	float cy;
	float ctx;
	float cty;

	syaw = tf::getYaw(start_pose.pose.orientation);
	syaw_t = tf::getYaw(start_pose.pose.orientation);
	sx = (round((start_pose.pose.position.x - DELTAR * cos(syaw)) * 20) * 0.05);
	sy = (round((start_pose.pose.position.y - DELTAR * sin(syaw)) * 20) * 0.05);

	s_ind = (int)(round(syaw/YAW_RESOLUTION) * grid_width * grid_height + round(sy/XY_RESOLUTION) * grid_width + round(sx/XY_RESOLUTION));

	start_pose.header.stamp = ros::Time::now();
	start_pose.header.frame_id = "map";
	start_pose.pose.position.x = sx;
	start_pose.pose.position.y = sy;
	tf::Quaternion quat = tf::createQuaternionFromYaw(syaw);
	start_pose.pose.orientation.x = quat.x();
	start_pose.pose.orientation.y = quat.y();
	start_pose.pose.orientation.z = quat.z();
	start_pose.pose.orientation.w = quat.w();
	start_pose_pub.publish(start_pose);

	cx = (sx + DELTAR * cos(syaw));
	cy = (sy + DELTAR * sin(syaw));

	robot_polygon_array.header.stamp = ros::Time::now();
	robot_polygon_array.header.frame_id = "map";
	robot_polygon_array.polygons.clear();
	robot_polygon_array.polygons.push_back(create_polygon(RL, RW, cx, cy, syaw));
	robot_polyogn_array_pub.publish(robot_polygon_array);

	ctx = (sx + DELTAT * cos(syaw));
	cty = (sy + DELTAT * sin(syaw));

	trailer_polygon_array.header.stamp = ros::Time::now();
	trailer_polygon_array.header.frame_id = "map";
	trailer_polygon_array.polygons.clear();
	trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, syaw_t));
	trailer_polyogn_array_pub.publish(trailer_polygon_array);

	Node4D start_node = Node4D(sx, sy, syaw, 0, syaw_t, s_ind);
	if(start_node.check_collision(grid, bin_map, acc_obs_map)) {
		// ROS_INFO("INVALID START");
		valid_start = false;
		return false;
	}
	ROS_INFO("VALID START - sx: %f sy: %f syaw: %f syaw_t: %f", sx, sy, syaw, syaw_t);
	valid_start = true;

	// Computing rear-axle/hitch-point pose for goal node
	gyaw = tf::getYaw(goal_pose.pose.orientation);
	gyaw_t = tf::getYaw(goal_pose.pose.orientation);
	gx = (round((goal_pose.pose.position.x - DELTAR * cos(gyaw)) * 20) * 0.05);
	gy = (round((goal_pose.pose.position.y - DELTAR * sin(gyaw)) * 20) * 0.05);

	g_ind = (int)(round(gyaw/YAW_RESOLUTION) * grid_width * grid_height + round(gy/XY_RESOLUTION) * grid_width + round(gx/XY_RESOLUTION));

	goal_pose.header.stamp = ros::Time::now();
	goal_pose.header.frame_id = "map";
	goal_pose.pose.position.x = gx;
	goal_pose.pose.position.y = gy;
	quat = tf::createQuaternionFromYaw(gyaw);
	goal_pose.pose.orientation.x = quat.x();
	goal_pose.pose.orientation.y = quat.y();
	goal_pose.pose.orientation.z = quat.z();
	goal_pose.pose.orientation.w = quat.w();
	goal_pose_pub.publish(goal_pose);

	cx = (gx + DELTAR * cos(gyaw));
	cy = (gy + DELTAR * sin(gyaw));

	robot_polygon_array.header.stamp = ros::Time::now();
	robot_polygon_array.header.frame_id = "map";
	robot_polygon_array.polygons.push_back(create_polygon(RL, RW, cx, cy, gyaw));
	robot_polyogn_array_pub.publish(robot_polygon_array);

	ctx = (gx + DELTAT * cos(gyaw));
	cty = (gy + DELTAT * sin(gyaw));

	trailer_polygon_array.header.stamp = ros::Time::now();
	trailer_polygon_array.header.frame_id = "map";
	trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, gyaw_t));
	trailer_polyogn_array_pub.publish(trailer_polygon_array);

	Node4D goal_node = Node4D(gx, gy, gyaw, 0, gyaw_t, g_ind);
	if(goal_node.check_collision(grid, bin_map, acc_obs_map)) {
		// ROS_INFO("INVALID GOAL");
		valid_goal = false;
		return false;
	}
	ROS_INFO("VALID GOAL - gx: %f gy: %f gyaw: %f", gx, gy, gyaw);
	valid_goal = true;

	ROS_INFO("Planning Hybrid A* path...");
	create_steer_inputs(45);

	Node4D current_node; // Node4D object to store the current node
	Node4D new_node; // Node4D object to store the next node
	Node4D sub_goal_node; // Node4D object to store sub-goals

	iterations = 0;
	total_nodes = 0;
	
	std::vector<std::vector<float>> sub_goals = voronoi_path();
	int sub_goal_counter = -1;
	bool valid_sub_goal = false;
	int sub_ind;

	std::map<int, Node4D> open_list; // Creating the open_list using a map
	std::map<int, Node4D> closed_list; // Creating the closed_list using a map

	open_list.insert(pair<int, Node4D>(start_node.get_ind(), start_node)); // Adding the start node to the open list
	priority_queue<pi, vector<pi>, greater<pi>> pq; // Creating a min priority queue to sort nodes with respect to highest priority (lowest cost)
	pq.push(make_pair(calc_heuristic_cost(start_node), start_node.get_ind())); // Adding the start node to the priority queue

	pair<float, int> ind;
	int new_ind; // Variable to store index of new nodes

	auto start = high_resolution_clock::now(); // Reading start time of planning
	execution_time = 0;

	iteration_limit_flag = false;

	while(true) {
			
		auto stop = high_resolution_clock::now(); // Reading end time of planning
		auto duration = duration_cast<milliseconds>(stop-start);
		execution_time = duration.count();

		iterations++;

		if(open_list.empty()) {
			ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
			visualize_nodes_pub.publish(nodes);
			execution_time = duration.count();
			ROS_INFO("Execution Time: %d milliseconds (%d seconds)", duration.count(), duration.count()/1000);
			return false;
		}

		ind = pq.top(); // Retrieve the pair with the highest priority (lowest cost)

		pq.pop(); // Pop the pair with highest priority

		current_node = open_list[ind.second]; // Update current node to the node with highest priority
		closed_list[ind.second] = current_node; // Added updated current node to the closed list
		open_list.erase(ind.second); // Remove updated current node from open list

		// Checking for dubins path to goal/sub-goals
		valid_sub_goal = false;
		for (int i = sub_goals.size() - 1; i > sub_goal_counter; --i) {
			sub_ind = (int)(round(sub_goals[i][2]/YAW_RESOLUTION) * grid_width * grid_height + round(sub_goals[i][1]/XY_RESOLUTION) * grid_width + round(sub_goals[i][0]/XY_RESOLUTION));
			sub_goal_node = Node4D(sub_goals[i][0], sub_goals[i][1], sub_goals[i][2], 0, sub_goals[i][2], sub_ind);
			new_node = create_dubins_node(current_node, sub_goal_node);
			if(!new_node.check_collision(grid, bin_map, acc_obs_map)) {
				pq.push(make_pair(calc_heuristic_cost(new_node), new_node.get_ind()));
				open_list[new_node.get_ind()] = new_node;
				sub_goal_counter = i;
				valid_sub_goal = true;
				total_nodes++;
				if(sub_goal_counter == sub_goals.size() - 1) {
					closed_list[new_node.get_ind()] = new_node;
					current_node = new_node;
					ROS_INFO("SOLUTION FOUND - DUBINS NODE");
					auto stop = high_resolution_clock::now(); // Reading end time of planning
					auto duration = duration_cast<milliseconds>(stop-start);
					ROS_INFO("Iterations: %d Nodes: %d", iterations, total_nodes);
					ROS_INFO("Execution Time: %d milliseconds (%d seconds) \n", duration.count(), duration.count()/1000);
					execution_time = duration.count();
					visualize_final_path(current_node, closed_list);
					visualize_nodes_pub.publish(nodes);
					return true;
				}
				break;
			}
		}

		if(valid_sub_goal) {
			continue;
		}

		// Expanding the current node
		for (unsigned int d = 0; d < direction.size(); ++d) {
			for(unsigned int i = 0; i < steer.size() + 1; ++i) {

				new_node = create_successor(current_node, steer[i], direction[d]); // Creating a successor node
				// Running collision-check for new node
				if(new_node.check_collision(grid, bin_map, acc_obs_map)) {
					continue;
				}

				// Add new node to visualization marker
				if(visualization) {
					for(int j=0;j<ceil(PATH_LENGTH/MOVE_STEP);j++) {
						p.x = new_node.get_x(j);
						p.y = new_node.get_y(j);
						p.z = 0;
						nodes.points.push_back(p);
					}
				}

				total_nodes++;

				new_ind = new_node.get_ind();

				// Checking if new node exists in closed list
				if(closed_list.count(new_ind)) {
					continue;
				}

				// Checking if new node exists in open list
				if(!open_list.count(new_ind)) {
					open_list[new_ind] = new_node;
					pq.push(make_pair(calc_heuristic_cost(new_node), new_node.get_ind()));
				} else {
					if(open_list[new_ind].get_cost() > new_node.get_cost()) {
						open_list[new_ind] = new_node; // Updating cost if new node is redundant but more optimal
					}
				}
			}
		}
	}

	iteration_limit_flag = true;
	// ROS_INFO("Iteration limit reached!\n");
	return false;
}


/*
	Subcribes/callback: /initial_pose
	Publishes: /start_pose

	Callback function to retrieve the initial pose and display it in rviz
*/
void callback_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {

	start_pose.header.stamp = ros::Time::now();
	start_pose.header.frame_id = "map";
	start_pose.pose.position = pose->pose.pose.position;
	start_pose.pose.orientation = pose->pose.pose.orientation;
	syaw = tf::getYaw(start_pose.pose.orientation);

	start_pose_pub.publish(start_pose);
}


/*
	Subcribes/callback: /move_base_simple/goal
	Publishes: /goal_pose

	Callback function to retrieve the final pose and display it in rviz
*/
void callback_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& pose) {

	goal_pose.header.stamp = ros::Time::now();
	goal_pose.header.frame_id = "map";
	goal_pose.pose.position = pose->pose.position;
	goal_pose.pose.orientation = pose->pose.orientation;
	gyaw = tf::getYaw(goal_pose.pose.orientation);

	goal_pose_pub.publish(goal_pose);

	hybrid_astar_plan();
}


/*
	Subscribes/Callback: /map
	Publishes: None

	Callback function to retrieve the occupancy grid and construct a 2D binary obstacle map 
*/
void callback_map(const nav_msgs::OccupancyGrid::Ptr map) {

	grid = map;
	ROS_INFO("Recieved the occupancy grid map");

	grid_height = map->info.height;
	grid_width = map->info.width;
	bin_map = new bool*[grid_width];

	for (int x = 0; x < grid_width; ++x) { bin_map[x] = new bool[grid_height]; }

	for (int x = 0; x < grid_width; ++x) {
		for (int y = 0; y < grid_height; ++y) {
			bin_map[x][y] = map->data[y * grid_width + x] ? true : false;
		}
	}

	acc_obs_map = new int* [grid_width];

	for (int x = 0; x < grid_width; x++) {
		acc_obs_map[x] = new int[grid_height];
		for (int y = 0; y < grid_height; y++) {
			acc_obs_map[x][y] = (bin_map[x][y] > 0);
		}
	}

	for (int x = 0; x < grid_width; x++) {
		for (int y = 1; y < grid_height; y++) {
			acc_obs_map[x][y] = acc_obs_map[x][y-1] + acc_obs_map[x][y];
		}
	}

	for (int y = 0; y < grid_height; y++) {
		for (int x = 1; x < grid_width; x++) {
			acc_obs_map[x][y] = acc_obs_map[x-1][y] + acc_obs_map[x][y];
		}
	}
}


/*	
	Subscribes: /segments

	Callback function to retrieve the voronoi nodes
*/
void callback_voronoi_graph(const tuw_multi_robot_msgs::Graph &vg) {

	voronoi_graph = vg;
	voronoi_graph_received = true;

	voronoi_map();

	voronoi_graph_sub.shutdown();
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
	goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
	hybrid_path_pub = nh.advertise<nav_msgs::Path>("hybrid_astar_path", 10);
	hybrid_tractor_path_pub = nh.advertise<nav_msgs::Path>("hybrid_astar_tractor_path", 10);
	hybrid_trailer_path_pub = nh.advertise<nav_msgs::Path>("hybrid_astar_trailer_path", 10);
	dubins_path_pub = nh.advertise<nav_msgs::Path>("dubins_path", 10);
	reeds_shepp_path_pub = nh.advertise<nav_msgs::Path>("reeds_shepp_path", 10);
	visualize_nodes_pub = nh.advertise<visualization_msgs::Marker>("nodes", 10);
	robot_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_polygon", 10);
	robot_polyogn_array_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("robot_polygon_array", 10);
	trailer_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("trailer_polygon", 10);
	trailer_polyogn_array_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("trailer_polygon_array", 10);
	robot_center_pub = nh.advertise<geometry_msgs::PointStamped>("robot_center", 10);
	trailer_center_pub = nh.advertise<geometry_msgs::PointStamped>("trailer_center", 10);
	robot_collision_check_pub = nh.advertise<visualization_msgs::Marker>("robot_collision_check", 10);
	trailer_collision_check_pub = nh.advertise<visualization_msgs::Marker>("trailer_collision_check", 10);
	voronoi_path_pub = nh.advertise<visualization_msgs::Marker>("voronoi_path", 10);
	voronoi_sub_goals_pub = nh.advertise<visualization_msgs::MarkerArray>("voronoi_sub_goals", 10);
	astar_path_pub = nh.advertise<nav_msgs::Path>("astar_path", 10);
	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	target_point_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 10);

	// Subscribers
	ros::Subscriber start_pose_sub = nh.subscribe("initialpose", 10, callback_start_pose);
	ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, callback_goal_pose);
	ros::Subscriber map_sub = nh.subscribe("map", 1, callback_map);
	voronoi_graph_sub = nh.subscribe("segments", 1, callback_voronoi_graph);

	ros::spin();

	return 0;
}