#include "../include/hybrid_astar/hybrid_astar.h"

using namespace std;
using namespace std::chrono;


// Global Variables
ros::Publisher start_pose_pub;
ros::Publisher goal_pose_pub;
ros::Publisher hybrid_path_pub;
ros::Publisher global_path_pub;
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

ros::Subscriber voronoi_graph_sub;

bool visualization = false; // Visualization toggle
bool visualization_final_node = true; // Final node visualization toggle

geometry_msgs::PoseStamped start_pose; // Start pose msg
bool valid_start; // Start pose validity check
geometry_msgs::PoseStamped goal_pose; // Goal pose msg
bool valid_goal; // Goal pose validity check

nav_msgs::OccupancyGrid::Ptr grid; // Pointer to the occupancy grid msg
int grid_height;
int grid_width;
bool** bin_map; // 2D Binary map of the grid 
int** acc_obs_map;

bool voronoi_graph_received = false; // Flag to stop computing voronoi graph
tuw_multi_robot_msgs::Graph voronoi_graph; // Voronoi graph representing the voronoi nodes

typedef pair<float, int> pi;

nav_msgs::Path path; // Hybrid A* path
bool path_found; // Solution found flag
nav_msgs::Path path_center; // Global Path w.r.t robot center
nav_msgs::Path dubins_path; // Dubins Path
visualization_msgs::Marker nodes; // All created nodes

jsk_recognition_msgs::PolygonArray robot_polygon_array;
jsk_recognition_msgs::PolygonArray trailer_polygon_array;

float sx; // x coordinate of current pose of the rear axle/hitch point of the vehicle
float sy; // y coordinate of current pose of the rear axle/hitch point of the vehicle
float syaw; // yaw coordinate of current pose of the rear axle/hitch point of the vehicle
float syaw_t; // yaw coordinate of current pose of the trailer
int s_ind;

float gx; // x coordinate of goal pose of the rear axle/hitch point of the vehicle
float gy; // y coordinate of goal pose of the rear axle/hitch point of the vehicle
float gyaw; // yaw coordinate of goal pose of the rear axle/hitch point of the vehicle
int g_ind;

float fx;
float fy;
float fyaw;
float fyaw_t;
int f_ind;

Node4D* start_node_ptr; // Pointer to the start node
Node4D* goal_node_ptr; // Pointer to the goal node

unsigned int iterations; // total iterations of hybrid a* planning
bool iteration_limit_flag;
unsigned int total_nodes; // total nodes of hybrid a* planning
unsigned int execution_time; // execution time of hybrid a* planning

std::vector<int> steer; // Steering inputs for which the nodes have to be created
std::vector<int> direction = {-1, 1}; // Direction inputs for which the nodes have to be created


/*
	Function to create a successor node

	node: input node to create successor for
	steer: steering input of the new node
	dir: direction of the new node

	Returns a pointer to the new node
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

	Returns a pointer to the new node
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
	// dubins_path_sample(&path, x, q);
	// dubins_xlist.push_back(q[0]);
	// dubins_ylist.push_back(q[1]);
	// dubins_yawlist.push_back(pi_2_pi(q[2]));
	dubins_yawtlist.push_back(pi_2_pi(start.get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(start.get_yaw(n-1) - start.get_yawt(n-1))));
	dubins_yawt.push_back(pi_2_pi(start.get_yaw_t(n-1) + (dir * MOVE_STEP / RTR) * sin(start.get_yaw(0) - start.get_yaw_t(0))));

	// pose_stamped.header.stamp = ros::Time::now();
	// pose_stamped.header.frame_id = "map";
	// pose_stamped.pose.position.x = q[0];
	// pose_stamped.pose.position.y = q[1];
	// pose_stamped.pose.position.z = 0;
	// pose_stamped.pose.orientation.w = pi_2_pi(q[2]);
	// dubins_path.poses.push_back(pose_stamped);

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
	Function to create a reeds-shepp node

	start: start node to start planning from
	goal: goal node

	Returns a pointer to the new node
*/
Node4D create_reeds_shepp_node(Node4D &start, Node4D &goal, float steer) {

	std::vector<int> directions;
	double steer_radius = steer;
	ReedsSheppStateSpace rs_planner(steer_radius);

	int n = start.get_size();

	double q0[3] = { start.get_x(n-1), start.get_y(n-1), start.get_yaw(n-1) }; // start
	double q1[3] = { goal.get_x(0), goal.get_y(0), goal.get_yaw(0) }; // goal

	std::vector<std::vector<double>> rs_path;
	double length = -1;
	directions = rs_planner.sample(q0, q1, MOVE_STEP, length, rs_path);

	std::vector<float> reeds_shepp_xlist;
	std::vector<float> reeds_shepp_ylist;
	std::vector<float> reeds_shepp_yawlist;
	std::vector<float> reeds_shepp_yawtlist;
	std::vector<float> reeds_shepp_yawt;

	nav_msgs::Path reeds_shepp_path;
	geometry_msgs::PoseStamped pose_stamped;
	reeds_shepp_path.header.frame_id = "map";
	reeds_shepp_path.header.stamp = ros::Time::now();
	pose_stamped.header = reeds_shepp_path.header;


	// reeds_shepp_xlist.push_back(start.get_x(n-1));
	// reeds_shepp_ylist.push_back(start.get_y(n-1));
	// reeds_shepp_yawlist.push_back(start.get_yaw(n-1));
	reeds_shepp_yawtlist.push_back(pi_2_pi(start.get_yawt(n-1) + (directions[0] * MOVE_STEP / RTR) * sin(start.get_yaw(n-1) - start.get_yawt(n-1))));
	reeds_shepp_yawt.push_back(pi_2_pi(start.get_yaw_t(n-1) + (directions[0] * MOVE_STEP / RTR) * sin(start.get_yaw(0) - start.get_yaw_t(0))));

	int i = 0;
	for (auto &point_itr : rs_path) {
		reeds_shepp_xlist.push_back(point_itr[0]);
		reeds_shepp_ylist.push_back(point_itr[1]);
		reeds_shepp_yawlist.push_back(point_itr[2]);
		// printf("%d ", directions[i]);
		reeds_shepp_yawtlist.push_back(pi_2_pi(reeds_shepp_yawtlist[i-1] + (directions[i] * MOVE_STEP / RTR) * sin(reeds_shepp_yawlist[i-1] - reeds_shepp_yawtlist[i-1])));
		reeds_shepp_yawt.push_back(pi_2_pi(reeds_shepp_yawt[i-1] + (directions[i] * MOVE_STEP / RTR) * sin(reeds_shepp_yawlist[i-1] - reeds_shepp_yawt[i-1])));
		i++;
		pose_stamped.pose.position.x = point_itr[0];
		pose_stamped.pose.position.y = point_itr[1];
		pose_stamped.pose.position.z = 0;
		pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(point_itr[2]);
		reeds_shepp_path.poses.push_back(pose_stamped);
	}

	reeds_shepp_yawtlist.erase(reeds_shepp_yawtlist.begin());
	reeds_shepp_yawt.erase(reeds_shepp_yawt.begin());

	// printf("%d\n", i);
	reeds_shepp_path_pub.publish(reeds_shepp_path);

	int nlist = reeds_shepp_xlist.capacity();
	int ind = (int)(round(reeds_shepp_yawlist[nlist-1]/YAW_RESOLUTION) * grid_width * grid_height + round(reeds_shepp_ylist[nlist-1]/XY_RESOLUTION) * grid_width + round(reeds_shepp_xlist[nlist-1]/XY_RESOLUTION));
	// reeds_shepp_path_pub.publish(reeds_shepp_path);
	// std::cout << "Reeds-Shepp Node created" << "\n";

	return Node4D(reeds_shepp_xlist, reeds_shepp_ylist, reeds_shepp_yawlist, reeds_shepp_yawtlist, reeds_shepp_yawt, -1, -1, 0.0, ind, start.get_ind());
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
		// cout << steer[i] << " ";
	}
}


/*
	Function to calculate the heuristic cost of the node

	n: pointer to the node

	Returns the total cost (g(n) + h(n))
*/
float calc_heuristic_cost(Node4D n) {
	int size = n.get_size();
	return n.get_cost() + (astar(n.get_x(size-1), n.get_y(size-1), gx, gy) * H_COST);
	// return n.get_cost() + (sqrt(pow((n.get_x(size-1) - gx),2) + pow((n.get_y(size-1) - gy), 2) + pow((n.get_yaw(size-1) - gyaw), 2)) * H_COST);
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
	Publishes the hybrid a* path using the current_node pointer
*/
void visualize_final_path(Node4D &current_node, Node4D &dubins_node, std::map<int, Node4D> &closed_list) {

	std::vector<float> xlist;
	std::vector<float> ylist;
	std::vector<float> yawlist;
	std::vector<float > yawt;

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "/map";
	path.poses.clear();

	int n;
	geometry_msgs::PoseStamped pose_stamped;
	n = dubins_node.get_size();
	for (int i = 0; i < n; i++) {
		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.header.frame_id = "map";
		pose_stamped.pose.position.x = dubins_node.get_x(n-i-1);
		pose_stamped.pose.position.y = dubins_node.get_y(n-i-1);
		pose_stamped.pose.position.z = 0;
		tf::Quaternion quat = tf::createQuaternionFromYaw(dubins_node.get_yaw(n-i-1));
		pose_stamped.pose.orientation.x = quat.x();
		pose_stamped.pose.orientation.y = quat.y();
		pose_stamped.pose.orientation.z = quat.z();
		pose_stamped.pose.orientation.w = quat.w();
		path.poses.push_back(pose_stamped);
		if(visualization_final_node) {
			xlist.push_back(pose_stamped.pose.position.x);
			ylist.push_back(pose_stamped.pose.position.y);
			yawlist.push_back(dubins_node.get_yaw(n-i-1));
			yawt.push_back(dubins_node.get_yaw_t(n-i-1));
		}
	}

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
			path.poses.push_back(pose_stamped);
			if(visualization_final_node) {
				xlist.push_back(pose_stamped.pose.position.x);
				ylist.push_back(pose_stamped.pose.position.y);
				yawlist.push_back(current_node.get_yaw(n-i-1));
				yawt.push_back(current_node.get_yaw_t(n-i-1));
			}
		}
		current_node = closed_list[current_node.get_parent_ind()];
	}

	reverse(path.poses.begin(), path.poses.end());

	reverse(xlist.begin(), xlist.end());
	reverse(ylist.begin(), ylist.end());
	reverse(yawlist.begin(), yawlist.end());
	reverse(yawt.begin(), yawt.end());

	hybrid_path_pub.publish(path);
	global_path_pub.publish(path);

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

	if(false) {
		
		float cx;
		float cy;
		float ctx;
		float cty;

		unsigned int sleep = 125000;

		cin.get();
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
	Publishes all the nodes from open and closed lists
*/
// void visualize_all_nodes() {

// 	nodes.header.stamp = ros::Time::now();
// 	nodes.header.frame_id = "/map";
// 	nodes.ns = "v_nodes";
// 	nodes.action = visualization_msgs::Marker::ADD;
// 	nodes.id = 0;
// 	nodes.type = visualization_msgs::Marker::LINE_LIST;
// 	nodes.scale.x = 0.02;
// 	nodes.color.r = 0.8;
// 	nodes.color.g = 0.59;
// 	nodes.color.b = 0.91;
// 	nodes.color.a = 1.0;

// 	geometry_msgs::Point p;
// 	while(current_node.get_parent() != nullptr) {
// 		for(int i=0;i<ceil(PATH_LENGTH/MOVE_STEP);i++) {
// 			p.x = current_node.get_x(i);
// 			p.y = current_node.get_y(i);
// 			p.z = 0;
// 			nodes.points.push_back(p);
// 		}
// 		current_node = current_node->get_parent();
// 	}
// }


/*
	Computes the Hybrid A* path
*/
bool hybrid_astar_plan() {

	if(1) {

		// path_found = false;

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

		// syaw = tf::getYaw(start_pose.pose.orientation);
		// syaw_t = tf::getYaw(start_pose.pose.orientation);
		// sx = (round((start_pose.pose.position.x - DELTAR * cos(syaw)) * 20) * 0.05);
		// sy = (round((start_pose.pose.position.y - DELTAR * sin(syaw)) * 20) * 0.05);

		// tf::StampedTransform transform_robot;
		// tf::StampedTransform transform_trailer;
		// tf::TransformListener listener;

		// listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));
		// listener.lookupTransform("map", "base_link", ros::Time(0), transform_robot);

		// syaw = tf::getYaw(transform_robot.getRotation());
		// sx = transform_robot.getOrigin().x() - deltar * cos(syaw);
		// sy = transform_robot.getOrigin().y() - deltar * sin(syaw);

		// listener.lookupTransform("map", "cargo_cart_link", ros::Time(0), transform_trailer);
		// syaw_t = tf::getYaw(transform_trailer.getRotation());

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
		trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, syaw));
		trailer_polyogn_array_pub.publish(trailer_polygon_array);

		Node4D start_node = Node4D(sx, sy, syaw, 0, syaw_t, s_ind);
		if(start_node.check_collision(grid, bin_map, acc_obs_map)) {
			ROS_INFO("INVALID START");
			valid_start = false;
			return false;
		}
		// ROS_INFO("VALID START - sx: %f sy: %f syaw: %f", sx, sy, syaw);
		valid_start = true;

		// Computing rear-axle/hitch-point pose for goal node
		// gyaw = tf::getYaw(goal_pose.pose.orientation);
		// gx = (round((goal_pose.pose.position.x - DELTAR * cos(gyaw)) * 20) * 0.05);
		// gy = (round((goal_pose.pose.position.y - DELTAR * sin(gyaw)) * 20) * 0.05);

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
		trailer_polygon_array.polygons.push_back(create_polygon(TL, TW, ctx, cty, gyaw));
		trailer_polyogn_array_pub.publish(trailer_polygon_array);

		Node4D goal_node = Node4D(gx, gy, gyaw, 0, gyaw, g_ind);
		if(goal_node.check_collision(grid, bin_map, acc_obs_map)) {
			ROS_INFO("INVALID GOAL");
			valid_goal = false;
			return false;
		}
		// ROS_INFO("VALID GOAL - gx: %f gy: %f gyaw: %f", gx, gy, gyaw);
		valid_goal = true;

		// Node4D start_node = sn;
		// Node4D goal_node = gn;

		ROS_INFO("Planning Hybrid A* path...");
		create_steer_inputs(45);

		Node4D current_node; // Object to store the current node
		Node4D new_node; // Object to store the next node
		Node4D sub_goal_node;

		iterations = 0;
		total_nodes = 0;

		std::vector<std::vector<float>> sub_goals = voronoi_path();
		int sub_goal_counter = 0;
		bool valid_sub_goal = false;
		int sub_ind;

		// new_node = create_dubins_node(start_node, goal_node);
		// if(!new_node.check_collision(grid, bin_map, acc_obs_map)) {
		// 	ROS_INFO("Ye");
		// }

		std::map<int, Node4D> open_list; // Creating the open_list using a map
		std::map<int, Node4D> closed_list; // Creating the closed_list using a map

		open_list.insert(pair<int, Node4D>(start_node.get_ind(), start_node)); // Adding the start node to the open list
		// cout << "Open List: " << open_list.begin()->first << " " << open_list.begin()->second << endl;
		// cout << "Open List (Added start node) - " << endl;
		// display_map(open_list);

		priority_queue<pi, vector<pi>, greater<pi>> pq; // Creating a min priority queue to sort nodes with respect to highest priority (lowest cost)
		pq.push(make_pair(calc_heuristic_cost(start_node), start_node.get_ind())); // Adding the start node to the priority queue
		// display_pq(pq);

		pair<float, int> ind;
		int new_ind;

		auto start = high_resolution_clock::now(); // Reading start time of planning

		iteration_limit_flag = false;

		while(iterations < 250000) {

			// cout << "Press ENTER for next iteration";
			// cin.get();

			iterations++;
			// cout << "Iteration: " << iterations << endl;

			if(open_list.empty()) {
				ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
				auto stop = high_resolution_clock::now(); // Reading end time of planning
				auto duration = duration_cast<milliseconds>(stop-start);
				ROS_INFO("Execution Time: %d milliseconds (%d seconds)", duration.count(), duration.count()/1000);
				execution_time = duration.count();
				visualize_nodes_pub.publish(nodes);
				return false;
			}

			ind = pq.top(); // Retrieve the pair with the highest priority (lowest cost)
			// cout << "Ind - " << endl;
			// cout << "\tCost: " << ind.first << " Index: " << ind.second << endl;

			pq.pop(); // Pop the pair with highest priority
			// display_pq(pq);

			current_node = open_list[ind.second];
			// cout << "Current Node: " << current_node.get_ind() << endl;
			closed_list[ind.second] = current_node;
			// cout << "Closed List (Added current node)- " << endl;
			// display_map(closed_list);
			open_list.erase(ind.second);
			// cout << "Open List - (Removed current node)" << endl;
			// display_map(open_list);

			valid_sub_goal = false;
			for (int i = sub_goals.size() - 1; i > sub_goal_counter; --i) {
				// ROS_INFO("DUBINS");
				// cin.get();
				sub_ind = (int)(round(sub_goals[i][2]/YAW_RESOLUTION) * grid_width * grid_height + round(sub_goals[i][1]/XY_RESOLUTION) * grid_width + round(sub_goals[i][0]/XY_RESOLUTION));
				sub_goal_node = Node4D(sub_goals[i][0], sub_goals[i][1], sub_goals[i][2], 0, sub_goals[i][2], sub_ind);
				new_node = create_dubins_node(current_node, sub_goal_node);
				if(!new_node.check_collision(grid, bin_map, acc_obs_map)) {
					// ROS_INFO("Sub goal found");
					pq.push(make_pair(calc_heuristic_cost(new_node), new_node.get_ind()));
					// display_pq(pq);
					open_list[new_node.get_ind()] = new_node;
					// cout << "Open List - (Add sub goal node)" << endl;
					// display_map(open_list);
					sub_goal_counter = i;
					valid_sub_goal = true;
					total_nodes++;
					if(sub_goal_counter == sub_goals.size() - 1) {
						ROS_INFO("SOLUTION FOUND - DUBINS NODE");
						auto stop = high_resolution_clock::now(); // Reading end time of planning
						auto duration = duration_cast<milliseconds>(stop-start);
						ROS_INFO("Iterations: %d Nodes: %d", iterations, total_nodes);
						ROS_INFO("Execution Time: %d milliseconds (%d seconds) \n", duration.count(), duration.count()/1000);
						execution_time = duration.count();
						visualize_final_path(current_node, new_node, closed_list);
						return true;
					}
					break;
				}
			}

			if(valid_sub_goal) {
				continue;
			}

			for (unsigned int d = 0; d < direction.size(); ++d) {
				for(unsigned int i = 0; i < steer.size() + 1; ++i) {
					
					// cout << "Press ENTER for next node";
					// cin.get();

					new_node = create_successor(current_node, steer[i], direction[d]);
					// cout << "New Node: " << new_node << " Parent: " << new_node->get_parent() << endl;

					if(new_node.check_collision(grid, bin_map, acc_obs_map)) {
						// ROS_INFO("NODE IN COLLISION");
						continue;
					}

					if(visualization) {
						for(int j=0;j<ceil(PATH_LENGTH/MOVE_STEP);j++) {
							p.x = new_node.get_x(j);
							p.y = new_node.get_y(j);
							p.z = 0;
							nodes.points.push_back(p);
						}
						visualize_nodes_pub.publish(nodes);
					}

					total_nodes++;
					// ROS_INFO("Total Nodes: %d", total_nodes);

					new_ind = new_node.get_ind();
					// cout << "New Index: " << new_ind << endl;

					if(closed_list.count(new_ind)) {
						continue;
					}

					if(!open_list.count(new_ind)) {
						open_list[new_ind] = new_node;
						// cout << "Open List (Added new node) - " << endl;
						// display_map(open_list);
						pq.push(make_pair(calc_heuristic_cost(new_node), new_node.get_ind()));
						// display_pq(pq);
					} else {
						if(open_list[new_ind].get_cost() > new_node.get_cost()) {
							open_list[new_ind] = new_node;
							// cout << "Open List (Updated node cost) - " << endl;
							// display_map(open_list);
						}
					}
				}
			}
		}

		// Node4D new_node_;
		// current_node = start_node;

		// for (unsigned int d = 0; d < direction.size(); ++d) {
		// 	for(unsigned int i = 0; i < steer.size() + 1; ++i) {
				
		// 		new_node = create_successor(current_node, steer[i], direction[d]);

		// 		for(int j=0;j<ceil(PATH_LENGTH/MOVE_STEP);j++) {
		// 			p.x = new_node.get_x(j);
		// 			p.y = new_node.get_y(j);
		// 			p.z = 0;
		// 			nodes.points.push_back(p);
		// 		}
		// 		visualize_nodes_pub.publish(nodes);

		// 		for (unsigned int d_ = 0; d_ < direction.size(); ++d_) {
		// 			for(unsigned int i_ = 0; i_ < steer.size() + 1; ++i_) {
		// 				new_node_ = create_successor(new_node, steer[i_], direction[d_]);

		// 				for(int j=0;j<ceil(PATH_LENGTH/MOVE_STEP);j++) {
		// 					p.x = new_node_.get_x(j);
		// 					p.y = new_node_.get_y(j);
		// 					p.z = 0;
		// 					nodes.points.push_back(p);
		// 				}
		// 				visualize_nodes_pub.publish(nodes);
		// 			}
		// 		}
		// 	}
		// }

		iteration_limit_flag = true;
		ROS_INFO("Iteration limit reached!\n");
		return false;
	}
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
	// astar(start_pose.pose.position.x, start_pose.pose.position.y, goal_pose.pose.position.x, goal_pose.pose.position.y);
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


/*
	Callback function to respond service call with the final global path
*/
bool callback_planner(hybrid_astar::GlobalPath::Request &req, hybrid_astar::GlobalPath::Response &res) {

	while(!path_found) {
	}
	res.plan = path_center;

	return true;
}


bool callback_monte_carlo(hybrid_astar::MonteCarloSim::Request &req, hybrid_astar::MonteCarloSim::Response &res) {
	
	sx = req.sx;
	sy = req.sy;
	syaw = req.syaw;
	syaw_t = req.syaw_t;

	gx = req.gx;
	gy = req.gy;
	gyaw = req.gyaw;

	res.solution_found = hybrid_astar_plan();
	res.valid_start = valid_start;
	res.valid_goal = valid_goal;
	res.path = path;
	res.iterations = iterations;
	res.iteration_limit = iteration_limit_flag;
	res.nodes = total_nodes;
	res.execution_time = execution_time;

	return true;
}


// void callback_global_path(const nav_msgs::Path &p) {

// 	float deltar = (RF - RB) / 2.0;
// 	float deltat = (RTF - RTB) / 2.0;

// 	geometry_msgs::PointStamped robot_center;
// 	robot_center.header.stamp = ros::Time::now();
// 	robot_center.header.frame_id = "/map";
// 	robot_center.point.x = p.poses[0].pose.position.x;
// 	robot_center.point.y = p.poses[0].pose.position.y;
// 	robot_center_pub.publish(robot_center);

// 	float cx = p.poses[0].pose.position.x;
// 	float cy = p.poses[0].pose.position.y;
// 	// tf::Quaternion quat = p.poses[0].pose.orientation;
// 	tf::Matrix3x3 m(p.poses[0].pose.orientation);
// 	// double roll, pitch, yaw;
// 	// m.getRPY(roll, pitch, yaw);
// 	// robot_polygon_pub.publish(create_polygon(RL, RW, cx, cy, yaw));

// 	// for(int i = 1; i < p.poses.size(); ++i) {
// 	// 	robot_center.point.x = p.poses[i].pose.position.x;
// 	// 	robot_center.point.y = p.poses[i].pose.position.y;
// 	// 	robot_center_pub.publish(robot_center);

// 	// 	cx = p.poses[i].pose.position.x;
// 	// 	cy = p.poses[i].pose.position.y;
// 	// 	tf::Quaternion quat = p.poses[i].pose.orientation;
// 	// 	tf::Matrix3x3 m(quat);
// 	// 	double roll, pitch, yaw;
// 	// 	m.getRPY(roll, pitch, yaw);
// 	// 	robot_polygon_pub.publish(create_polygon(RL, RW, cx, cy, yaw));
// 	// }
// }


int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
	goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
	hybrid_path_pub = nh.advertise<nav_msgs::Path>("hybrid_astar_path", 10);
	global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 10);
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

	// Subscribers
	ros::Subscriber start_pose_sub = nh.subscribe("initialpose", 10, callback_start_pose);
	ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, callback_goal_pose);
	ros::Subscriber map_sub = nh.subscribe("map", 1, callback_map);
	// ros::Subscriber map_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, callback_map);
	voronoi_graph_sub = nh.subscribe("segments", 1, callback_voronoi_graph);
	// ros::Subscriber global_path_sub = nh.subscribe("global_path", 10, callback_global_path);

	// Services
	// ros::ServiceServer pure_pursuit_service = nh.advertiseService("hybrid_astar_planner_service", callback_planner);
	ros::ServiceServer monte_carlo_sim_service = nh.advertiseService("monte_carlo_sim_service", callback_monte_carlo);

	ros::spin();
	// ros::Rate loop_rate(10);

	// while(ros::ok()) {

	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }

	return 0;
}