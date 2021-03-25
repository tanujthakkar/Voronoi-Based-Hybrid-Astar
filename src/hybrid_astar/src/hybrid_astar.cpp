#include "../include/hybrid_astar/hybrid_astar.h"

using namespace std;
using namespace std::chrono;


// Global Variables
ros::Publisher start_pose_pub;
ros::Publisher goal_pose_pub;
ros::Publisher hybrid_path_pub;
ros::Publisher global_path_pub;
ros::Publisher dubins_path_pub;
ros::Publisher visualize_nodes_pub;
ros::Publisher robot_polygon_pub;
ros::Publisher trailer_polygon_pub;
ros::Publisher robot_center_pub;
ros::Publisher trailer_center_pub;
ros::Publisher robot_collision_check_pub;
ros::Publisher trailer_collision_check_pub;

bool visualization = true; // Visualization toggle

geometry_msgs::PoseStamped start_pose; // Start pose msg
bool valid_start; // Start pose validity check
geometry_msgs::PoseStamped goal_pose; // Goal pose msg
bool valid_goal; // Goal pose validity check

nav_msgs::OccupancyGrid::Ptr grid; // Pointer to the occupancy grid msg
int grid_height;
int grid_width;
bool** bin_map; // 2D Binary map of the grid 
int** acc_obs_map;

typedef pair<float, int> pi;

nav_msgs::Path path; // Hybrid A* path
bool path_found; // Solution found flag
nav_msgs::Path path_center; // Global Path w.r.t robot center
nav_msgs::Path dubins_path; // Dubins Path
visualization_msgs::Marker nodes; // All created nodes

Node4D* current_node; // Pointer to the current node
Node4D* new_node; // Pointer to the next node

float sx; // x coordinate of current pose of the rear axle/hitch point of the vehicle
float sy; // y coordinate of current pose of the rear axle/hitch point of the vehicle
float syaw; // yaw coordinate of current pose of the rear axle/hitch point of the vehicle
float syaw_t; // yaw coordinate of current pose of the trailer

float gx; // x coordinate of goal pose of the rear axle/hitch point of the vehicle
float gy; // y coordinate of goal pose of the rear axle/hitch point of the vehicle
float gyaw; // yaw coordinate of goal pose of the rear axle/hitch point of the vehicle

int iterations; // total iterations of hybrid a* planning
int total_nodes; // total nodes of hybrid a* planning

std::vector<int> steer = {-45, -30, -15, 0, 15, 30, 45}; // Steering inputs for which the nodes have to be created


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

	xlist[0] = node->get_x(n-1) + (dir * MOVE_STEP) * cos(node->get_yaw(n-1));
	ylist[0] = node->get_y(n-1) + (dir * MOVE_STEP) * sin(node->get_yaw(n-1));
	yawlist[0] = pi_2_pi(node->get_yaw(n-1) + (dir * MOVE_STEP / WHEELBASE) * tan(to_rad(steer)));
	yawtlist[0] = pi_2_pi(node->get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(node->get_yaw(n-1) - node->get_yawt(n-1)));
	yawt[0] = pi_2_pi(node->get_yaw_t(0) + (dir * MOVE_STEP / RTR) * sin(node->get_yaw(0) - node->get_yaw_t(0)));
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
		// cout << "Cost : " << cost << endl;
	} else {
		cost = nlist * MOVE_STEP + BACKWARD_COST; // Penalizing backward motion
		// cout << "Cost : " << cost << endl;
	}

	cost = cost + abs(steer) * STEER_ANGLE_COST; // Penalizing steering angle
	// cout << "Cost : " << cost << endl;
	cost = cost + abs(node->get_steer() - steer) * STEER_CHANGE_COST; // Penalizing change in steering angle
	// cout << "Cost : " << cost << endl;
	cost = cost + jacknife_sum * JACKNIFE_COST; // Penalizing jacknifing

	cost = cost + node->get_cost();
	// cout << "Cost : " << cost << endl;

	return new Node4D(xlist, ylist, yawlist, yawtlist, yawt, dir, steer, cost, node);
}

/*
	Function to create a dubins node

	start: start node to start planning from
	goal: goal node

	Returns a pointer to the new node
*/
Node4D* create_dubins_node(Node4D* start, const Node4D goal) {

	int n = start->get_size();
	int dir = 1;

	double q0[] = { start->get_x(n-1), start->get_y(n-1), start->get_yaw(n-1) }; // start
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
	dubins_path_sample(&path, x, q);
	dubins_xlist.push_back(q[0]);
	dubins_ylist.push_back(q[1]);
	dubins_yawlist.push_back(pi_2_pi(q[2]));
	dubins_yawtlist.push_back(pi_2_pi(start->get_yawt(n-1) + (dir * MOVE_STEP / RTR) * sin(start->get_yaw(n-1) - start->get_yawt(n-1))));
	dubins_yawt.push_back(pi_2_pi(start->get_yaw_t(0) + (dir * MOVE_STEP / RTR) * sin(start->get_yaw(0) - start->get_yaw_t(0))));

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

	// dubins_path_pub.publish(dubins_path);
	// std::cout << "Dubins Node created" << "\n";

	return new Node4D(dubins_xlist, dubins_ylist, dubins_yawlist, dubins_yawtlist, dubins_yawt, 1, -1, 0.0, start);
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
		cout << steer[i] << " ";
	}
}


/*
	Function to calculate the index of the node to check for redundant nodes in open and close lists

	n: Pointer to the node

	Returns the index (int)
*/
int calc_index(Node4D* n) {

	int size = n->get_size();
	return (int)(round(n->get_yaw(size-1)/YAW_RESOLUTION) * grid_width * grid_height + round(n->get_y(size-1)/XY_RESOLUTION) * grid_width + round(n->get_x(size-1)/XY_RESOLUTION));
}


/*
	Function to calculate the heuristic cost of the node

	n: pointer to the node

	Returns the total cost (g(n) + h(n))
*/
float calc_heuristic_cost(Node4D* n) {
	int size = n->get_size();
	return (n->get_cost() + sqrt(pow((n->get_x(size-1) - gx),2) + pow((n->get_y(size-1) - gy), 2) + pow((n->get_yaw(size-1) - gyaw), 2)));
}


/*
	Function to display contents of a map

	m: map to be displayed
*/
void display_map(map<int, Node4D*> m) {

	map<int, Node4D*>::iterator itr;
	if(m.empty()) {
		cout << "List EMPTY" << endl;
	}
	for (itr = m.begin(); itr != m.end(); ++itr) {
		cout << "\tIndex: " << itr->first << " Node*: " << itr->second << endl;
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
void visualize_final_path() {

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "/map";
	path.poses.clear();

	int n;
	geometry_msgs::PoseStamped pose_stamped;
	while(current_node->get_parent() != nullptr) {
		n = current_node->get_size();
		for (int i = 0; i < n; i++) {
			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.header.frame_id = "map";
			pose_stamped.pose.position.x = current_node->get_x(n-i-1);
			pose_stamped.pose.position.y = current_node->get_y(n-i-1);
			pose_stamped.pose.position.z = 0;
			tf::Quaternion quat = tf::createQuaternionFromYaw(current_node->get_yaw(n-i-1));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();;
			path.poses.push_back(pose_stamped);
		}
		current_node->get_parent()->set_child(current_node);
		current_node = current_node->get_parent();
	}

	hybrid_path_pub.publish(path);
}


/*
	Publishes the final path using the current_node pointer
*/
void visualize_final_path_center() {

	path_center.header.stamp = ros::Time::now();
	path_center.header.frame_id = "/map";
	path_center.poses.clear();

	int n;
	float deltar = (RF - RB) / 2.0;
	float yaw;
	geometry_msgs::PoseStamped pose_stamped;
	while(current_node->get_child() != nullptr) {
		n = current_node->get_size();
		for (int i = 0; i < n; i++) {
			pose_stamped.header.stamp = ros::Time::now();
			pose_stamped.header.frame_id = "map";
			pose_stamped.pose.position.x = current_node->get_x(i) + deltar * cos(current_node->get_yaw(i));
			pose_stamped.pose.position.y = current_node->get_y(i) + deltar * sin(current_node->get_yaw(i));
			pose_stamped.pose.position.z = 0;
			tf::Quaternion quat = tf::createQuaternionFromYaw(current_node->get_yaw(i));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();;
			path_center.poses.push_back(pose_stamped);
		}
		current_node = current_node->get_child();
	}

	global_path_pub.publish(path_center);
}


/*
	Publishes all the nodes from open and closed lists
*/
void visualize_all_nodes(std::map<int, Node4D*> open, std::map<int, Node4D*> closed) {

	nodes.header.stamp = ros::Time::now();
	nodes.header.frame_id = "/map";
	nodes.ns = "v_nodes";
	nodes.action = visualization_msgs::Marker::ADD;
	nodes.id = 0;
	nodes.type = visualization_msgs::Marker::LINE_LIST;
	nodes.scale.x = 0.02;
	nodes.color.r = 0.8;
	nodes.color.g = 0.59;
	nodes.color.b = 0.91;
	nodes.color.a = 1.0;

	geometry_msgs::Point p;
	while(current_node->get_parent() != nullptr) {
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
	Computes the Hybrid A* path
*/
bool hybrid_astar_plan() {

	if(1) {

		path_found = false;

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

		iterations = 0;
		total_nodes = 0;

		// Computing rear-axle/hitch-point pose for start node
		// float deltar = (RF - RB) / 2.0;
		// syaw = tf::getYaw(start_pose.pose.orientation);
		// sx = start_pose.pose.position.x - deltar * cos(syaw);
		// sy = start_pose.pose.position.y - deltar * sin(syaw);

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

		Node4D start_node = Node4D(sx, sy, syaw, 0, syaw_t);
		if(start_node.check_collision(grid, bin_map, acc_obs_map)) {
			ROS_INFO("INVALID START");
			valid_start = false;
			return false;
		}
		ROS_INFO("VALID START - sx: %f sy: %f syaw: %f", sx, sy, syaw);
		valid_start = true;

		current_node = &start_node;

		// Computing rear-axle/hitch-point pose for goal node
		// deltar = (RF - RB) / 2.0;
		// gyaw = tf::getYaw(goal_pose.pose.orientation);
		// gx = goal_pose.pose.position.x - deltar * cos(gyaw);
		// gy = goal_pose.pose.position.y - deltar * sin(gyaw);

		Node4D goal_node = Node4D(gx, gy, gyaw, 0, 0);
		if(goal_node.check_collision(grid, bin_map, acc_obs_map)) {
			ROS_INFO("INVALID GOAL");
			valid_goal = false;
			return false;
		}
		ROS_INFO("VALID GOAL - gx: %f gy: %f gyaw: %f", gx, gy, gyaw);
		valid_goal = true;

		ROS_INFO("Planning Hybrid A* path...");
		// create_steer_inputs(30);

		std::map<int, Node4D*> open_list; // Creating the open_list using a map
		std::map<int, Node4D*> closed_list; // Creating the closed_list using a map

		open_list.insert(pair<int, Node4D*>(calc_index(current_node), current_node)); // Adding the start node to the open list
		// cout << "Open List: " << open_list.begin()->first << " " << open_list.begin()->second << endl;
		// cout << "Open List (Added start node) - " << endl;
		// display_map(open_list);

		priority_queue<pi, vector<pi>, greater<pi>> pq; // Creating a min priority queue to sort nodes with respect to highest priority (lowest cost)
		pq.push(make_pair(calc_heuristic_cost(current_node), calc_index(current_node))); // Adding the start node to the priority queue
		// display_pq(pq);

		pair<float, int> ind;
		int new_ind;

		while(true) {

			// cout << "Press ENTER for next iteration";
			// cin.get();

			iterations++;
			// cout << "Iteration: " << iterations << endl;

			if(open_list.empty()) {
				ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
				return false;
			}

			ind = pq.top(); // Retrieve the pair with the highest priority (lowest cost)
			// cout << "Ind - " << endl;
			// cout << "\tCost: " << ind.first << " Index: " << ind.second << endl;

			pq.pop(); // Pop the pair with highest priority
			// display_pq(pq);

			current_node = open_list[ind.second];
			// cout << "Current Node: " << current_node << endl;
			closed_list[ind.second] = current_node;
			// cout << "Closed List (Added current node)- " << endl;
			// display_map(closed_list);
			open_list.erase(ind.second);
			// cout << "Open List - (Removed current node)" << endl;
			// display_map(open_list);

			new_node = create_dubins_node(current_node, goal_node);
			if(!new_node->check_collision(grid, bin_map, acc_obs_map)) {
				cout << "SOLUTION FOUND - DUBINS NODE" << endl;
				printf("Iterations: %d Nodes: %d \n", iterations, total_nodes);
				new_node->set_child(&goal_node);
				current_node = new_node;
				visualize_final_path();
				visualize_final_path_center();
				if(visualization) {
					visualize_nodes_pub.publish(nodes);
				}
				path_found = true;
				return true;
			}

			for(int i = 0; i < steer.capacity(); ++i) {
				
				// cout << "Press ENTER for next node";
				// cin.get();

				new_node = create_successor(current_node, steer[i], 1);
				// cout << "New Node: " << new_node << " Parent: " << new_node->get_parent() << endl;

				if(new_node->check_collision(grid, bin_map, acc_obs_map)) {
					// ROS_INFO("NODE IN COLLISION");
					continue;
				}

				if(visualization) {
					for(int j=0;j<ceil(PATH_LENGTH/MOVE_STEP);j++) {
						p.x = new_node->get_x(j);
						p.y = new_node->get_y(j);
						p.z = 0;
						nodes.points.push_back(p);
					}
					visualize_nodes_pub.publish(nodes);
				}

				total_nodes++;
				// ROS_INFO("Total Nodes: %d", total_nodes);

				new_ind = calc_index(new_node);
				// cout << "New Index: " << new_ind << endl;

				if(closed_list.count(new_ind)) {
					continue;
				}

				if(!open_list.count(new_ind)) {
					open_list[new_ind] = new_node;
					// cout << "Open List (Added new node) - " << endl;
					// display_map(open_list);
					pq.push(make_pair(calc_heuristic_cost(new_node), calc_index(new_node)));
					// display_pq(pq);
				} else {
					if(open_list[new_ind]->get_cost() > new_node->get_cost()) {
						open_list[new_ind] = new_node;
						// cout << "Open List (Updated node cost) - " << endl;
						// display_map(open_list);
					}
				}
			}
		}
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

	ROS_INFO("START - X: %f \t Y: %f \t YAW: %f", start_pose.pose.position.x, start_pose.pose.position.y, syaw);
	if(grid->info.height >= start_pose.pose.position.y && start_pose.pose.position.y >= 0 && 
		grid->info.width >= start_pose.pose.position.x && start_pose.pose.position.x >= 0 && bin_map[(int)round(start_pose.pose.position.x/XY_RESOLUTION)][(int)round(start_pose.pose.position.y/XY_RESOLUTION)] == 0) {
		valid_start = true;
		ROS_INFO("VALID START!");
		start_pose_pub.publish(start_pose);
		// if(valid_goal) {
		// 	auto start = high_resolution_clock::now(); // Reading start time of planning
		// 	hybrid_astar_plan();
		// 	auto stop = high_resolution_clock::now(); // Reading end time of planning
		// 	auto duration = duration_cast<milliseconds>(stop-start);
		// 	std::cout << "Execution Time : " << duration.count() << " milliseconds (" << duration.count()/1000 << " seconds)" << endl;
		// } else {
		// 	ROS_INFO("NO VALID GOAL FOUND!");
		// }
	} else {
		valid_start = false;
		ROS_INFO("INVALID START!");
	}
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

	ROS_INFO("GOAL - X: %f \t Y: %f \t YAW: %f", goal_pose.pose.position.x, goal_pose.pose.position.y, gyaw);
	if (grid->info.height >= goal_pose.pose.position.y && goal_pose.pose.position.y >= 0 && 
		grid->info.width >= goal_pose.pose.position.x && goal_pose.pose.position.x >= 0 && bin_map[(int)round(goal_pose.pose.position.x/XY_RESOLUTION)][(int)round(goal_pose.pose.position.y/XY_RESOLUTION)] == 0) {
		valid_goal = true;
		ROS_INFO("VALID GOAL!");
		goal_pose_pub.publish(goal_pose);
		if(valid_start) {
			auto start = high_resolution_clock::now(); // Reading start time of planning
			hybrid_astar_plan();
			auto stop = high_resolution_clock::now(); // Reading end time of planning
			auto duration = duration_cast<milliseconds>(stop-start);
			std::cout << "Execution Time : " << duration.count() << " milliseconds (" << duration.count()/1000 << " seconds)" << endl;
		} else {
			ROS_INFO("NO VALID START FOUND!");
		}
	} else  {
		valid_goal = false;
		ROS_INFO("INVALID GOAL!");
	}
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

	auto start = high_resolution_clock::now(); // Reading start time of planning
	res.solution_found = hybrid_astar_plan();
	auto stop = high_resolution_clock::now(); // Reading end time of planning
	auto execution_time = duration_cast<milliseconds>(stop-start);
	res.valid_start = valid_start;
	res.valid_goal = valid_goal;
	res.path = path_center;
	res.iterations = iterations;
	res.nodes = total_nodes;
	res.execution_time = execution_time.count();

	return true;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("start_pose", 10);
	goal_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 10);
	hybrid_path_pub = nh.advertise<nav_msgs::Path>("hybrid_astar_path", 10);
	global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 10);
	dubins_path_pub = nh.advertise<nav_msgs::Path>("dubins_path", 10);
	visualize_nodes_pub = nh.advertise<visualization_msgs::Marker>("nodes", 10);
	robot_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("robot_polygon", 10);
	trailer_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("trailer_polygon", 10);
	robot_center_pub = nh.advertise<geometry_msgs::PointStamped>("robot_center", 10);
	trailer_center_pub = nh.advertise<geometry_msgs::PointStamped>("trailer_center", 10);
	robot_collision_check_pub = nh.advertise<visualization_msgs::Marker>("robot_collision_check", 10);
	trailer_collision_check_pub = nh.advertise<visualization_msgs::Marker>("trailer_collision_check", 10);

	// Subscribers
	ros::Subscriber start_pose_sub = nh.subscribe("initialpose", 10, callback_start_pose);
	ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 10, callback_goal_pose);
	ros::Subscriber map_sub = nh.subscribe("map", 1, callback_map);
	// ros::Subscriber map_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, callback_map);

	// Services
	// ros::ServiceServer pure_pursuit_service = nh.advertiseService("hybrid_astar_planner_service", callback_planner);
	ros::ServiceServer monte_carlo_sim_service = nh.advertiseService("monte_carlo_sim_service", callback_monte_carlo);

	ros::Rate loop_rate(10);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}