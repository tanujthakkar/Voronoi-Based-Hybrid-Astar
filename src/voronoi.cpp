#include "../include/hybrid_astar/voronoi.h"

std::map<uint, std::vector<uint>> voronoi_nodes; // Map of voronoi nodes with their respective neighbours
typedef pair<float, uint> pi;


/*
	Function to create a map of voronoi nodes with their neighbours

	Returns: updates voronoi_nodes variable
*/
void voronoi_map() {

	std::vector<uint> neighbours; // Vector to store neighbours of a node
	std::vector<geometry_msgs::Point> redundunt_nodes; // Vector to store redundant nodes

	for(int i = 0; i < voronoi_graph.vertices.size(); ++i)
	{	
		if(!voronoi_nodes.count(voronoi_graph.vertices[i].id)) {
			neighbours.clear();
			redundunt_nodes.clear();
			redundunt_nodes.push_back(voronoi_graph.vertices[i].path[0]);

			for (int succ = 0; succ < voronoi_graph.vertices[i].successors.size(); ++succ)
			{	
				if(count(neighbours.begin(), neighbours.end(), voronoi_graph.vertices[i].successors[succ])) {
					continue;
				} else {
					redundunt_nodes.push_back(voronoi_graph.vertices[voronoi_graph.vertices[i].successors[succ]].path[0]);
					neighbours.push_back(voronoi_graph.vertices[i].successors[succ]);
				}
			}

			for (int pred = 0; pred < voronoi_graph.vertices[i].predecessors.size(); ++pred)
			{	
				if(count(neighbours.begin(), neighbours.end(), voronoi_graph.vertices[i].predecessors[pred])) {
					continue;
				} else {
					redundunt_nodes.push_back(voronoi_graph.vertices[voronoi_graph.vertices[i].predecessors[pred]].path[0]);
					neighbours.push_back(voronoi_graph.vertices[i].predecessors[pred]);
				}
			}

			voronoi_nodes[voronoi_graph.vertices[i].id] = neighbours;
		} else {
			continue;
		}
	}
}


/*
	Function to calculate node cost

	x: x coordinate of node
	y: y coordinate of node
	gx: goal x
	gy: goal y
	cost_so_far: cost of the path so far

	Returns: node cost
*/
float calc_node_cost(float x, float y, float gx, float gy, float cost_so_far) {
	return cost_so_far + sqrt(pow(x - gx, 2) + pow(y - gy, 2) + pow(abs(abs(abs(calc_yaw(x, y, sx, sy) - syaw) - 3.14) - 3.14), 2));
	// (hypot(x - gx, y - gy)) + (abs(abs(abs(calc_yaw(x, y, sx, sy) - syaw) - 3.14) - 3.14));
}


/*
	Function to calculate yaw/heading of nodes

	x_1: x coordinate of parent node
	y_1: y coordinate of parent node
	x_2: x coordinate of current node
	y_2: y coordinate of current node

	Returns: new node object
*/
float calc_yaw(float x_1, float y_1, float x_2, float y_2) {

	if(y_2 - y_1 > 0 && x_2 - x_1 > 0) {
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 > 0 && x_2 - x_1 < 0) {
		return (3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 > 0) {
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 < 0) {
		return (-3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 == 0) {
		if(x_2 > x_1) {
			return 0.0;
		} else {
			return 3.14;
		}
	}

	if(x_2 - x_1 == 0) {
		if(y_2 > y_1) {
			return 1.57;
		} else {
			return -1.57;
		}
	}
}


/*
	Function to display contents of a map

	m: map to be displayed
*/
void display_map(std::map<uint, Node2D> m) {
	std::map<uint, Node2D>::iterator itr;
	for (itr = m.begin(); itr != m.end(); ++itr) {
		cout << '\t' << itr->first << endl;
	}
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
	Function to compute the voronoi path
*/
std::vector<std::vector<float>> voronoi_path() {

	geometry_msgs::PointStamped voronoi_start;
	float nearest_voronoi_start = hypot(voronoi_graph.vertices[0].path[0].x  - sx, voronoi_graph.vertices[0].path[0].y - sy);
	uint voronoi_start_id;
	geometry_msgs::PointStamped voronoi_goal;
	float nearest_voronoi_goal = hypot(voronoi_graph.vertices[0].path[0].x  - gx, voronoi_graph.vertices[0].path[0].y - gy);
	uint voronoi_goal_id;

	voronoi_start.header.stamp = ros::Time::now();
	voronoi_start.header.frame_id = "/map";
	voronoi_goal.header.stamp = ros::Time::now();
	voronoi_goal.header.frame_id = "/map";

	for (int i = 0; i < voronoi_graph.vertices.size(); ++i) {
		if(nearest_voronoi_start >= hypot(voronoi_graph.vertices[i].path[0].x - sx, voronoi_graph.vertices[i].path[0].y - sy)) {
			nearest_voronoi_start = hypot(voronoi_graph.vertices[i].path[0].x - sx, voronoi_graph.vertices[i].path[0].y - sy);
			voronoi_start.point = voronoi_graph.vertices[i].path[0];
			voronoi_start_id = voronoi_graph.vertices[i].id;
			robot_center_pub.publish(voronoi_start);
		}
		if(nearest_voronoi_goal >= hypot(voronoi_graph.vertices[i].path[0].x - gx, voronoi_graph.vertices[i].path[0].y - gy)) {
			nearest_voronoi_goal = hypot(voronoi_graph.vertices[i].path[0].x - gx, voronoi_graph.vertices[i].path[0].y - gy);
			voronoi_goal.point = voronoi_graph.vertices[i].path[0];
			voronoi_goal_id = voronoi_graph.vertices[i].id;
			trailer_center_pub.publish(voronoi_goal);
		}
	}

	visualization_msgs::Marker voronoi_path_points;
	voronoi_path_points.header.stamp = ros::Time::now();
	voronoi_path_points.header.frame_id = "/map";
	voronoi_path_points.ns = "voronoi_path_points";
	voronoi_path_points.action = visualization_msgs::Marker::ADD;
	voronoi_path_points.id = 0;
	voronoi_path_points.type = visualization_msgs::Marker::POINTS;
	voronoi_path_points.scale.x = 0.2;
	voronoi_path_points.scale.y = 0.2;
	voronoi_path_points.color.r = 1.0;
	voronoi_path_points.color.g = 0.0;
	voronoi_path_points.color.b = 0.0;
	voronoi_path_points.color.a = 1.0;
	voronoi_path_points.points.clear();

	geometry_msgs::Point voronoi_neighbour;

	Node2D current_node; // Node2D object to track current node
	Node2D new_node; // Node2D object to track new nodes
	float node_cost;
	float cost_so_far = 0;
	pair<float, int> current_id; // Variable to track ID of current node
	uint new_id; // Variable to track ID of new node

	std::map<uint, Node2D> open_list; // Creating the open_list using a map
	std::map<uint, Node2D> closed_list; // Creating the closed_list using a map

	open_list[voronoi_start_id] = Node2D(voronoi_graph.vertices[voronoi_start_id].path[0].x, voronoi_graph.vertices[voronoi_start_id].path[0].y, 0, NULL);

	priority_queue<pi, vector<pi>, greater<pi>> pq; // Creating a min priority queue to sort nodes with respect to highest priority (lowest cost)
	pq.push(make_pair(0, voronoi_start_id)); // Adding the start node to the priority queue

	while(true) {

		if(open_list.empty()) {
			ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
			break;
		}

		current_id = pq.top(); // Retrieve the pair with the highest priority (lowest cost)
		pq.pop(); // Pop the pair with highest priority

		current_node = open_list[current_id.second]; // Update current node to the node with highest priority
		cost_so_far = cost_so_far + current_node.get_cost(); // Update cost_so_far
		closed_list[current_id.second] = current_node; // Added updated current node to the closed list
		open_list.erase(current_id.second); // Remove updated current node from open list

		// Check if goal is reached
		if(current_id.second == voronoi_goal_id) {
			break;
		}
		
		// Expand the current node
		for (int i = 0; i < voronoi_nodes[current_id.second].size(); ++i) {	

			node_cost = calc_node_cost(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, gx, gy, cost_so_far);
			node_cost = node_cost + hypot(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x - voronoi_graph.vertices[current_id.second].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y - voronoi_graph.vertices[current_id.second].path[0].y);
			new_node = Node2D(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, node_cost, current_id.second);

			new_id = voronoi_nodes[current_id.second][i]; // Updaign new_id

			// Checking if new node exists in closed list
			if(closed_list.count(new_id)) {
				continue;
			}

			// Checking if new node exists in open list
			if(!open_list.count(new_id)) {
				open_list[new_id] = new_node;
				pq.push(make_pair(new_node.get_cost(), new_id));
			} else {
				if(open_list[new_id].get_cost() > new_node.get_cost()) {
					open_list[new_id] = new_node;
				}
			}
		}
	}

	std::vector<std::vector<float>> sub_goals;
	std::vector<geometry_msgs::Point> redundunt_nodes;

	float yaw;
	geometry_msgs::Pose p;

	while(current_node.get_pind() != NULL) {
		if(count(redundunt_nodes.begin(), redundunt_nodes.end(), voronoi_graph.vertices[current_node.get_pind()].path[0])) {
			current_node = closed_list[current_node.get_pind()];
			continue;
		}
		voronoi_path_points.points.push_back(voronoi_graph.vertices[current_node.get_pind()].path[0]);
		p.position.x = voronoi_graph.vertices[current_node.get_pind()].path[0].x;
		p.position.y = voronoi_graph.vertices[current_node.get_pind()].path[0].y;
		yaw = calc_yaw(voronoi_graph.vertices[current_node.get_pind()].path[0].x, voronoi_graph.vertices[current_node.get_pind()].path[0].y, current_node.get_x(), current_node.get_y());
		tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
		p.orientation.x = quat.x();
		p.orientation.y = quat.y();
		p.orientation.z = quat.z();
		p.orientation.w = quat.w();
		redundunt_nodes.push_back(voronoi_graph.vertices[current_node.get_pind()].path[0]);
		sub_goals.push_back({p.position.x, p.position.y, yaw});
		current_node = closed_list[current_node.get_pind()];
		voronoi_path_pub.publish(voronoi_path_points);
	}
	voronoi_path_points.points.erase(voronoi_path_points.points.begin());
	voronoi_path_pub.publish(voronoi_path_points);

	sub_goals.erase(sub_goals.begin());
	reverse(sub_goals.begin(), sub_goals.end());
	sub_goals.push_back({gx, gy, gyaw});

	return sub_goals;
}