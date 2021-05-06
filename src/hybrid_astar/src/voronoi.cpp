#include "../include/hybrid_astar/voronoi.h"

std::map<uint, std::vector<uint>> voronoi_nodes;
typedef pair<float, uint> pi;

void voronoi_map() {

	std::vector<uint> neighbours;
	std::vector<geometry_msgs::Point> redundunt_nodes;

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

float calc_node_cost(float x, float y, float gx, float gy, float cost_so_far) {
	return cost_so_far + (hypot(x - gx, y - gy));
}

float calc_yaw(float x_1, float y_1, float x_2, float y_2) {

	if(y_2 - y_1 > 0 && x_2 - x_1 > 0) {
		// ROS_INFO("Case 1");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 > 0 && x_2 - x_1 < 0) {
		// ROS_INFO("Case 2");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		return (3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 > 0) {
		// ROS_INFO("Case 3");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		return asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1));
	}

	if(y_2 - y_1 < 0 && x_2 - x_1 < 0) {
		// ROS_INFO("Case 4");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		return (-3.14 - asin((y_2 - y_1)/hypot(x_2 - x_1, y_2 - y_1)));
	}

	if(y_2 - y_1 == 0) {
		// ROS_INFO("Case 5");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		if(x_2 > x_1) {
			return 0.0;
		} else {
			return 3.14;
		}
	}

	if(x_2 - x_1 == 0) {
		// ROS_INFO("Case 6");
		// ROS_INFO("x_1: %f y_1: %f x_2: %f y_2: %f", x_1, y_1, x_2, y_2);
		if(y_2 > y_1) {
			return 1.57;
		} else {
			return -1.57;
		}
	}
}

void display_map(std::map<uint, Node2D> m) {
	std::map<uint, Node2D>::iterator itr;
	for (itr = m.begin(); itr != m.end(); ++itr) {
		cout << '\t' << itr->first << endl;
	}
}

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

std::vector<std::vector<float>> voronoi_path() {

	float syaw = tf::getYaw(start_pose.pose.orientation);
	float syaw_t = tf::getYaw(start_pose.pose.orientation);
	float sx = start_pose.pose.position.x;
	float sy = start_pose.pose.position.y;

	float gyaw = tf::getYaw(goal_pose.pose.orientation);
	float gx = goal_pose.pose.position.x;
	float gy = goal_pose.pose.position.y;

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

	// ROS_INFO("Voronoi Start ID: %u \t Vornoi Goal ID: %u", voronoi_start_id, voronoi_goal_id);

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

	visualization_msgs::MarkerArray voronoi_sub_goals;
	voronoi_sub_goals.markers.clear();

	visualization_msgs::Marker voronoi_sub_goal;
	voronoi_sub_goal.header.stamp = ros::Time::now();
	voronoi_sub_goal.header.frame_id = "/map";
	voronoi_sub_goal.ns = "voronoi_sub_goal";
	voronoi_sub_goal.action = visualization_msgs::Marker::ADD;
	voronoi_sub_goal.id = 0;
	voronoi_sub_goal.type = visualization_msgs::Marker::ARROW;
	voronoi_sub_goal.scale.x = 1.0;
	voronoi_sub_goal.scale.y = 0.05;
	voronoi_sub_goal.color.r = 1.0;
	voronoi_sub_goal.color.g = 0.0;
	voronoi_sub_goal.color.b = 0.0;
	voronoi_sub_goal.color.a = 1.0;
	voronoi_sub_goal.lifetime = ros::Duration(100);
	voronoi_sub_goal.frame_locked = true;
	// voronoi_sub_goal.points.clear();
	geometry_msgs::Point voronoi_neighbour;
		
	Node2D current_node;
	Node2D new_node;
	float node_cost;
	float cost_so_far = 0;
	pair<float, int> current_id;
	uint new_id;

	std::map<uint, Node2D> open_list;
	std::map<uint, Node2D> closed_list;

	open_list[voronoi_start_id] = Node2D(voronoi_graph.vertices[voronoi_start_id].path[0].x, voronoi_graph.vertices[voronoi_start_id].path[0].y, 0, NULL);
	// cout << "Open List - " << endl;
	// display_map(open_list);

	priority_queue<pi, vector<pi>, greater<pi>> pq;
	pq.push(make_pair(0, voronoi_start_id));
	// display_pq(pq);

	while(true) {

		// cin.get();

		if(open_list.empty()) {
			ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
			break;
		}

		// display_pq(pq);

		current_id = pq.top();
		pq.pop();

		current_node = open_list[current_id.second];
		cost_so_far = cost_so_far + current_node.get_cost();
		closed_list[current_id.second] = current_node;
		open_list.erase(current_id.second);
		// cout << "Open List - " << endl;
		// display_map(open_list);

		if(current_id.second == voronoi_goal_id) {
			ROS_INFO("VORONOI PATH FOUND");
			break;
		}
		
		for (int i = 0; i < voronoi_nodes[current_id.second].size(); ++i) {	
			// cin.get();

			node_cost = calc_node_cost(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, gx, gy, cost_so_far);
			node_cost = node_cost + hypot(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x - voronoi_graph.vertices[current_id.second].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y - voronoi_graph.vertices[current_id.second].path[0].y);
			new_node = Node2D(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, node_cost, current_id.second);

			new_id = voronoi_nodes[current_id.second][i];

			if(closed_list.count(new_id)) {
				continue;
			}

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

	sub_goals.push_back({gx, gy, gyaw});

	float yaw;
	geometry_msgs::Pose p;
	// p.position.x = voronoi_graph.vertices[current_id.second].path[0].x;
	// p.position.y = voronoi_graph.vertices[current_id.second].path[0].y;
	// yaw = calc_yaw(voronoi_graph.vertices[current_id.second].path[0].x, voronoi_graph.vertices[current_id.second].path[0].y, gx, gy);
	// tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
	// p.orientation.x = quat.x();
	// p.orientation.y = quat.y();
	// p.orientation.z = quat.z();
	// p.orientation.w = quat.w();
	// voronoi_sub_goal.pose = p;
	// voronoi_sub_goals.markers.push_back(voronoi_sub_goal);
	// sub_goals.push_back({p.position.x, p.position.y, yaw});

	// voronoi_path_points.points.push_back(voronoi_graph.vertices[current_id.second].path[0]);
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
		voronoi_sub_goal.pose = p;
		voronoi_sub_goals.markers.push_back(voronoi_sub_goal);
		redundunt_nodes.push_back(voronoi_graph.vertices[current_node.get_pind()].path[0]);
		sub_goals.push_back({p.position.x, p.position.y, yaw});
		current_node = closed_list[current_node.get_pind()];
	}
	voronoi_sub_goals_pub.publish(voronoi_sub_goals);
	voronoi_path_pub.publish(voronoi_path_points);

	reverse(sub_goals.begin(), sub_goals.end());

	return sub_goals;
}