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

void voronoi_path() {

	float syaw = tf::getYaw(start_pose.pose.orientation);
	float syaw_t = tf::getYaw(start_pose.pose.orientation);
	float sx = start_pose.pose.position.x - DELTAR * cos(syaw);
	float sy = start_pose.pose.position.y - DELTAR * sin(syaw);

	float gyaw = tf::getYaw(goal_pose.pose.orientation);
	float gx = goal_pose.pose.position.x - DELTAR * cos(gyaw);
	float gy = goal_pose.pose.position.y - DELTAR * sin(gyaw);

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

	ROS_INFO("Voronoi Start ID: %u \t Vornoi Goal ID: %u", voronoi_start_id, voronoi_goal_id);

	visualization_msgs::Marker voronoi_neighbours;
	voronoi_neighbours.header.stamp = ros::Time::now();
	voronoi_neighbours.header.frame_id = "/map";
	voronoi_neighbours.ns = "voronoi_neighbours";
	voronoi_neighbours.action = visualization_msgs::Marker::ADD;
	voronoi_neighbours.id = 0;
	voronoi_neighbours.type = visualization_msgs::Marker::POINTS;
	voronoi_neighbours.scale.x = 0.2;
	voronoi_neighbours.scale.y = 0.2;
	voronoi_neighbours.color.r = 1.0;
	voronoi_neighbours.color.g = 0.0;
	voronoi_neighbours.color.b = 0.0;
	voronoi_neighbours.color.a = 1.0;
	voronoi_neighbours.points.clear();
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
	cout << "Open List - " << endl;
	display_map(open_list);

	priority_queue<pi, vector<pi>, greater<pi>> pq;
	pq.push(make_pair(0, voronoi_start_id));
	display_pq(pq);

	while(true) {

		// cin.get();

		if(open_list.empty()) {
			ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
			break;
		}

		display_pq(pq);

		current_id = pq.top();
		pq.pop();

		current_node = open_list[current_id.second];
		cost_so_far = cost_so_far + current_node.get_cost();
		closed_list[current_id.second] = current_node;
		open_list.erase(current_id.second);
		cout << "Open List - " << endl;
		display_map(open_list);

		voronoi_neighbours.points.push_back(voronoi_graph.vertices[current_id.second].path[0]);
		voronoi_nodes_points_pub.publish(voronoi_neighbours);

		if(current_id.second == voronoi_goal_id) {
			ROS_INFO("VORONOI PATH FOUND");
			break;
		}

		voronoi_neighbours.points.clear();
		
		for (int i = 0; i < voronoi_nodes[current_id.second].size(); ++i) {	
			// cin.get();

			node_cost = calc_node_cost(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, gx, gy, cost_so_far);
			new_node = Node2D(voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].x, voronoi_graph.vertices[voronoi_nodes[current_id.second][i]].path[0].y, node_cost, current_id.second);

			new_id = voronoi_nodes[current_id.second][i];

			voronoi_neighbours.points.push_back(voronoi_graph.vertices[new_id].path[0]);
			voronoi_nodes_points_pub.publish(voronoi_neighbours);

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

	voronoi_neighbours.points.clear();

	voronoi_neighbours.points.push_back(voronoi_graph.vertices[current_id.second].path[0]);
	while(current_node.get_pind() != NULL) {
		voronoi_neighbours.points.push_back(voronoi_graph.vertices[current_node.get_pind()].path[0]);
		current_node = closed_list[current_node.get_pind()];
	}
	voronoi_nodes_points_pub.publish(voronoi_neighbours);
}