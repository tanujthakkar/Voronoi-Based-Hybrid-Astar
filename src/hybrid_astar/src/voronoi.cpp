#include "../include/hybrid_astar/voronoi.h"

std::map<uint, std::vector<uint>> voronoi_nodes;

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
				} else if(count(redundunt_nodes.begin(), redundunt_nodes.end(), voronoi_graph.vertices[voronoi_graph.vertices[i].successors[succ]].path[0])) {
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
				} else if(count(redundunt_nodes.begin(), redundunt_nodes.end(), voronoi_graph.vertices[voronoi_graph.vertices[i].predecessors[pred]].path[0])) {
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
	
	uint current_id = -1;
	uint new_id = -1;
	float cost_so_far;
	float node_cost;

	ROS_INFO("Check - 1");

	while(1) {

		current_id = voronoi_goal_id;

		for (int i = 0; i < voronoi_nodes[current_id].size(); ++i)
		{	
			// if(node_cost >=  hypot(voronoi_graph.vertices[voronoi_graph.vertices[current_id].successors[i]].path[0].x - voronoi_goal.point.x, voronoi_graph.vertices[voronoi_graph.vertices[current_id].successors[i]].path[0].y - voronoi_goal.point.y)) {
			// 	node_cost = hypot(voronoi_graph.vertices[voronoi_graph.vertices[current_id].successors[i]].path[0].x - voronoi_goal.point.x, voronoi_graph.vertices[voronoi_graph.vertices[current_id].successors[i]].path[0].y - voronoi_goal.point.y);
			// 	new_id = voronoi_graph.vertices[current_id].successors[i];
			// }
				ROS_INFO("neighbour id: %u", voronoi_nodes[current_id][i]);
				// cout << voronoi_graph.vertices[voronoi_graph.vertices[voronoi_start_id].successors[i]] << endl;
				voronoi_neighbours.points.push_back(voronoi_graph.vertices[voronoi_nodes[current_id][i]].path[0]);
		}
	}

	// ROS_INFO("%f\n", sin(voronoi_graph.vertices[next_id].path[0].x - voronoi_goal.point.x / hypot(voronoi_graph.vertices[next_id].path[0].x - voronoi_goal.point.x, voronoi_graph.vertices[next_id].path[0].y - voronoi_goal.point.y)));

	voronoi_nodes_points_pub.publish(voronoi_neighbours);
}