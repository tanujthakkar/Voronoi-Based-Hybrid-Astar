#include "../include/hybrid_astar/node4d.h"

using namespace std;

jsk_recognition_msgs::PolygonArray system_polygons;

/*
	Function to check point collision

	bin_map: A 2D binary obstacle map

	Returns a boolean, true for collision, false otherwise
*/
bool Node4D::check_path_collision(bool** bin_map) {

	for (int i = 0; i < xlist.capacity(); ++i)
	{
		if(bin_map[(int)round(xlist[i]/XY_RESOLUTION)][(int)round(ylist[i]/XY_RESOLUTION)]) {
			ROS_INFO("IN COLLISION!");
			return true;
		}
	}

	ROS_INFO("NO COLLISION!");
	return false; // NO collision
}

/*
	Function to check node collision
	
	grid: Pointer to the occupancy grid msg
	bin_map: A 2D binary obstacle map

	Returns: boolean, true for collision, false otherwise
*/
bool Node4D::check_collision(nav_msgs::OccupancyGrid::Ptr grid, bool** bin_map, int** acc_obs_map) {

	int max_x;
	int min_x;
	int max_y;
	int min_y;

	float deltar;
	float deltat;

	int s;
	int t;

	visualization_msgs::Marker robot_collision_check_points;
	visualization_msgs::Marker trailer_collision_check_points;

	// Robot/Tractor Collision Check Variables
	robot_collision_check_points.header.stamp = ros::Time::now();
	robot_collision_check_points.header.frame_id = "/map";
	robot_collision_check_points.ns = "robot_collision_check_points";
	robot_collision_check_points.action = visualization_msgs::Marker::ADD;
	robot_collision_check_points.id = 0;
	robot_collision_check_points.type = visualization_msgs::Marker::POINTS;
	robot_collision_check_points.scale.x = 0.05;
	robot_collision_check_points.scale.y = 0.05;
	robot_collision_check_points.color.g = 1.0;
	robot_collision_check_points.color.a = 1.0;
	geometry_msgs::Point robot_collision_check_point;

	float cx; // Center x of the robot
	float cy; // Center y of the robot

	deltar = (RF - RB) / 2.0;

	// Trailer Collision Check Variables
	trailer_collision_check_points.header.stamp = ros::Time::now();
	trailer_collision_check_points.header.frame_id = "/map";
	trailer_collision_check_points.ns = "trailer_collision_check_points";
	trailer_collision_check_points.action = visualization_msgs::Marker::ADD;
	trailer_collision_check_points.id = 0;
	trailer_collision_check_points.type = visualization_msgs::Marker::POINTS;
	trailer_collision_check_points.scale.x = 0.05;
	trailer_collision_check_points.scale.y = 0.05;
	trailer_collision_check_points.color.b = 1.0;
	trailer_collision_check_points.color.a = 1.0;
	geometry_msgs::Point trailer_collision_check_point;

	float ctx; // Center x of the trailer
	float cty; // Center y of the trailer

	deltat = (RTF - RTB) / 2.0;

	for (int i = 0; i < xlist.capacity(); ++i) {

		// printf("xlist: %f ylist: %f yaw: %f yawt: %f yaw_t: %f \n", xlist[i], ylist[i], yawlist[i], yawtlist[i], yawt[i]);
		// cout << "Press ENTER for collision check " << i << endl;
		// cin.get();

		// Checking for jackknifing
		if(abs(abs(yawlist[i] - yawt[i]) - 3.14) <= 1.744) {
			// ROS_INFO("SELF-COLLISION - JACKNIFE");
			return true;
		}
		
		// Computing center of tractor from hitch position
		cx = xlist[i] + deltar * cos(yawlist[i]);
		cy = ylist[i] + deltar * sin(yawlist[i]);

		// Visualize robot center
		if(visualization) {	
			geometry_msgs::PointStamped robot_center;
			robot_center.header.stamp = ros::Time::now();
			robot_center.header.frame_id = "/map";
			robot_center.point.x = xlist[i];
			robot_center.point.y = ylist[i];
			robot_center_pub.publish(robot_center);
			robot_polygon_pub.publish(create_polygon(RL, RW, cx, cy, yawlist[i]));
		}

		// Checking for out of bounds robot state
		if(xlist[i] >= grid->info.width || xlist[i]<0 || ylist[i] >= grid->info.height || ylist[i] < 0) {
			// ROS_INFO("ROBOT OUT OF BOUNDS");
			return true; // OUT OF BOUNDS
		}
		
		max_x =  (cx + RL * abs(cos(yawlist[i]))/2 + RW * abs(sin(yawlist[i]))/2) + MIN_SAFE_DIST;
		min_x =  (cx - RL * abs(cos(yawlist[i]))/2 - RW * abs(sin(yawlist[i]))/2) - MIN_SAFE_DIST;

		max_y =  (cy + RL * abs(sin(yawlist[i]))/2 + RW * abs(cos(yawlist[i]))/2) + MIN_SAFE_DIST;
		min_y =  (cy - RL * abs(sin(yawlist[i]))/2 - RW * abs(cos(yawlist[i]))/2) - MIN_SAFE_DIST;
		
		if(max_x >= grid->info.width || min_x < 0 || max_y >= grid->info.height || min_y < 0) {
			// ROS_INFO("ROBOT OUT OF BOUNDS");
			return true; // OUT OF BOUNDS
		}

		// Checking the robot polygon for collision
		for(float k = -RL/2; k <= RL/2 + MIN_SAFE_DIST; k += RL/4) {
			for(float j = -RW/2 - MIN_SAFE_DIST; j <= RW/2 + MIN_SAFE_DIST; j += RW/4) {

				// Top Right
				s = (cx + (k * cos(yawlist[i])) - (j * sin(yawlist[i])))/XY_RESOLUTION;
				t = (cy + (k * sin(yawlist[i])) + (j * cos(yawlist[i])))/XY_RESOLUTION;

				// robot_collision_check_point.x = s * XY_RESOLUTION;
				// robot_collision_check_point.y = t * XY_RESOLUTION;
				// robot_collision_check_points.points.push_back(robot_collision_check_point);
				// robot_collision_check_pub.publish(robot_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Top Left
				s = (cx - (k * cos(yawlist[i])) - (j * sin(yawlist[i])))/XY_RESOLUTION;
				t = (cy - (k * sin(yawlist[i])) + (j * cos(yawlist[i])))/XY_RESOLUTION;

				// robot_collision_check_point.x = s * XY_RESOLUTION;
				// robot_collision_check_point.y = t * XY_RESOLUTION;
				// robot_collision_check_points.points.push_back(robot_collision_check_point);
				// robot_collision_check_pub.publish(robot_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Bottom Left
				s = (cx - (k * cos(yawlist[i])) + (j * sin(yawlist[i])))/XY_RESOLUTION;
				t = (cy - (k * sin(yawlist[i])) - (j * cos(yawlist[i])))/XY_RESOLUTION;

				// robot_collision_check_point.x = s * XY_RESOLUTION;
				// robot_collision_check_point.y = t * XY_RESOLUTION;
				// robot_collision_check_points.points.push_back(robot_collision_check_point);
				// robot_collision_check_pub.publish(robot_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Bottom Right
				s = (cx + (k * cos(yawlist[i])) + (j * sin(yawlist[i])))/XY_RESOLUTION;
				t = (cy + (k * sin(yawlist[i])) - (j * cos(yawlist[i])))/XY_RESOLUTION;

				// robot_collision_check_point.x = s * XY_RESOLUTION;
				// robot_collision_check_point.y = t * XY_RESOLUTION;
				// robot_collision_check_points.points.push_back(robot_collision_check_point);
				// robot_collision_check_pub.publish(robot_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}
			}
		}

		// ROS_INFO("Trailer Collision check for X : %f || Y : %f ", xlist[i], ylist[i]);

		// Computing trailer center from hitch position
		ctx = xlist[i] + deltat * cos(yawt[i]);
		cty = ylist[i] + deltat * sin(yawt[i]);

		// Visualize trailer center
		if(visualization) {
			geometry_msgs::PointStamped trailer_center;
			trailer_center.header.stamp = ros::Time::now();
			trailer_center.header.frame_id = "/map";
			trailer_center.point.x = ctx;
			trailer_center.point.y = cty;
			trailer_center_pub.publish(trailer_center);
			trailer_polygon_pub.publish(create_polygon(TL, TW, ctx, cty, yawt[i]));
		}

		max_x =  (ctx + TL * (cos(yawt[i]))/2 + TW * (sin(yawt[i]))/2) + MIN_SAFE_DIST;
		min_x =  (ctx - TL * (cos(yawt[i]))/2 - TW * (sin(yawt[i]))/2) - MIN_SAFE_DIST;

		max_y =  (cty + TL * (sin(yawt[i]))/2 + TW * (cos(yawt[i]))/2) + MIN_SAFE_DIST;
		min_y =  (cty - TL * (sin(yawt[i]))/2 - TW * (cos(yawt[i]))/2) - MIN_SAFE_DIST;
		
		if(max_x >= grid->info.width || min_x < 0 || max_y >= grid->info.height || min_y < 0) {
			// ROS_INFO("TRAILER OUT OF BOUNDS");
			return true; // OUT OF BOUNDS
		}

		// Checking the trailer polygon/rectangle
		for(float k = -TL/2 - MIN_SAFE_DIST; k <= TL/2 + MIN_SAFE_DIST; k += TL/4) {
			for(float j = -TW/2 - MIN_SAFE_DIST; j <= TW/2 + MIN_SAFE_DIST; j += TW/4) {

				// Top Right
				s = (ctx + (k * cos(yawt[i])) - (j * sin(yawt[i])))/XY_RESOLUTION;
				t = (cty + (k * sin(yawt[i])) + (j * cos(yawt[i])))/XY_RESOLUTION;

				// trailer_collision_check_point.x = s * XY_RESOLUTION;
				// trailer_collision_check_point.y = t * XY_RESOLUTION;
				// trailer_collision_check_points.points.push_back(trailer_collision_check_point);
				// trailer_collision_check_pub.publish(trailer_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Top Left
				s = (ctx - (k * cos(yawt[i])) - (j * sin(yawt[i])))/XY_RESOLUTION;
				t = (cty - (k * sin(yawt[i])) + (j * cos(yawt[i])))/XY_RESOLUTION;

				// trailer_collision_check_point.x = s * XY_RESOLUTION;
				// trailer_collision_check_point.y = t * XY_RESOLUTION;
				// trailer_collision_check_points.points.push_back(trailer_collision_check_point);
				// trailer_collision_check_pub.publish(trailer_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Bottom Left
				s = (ctx - (k * cos(yawt[i])) + (j * sin(yawt[i])))/XY_RESOLUTION;
				t = (cty - (k * sin(yawt[i])) - (j * cos(yawt[i])))/XY_RESOLUTION;

				// trailer_collision_check_point.x = s * XY_RESOLUTION;
				// trailer_collision_check_point.y = t * XY_RESOLUTION;
				// trailer_collision_check_points.points.push_back(trailer_collision_check_point);
				// trailer_collision_check_pub.publish(trailer_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}

				// Bottom Right
				s = (ctx + (k * cos(yawt[i])) + (j * sin(yawt[i])))/XY_RESOLUTION;
				t = (cty + (k * sin(yawt[i])) - (j * cos(yawt[i])))/XY_RESOLUTION;

				// trailer_collision_check_point.x = s * XY_RESOLUTION;
				// trailer_collision_check_point.y = t * XY_RESOLUTION;
				// trailer_collision_check_points.points.push_back(trailer_collision_check_point);
				// trailer_collision_check_pub.publish(trailer_collision_check_points);

				if(bin_map[s][t] != 0) {
					// cout << "X : " << s << " Y : "<< t << " BIN_MAP : " << bin_map[s][t] << endl;
					// robot_collision_check_pub.publish(robot_collision_check_points);
					// ROS_INFO("ROBOT POLYGON COLLISION");
					return true; // IN COLLISION
				}
			}
		}
	}

	// ROS_INFO("NO COLLISION - SAFE");
	return false; // NO COLLISION
}


/*
	Function to visualize robot/trailer polygons
	
	grid: Pointer to the occupancy grid msg
	bin_map: A 2D binary obstacle map

	Returns: geometry_msgs:PolygonStamped
*/
geometry_msgs::PolygonStamped create_polygon(float l, float w, float cx, float cy, float yaw) {

	geometry_msgs::PolygonStamped polygon;
	polygon.header.stamp = ros::Time::now();
	polygon.header.frame_id = "/map";
	polygon.polygon.points.clear();

	float length = l;
	float width = w;

	geometry_msgs::Point32 p;
	// Top Right
	p.x = cx + ((length/2) * cos(yaw)) - ((width/2) * sin(yaw));
	p.y = cy + ((length/2) * sin(yaw)) + ((width/2) * cos(yaw));
	polygon.polygon.points.push_back(p);

	// Top Left
	p.x = cx - ((length/2) * cos(yaw)) - ((width/2) * sin(yaw));
	p.y = cy - ((length/2) * sin(yaw)) + ((width/2) * cos(yaw));
	polygon.polygon.points.push_back(p);

	// Bottom Left
	p.x = cx - ((length/2) * cos(yaw)) + ((width/2) * sin(yaw));
	p.y = cy - ((length/2) * sin(yaw)) - ((width/2) * cos(yaw));
	polygon.polygon.points.push_back(p);

	// Bottom Right
	p.x = cx + ((length/2) * cos(yaw)) + ((width/2) * sin(yaw));
	p.y = cy + ((length/2) * sin(yaw)) - ((width/2) * cos(yaw));
	polygon.polygon.points.push_back(p);

	return polygon;
}