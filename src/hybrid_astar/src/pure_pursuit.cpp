#include "../include/hybrid_astar/pure_pursuit.h"

using namespace std;

// Current pose of the robot
float x_r;
float y_r;
float yaw_r;

// Current pose of the trailer
float x_t;
float y_t;
float yaw_t;

float alpha;
float phi;

// PID Constants
const float Kp = 0.3; // Proportional constant

// Pure Pursuit Constants
const float ld = 0.3; // look-ahead distance
const float kf = 0.1; // look forward gain
const float max_vel = 0.3; // maximum velocity
const float max_ang_vel = 0.3; // maximum angular

float k = 16.0;


/*	
	Callback function for "global_path" to implement the pure pursuit controller
	
	Subscribes: global_path
	Publishers: target_point, cmd_vel
*/
void pure_pursuit() {

	tf::TransformListener listener_robot;
	tf::StampedTransform transform_robot;
	listener_robot.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));

	tf::TransformListener listener_trailer;
	tf::StampedTransform transform_trailer;
	listener_trailer.waitForTransform("map", "cargo_cart_link", ros::Time::now(), ros::Duration(2.0));

	listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
	geometry_msgs::PoseStamped pose_stamped;
	float x;
	float y;
	float yaw;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "/map";
	path.poses.clear();

	for(float i = 0.0; i < 30; i = i + 0.1) {
		yaw = tf::getYaw(transform_robot.getRotation());
		x = transform_robot.getOrigin().x();
		y = transform_robot.getOrigin().y();

		pose_stamped.header.stamp = ros::Time::now();
		pose_stamped.header.frame_id = "map";
		pose_stamped.pose.position.x = x - i;
		pose_stamped.pose.position.y = y;
		tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
		pose_stamped.pose.orientation.x = quat.x();
		pose_stamped.pose.orientation.y = quat.y();
		pose_stamped.pose.orientation.z = quat.z();
		pose_stamped.pose.orientation.w = quat.w();
		path.poses.push_back(pose_stamped);
		dirs.push_back(-1);
	}
	global_path_pub.publish(path);

	int index = 0;
	int n = path.poses.size();
	printf("path length: %d", n);

	float lf;
	float tx; // target x
	float ty; // target y
	float alpha;
	float alpha_;
	float delta;

	while(true) {
		
		listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
		listener_trailer.lookupTransform("map", "cargo_cart_link", ros::Time(0), transform_trailer);

		yaw_r = tf::getYaw(transform_robot.getRotation());
		x_r = transform_robot.getOrigin().x();
		y_r = transform_robot.getOrigin().y();
		// ROS_INFO("x_r: %f y_r: %f yaw_r: %f", x_r, y_r, yaw_r);

		yaw_t = tf::getYaw(transform_trailer.getRotation());
		x_t = transform_robot.getOrigin().x() - ((DELTAR + 0.0) - DELTAT) * cos(yaw_t);
		y_t = transform_robot.getOrigin().y() - ((DELTAR + 0.0) - DELTAT) * sin(yaw_t);
		// ROS_INFO("x_t: %f y_t: %f yaw_t: %f", x_t, y_t, yaw_t);

		lf = kf * max_vel + ld;

		// ROS_INFO("DIR: %d", dirs[index]);
		if(dirs[index] == 1) {
			float m = 1000.0;
			// printf("index: %d\n", index);
			for (int i = index; i < n; ++i) {
				// printf("m: %f point: %f\n", m, hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y));
				if(hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y) < m) {
					m = hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y);
					index = i;
				}

				if(hypot(x_r - path.poses[i].pose.position.x, y_r - path.poses[i].pose.position.y) > lf) {
					index = i;
					break;
				}
			}

			if(hypot(x_r - path.poses[n-1].pose.position.x, y_r - path.poses[n-1].pose.position.y) < 0.2) {
				ROS_INFO("GOAL REACHED!");
				break;
			}

			tx = path.poses[index].pose.position.x;
			ty = path.poses[index].pose.position.y;
			geometry_msgs::PointStamped target_point;
			target_point.header.stamp = ros::Time::now();
			target_point.header.frame_id = "/map";
			target_point.point.x = tx;
			target_point.point.y = ty;
			target_point_pub.publish(target_point);

			alpha = atan2(ty - y_r, tx - x_r) - yaw_r;
			delta = atan2(2.0 * WHEELBASE * sin(alpha), lf);

			geometry_msgs::Twist cmd;
			cmd.linear.x = min((float)max_vel, hypot(tx - x_r, ty - y_r));
			cmd.angular.z = delta;
			// printf("alpha: %f alpha_: %f delta: %f linear: %f\n", alpha, alpha_, delta, cmd.linear.x);
			cmd_pub.publish(cmd);
		}

		if(dirs[index] == -1) {
			float m = 1000.0;
			// printf("index: %d\n", index);
			for (int i = index; i < n; ++i) {
				// printf("m: %f point: %f\n", m, hypot(x_t - path.poses[i].pose.position.x, y_t - path.poses[i].pose.position.y));
				if(hypot(x_t - path.poses[i].pose.position.x, x_t - path.poses[i].pose.position.y) < m) {
					m = hypot(x_t - path.poses[i].pose.position.x, x_t - path.poses[i].pose.position.y);
					index = i;
				}

				if(hypot(x_t - path.poses[i].pose.position.x, x_t - path.poses[i].pose.position.y) > lf) {
					index = i;
					break;
				}
			}

			if(hypot(x_t - path.poses[n-1].pose.position.x, x_t - path.poses[n-1].pose.position.y) < 0.2) {
				ROS_INFO("GOAL REACHED!");
				break;
			}

			tx = path.poses[index].pose.position.x;
			ty = path.poses[index].pose.position.y;
			geometry_msgs::PointStamped target_point;
			target_point.header.stamp = ros::Time::now();
			target_point.header.frame_id = "/map";
			target_point.point.x = tx;
			target_point.point.y = ty;
			target_point_pub.publish(target_point);

			// alpha=atan2((yd(i)-y2),(xd(i)-x2))-beta2
			// delta_des=atan(d*(K*sign(v1)*(phi+atan(2*d*sin(alpha)/lookahead_dist))-sin(phi)/e));

			phi = pi_2_pi(yaw_r - yaw_t);
			alpha = pi_2_pi(atan2(ty - y_t, tx - x_t) - yaw_t);
			delta = atan(WHEELBASE * (k * -1.0 * (phi + atan(2 * RTR * sin(alpha)/lf)) - sin(phi)/RTR));

			geometry_msgs::Twist cmd;
			cmd.linear.x = -0.2;
			cmd.angular.z = delta;
			cmd_pub.publish(cmd);
			// printf("phi: %f alpha: %f delta: %f linear: %f\n", phi, alpha, delta, cmd.linear.x);
		}
		// cin.get();
	}
}