#include "../include/hybrid_astar/pure_pursuit.h"

using namespace std;

// Current Pose of the robot
float x;
float y;
float yaw;

float x_r;
float y_r;
float yaw_r;

float alpha;

// PID Constants
const float Kp = 0.3; // Proportional constant

// Pure Pursuit Constants
const float ld = 0.3; // look-ahead distance
const float kf = 0.1; // look forward gain
const float max_vel = 0.3; // maximum velocity
const float max_ang_vel = 0.3; // maximum angular

float k = 2.0;


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

	int index = 0;
	int n = path.poses.size();

	float lf;
	float tx; // target x
	float ty; // target y
	float alpha;
	float alpha_;
	float delta;

	while(true) {
		
		listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
		listener_trailer.lookupTransform("map", "cargo_cart_link", ros::Time(0), transform_trailer);

		yaw = tf::getYaw(transform_robot.getRotation());
		x = transform_robot.getOrigin().x();
		y = transform_robot.getOrigin().y();

		// yaw_r = tf::getYaw(transform_robot.getRotation());
		// x_r = transform_robot.getOrigin().x();
		// y_r = transform_robot.getOrigin().y();
		// // ROS_INFO("x_r: %f y_r: %f yaw_r: %f", x_r, y_r, yaw_r);

		// yaw = tf::getYaw(transform_trailer.getRotation());
		// x = transform_trailer.getOrigin().x();
		// y = transform_trailer.getOrigin().y();
		// ROS_INFO("x: %f y: %f yaw: %f", x, y, yaw);

		lf = kf * max_vel + ld;

		int m = 0;
		for (int i = index; i < n; ++i) {
			if(hypot(x - path.poses[i].pose.position.x, y - path.poses[i].pose.position.y) < m) {
				m = hypot(x - path.poses[i].pose.position.x, y - path.poses[i].pose.position.y);
				index = i;
			}

			if(hypot(x - path.poses[i].pose.position.x, y - path.poses[i].pose.position.y) > lf) {
				index = i;
				break;
			}
		}

		if(hypot(x - path.poses[n-1].pose.position.x, y - path.poses[n-1].pose.position.y) < 0.2) {
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

		alpha = atan2(ty - y, tx - x) - yaw;
		delta = atan2(2.0 * WHEELBASE * sin(alpha), lf);

		// alpha=atan2((yd(i)-y2),(xd(i)-x2))-beta2+pi
		// delta_des=atan(l*(K*sign(v1)*(-phi+atan(2*l*sin(alpha)/lookahead_dist))+sin(phi)/e));

		// alpha = pi_2_pi(yaw_r - yaw);
		// alpha_ = atan2(ty - y, tx - x) - yaw;
		// delta = atan(WHEELBASE * (k * 1.0) * ((alpha - atan2(2 * WHEELBASE * sin(alpha_), lf)) - (sin(alpha)/RTR)));

		geometry_msgs::Twist cmd;
		cmd.linear.x = min((float)max_vel, hypot(tx - x, ty - y));
		cmd.angular.z = delta;
		printf("alpha: %f alpha_: %f delta: %f linear: %f\n", alpha, alpha_, delta, cmd.linear.x);
		cmd_pub.publish(cmd);
	}
}


// int main(int argc, char **argv) {

// 	ros::init(argc, argv, "pure_pursuit");

// 	ros::NodeHandle nh;
// 	hybrid_astar::GlobalPath path;


// 	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
// 	target_point_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 10);

// 	ros::Subscriber map_sub = nh.subscribe("global_path", 1, callback_path);

// 	ros::Rate rate(5);

// 	ros::spin();
// 	rate.sleep();

// 	return 0;
// }