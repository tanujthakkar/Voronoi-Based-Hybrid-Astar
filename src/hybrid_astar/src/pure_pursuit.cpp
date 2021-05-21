#include "../include/hybrid_astar/pure_pursuit.h"

using namespace std;


ros::Publisher cmd_pub;
ros::Publisher target_point_pub;

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


void callback_hitch(const sensor_msgs::JointState js) {
	alpha = js.position[0];
}


/*	
	Callback function for "global_path" to implement the pure pursuit controller
	
	Subscribes: global_path
	Publishers: target_point, cmd_vel
*/
void callback_path(const nav_msgs::Path p) {

	tf::TransformListener listener_robot;
	tf::StampedTransform transform_robot;
	listener_robot.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));

	tf::TransformListener listener_trailer;
	tf::StampedTransform transform_trailer;
	listener_trailer.waitForTransform("map", "cargo_cart_link", ros::Time::now(), ros::Duration(2.0));

	int index = 0;
	int n = p.poses.size();

	float lf;
	float tx; // target x
	float ty; // target y
	float alpha_;
	float delta;

	while(true) {
		
		listener_robot.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
		listener_trailer.lookupTransform("map", "cargo_cart_link", ros::Time(0), transform_trailer);

		// yaw = tf::getYaw(transform_robot.getRotation());
		// x = transform_robot.getOrigin().x();
		// y = transform_robot.getOrigin().y();

		yaw_r = tf::getYaw(transform_robot.getRotation());
		x_r = transform_robot.getOrigin().x();
		y_r = transform_robot.getOrigin().y();
		// ROS_INFO("x_r: %f y_r: %f yaw_r: %f", x_r, y_r, yaw_r);

		yaw = tf::getYaw(transform_trailer.getRotation());
		x = transform_trailer.getOrigin().x();
		y = transform_trailer.getOrigin().y();
		// ROS_INFO("x: %f y: %f yaw: %f", x, y, yaw);

		lf = kf * max_vel + ld;

		int m = 0;
		for (int i = index; i < n; ++i) {
			if(hypot(x - p.poses[i].pose.position.x, y - p.poses[i].pose.position.y) < m) {
				m = hypot(x - p.poses[i].pose.position.x, y - p.poses[i].pose.position.y);
				index = i;
			}

			if(hypot(x - p.poses[i].pose.position.x, y - p.poses[i].pose.position.y) > lf) {
				index = i;
				break;
			}
		}

		if(hypot(p.poses[n-1].pose.position.x - x_r, p.poses[n-1].pose.position.y - y_r) < 0.2) {
			ROS_INFO("GOAL REACHED!");
			break;
		}

		tx = p.poses[index].pose.position.x;
		ty = p.poses[index].pose.position.y;
		geometry_msgs::PointStamped target_point;
		target_point.header.stamp = ros::Time::now();
		target_point.header.frame_id = "/map";
		target_point.point.x = tx;
		target_point.point.y = ty;
		target_point_pub.publish(target_point);

		// alpha = atan2(ty - y, tx - x) - yaw;
		// delta = atan2(2.0 * WHEELBASE * sin(alpha), lf);

		alpha = abs(abs(yaw_r - yaw) - 3.14) - 3.14;
		if(yaw_r < 0 && yaw < 0) {
			if(yaw_r < yaw) {
				alpha = -alpha;
			}
		} else if(yaw_r > 0 && yaw > 0) {
			if(yaw_r < yaw) {
				alpha = -alpha;
			}
		}
		alpha_ = -atan2(2 * RTR * sin(atan2(ty - y, tx - x) - yaw), lf);
		delta = atan(WHEELBASE * ((alpha + alpha_) - (sin(alpha)/RTR)));
		printf("alpha: %f alpha_: %f delta: %f \n", alpha, alpha_, delta);

		geometry_msgs::Twist cmd;
		cmd.linear.x = min((float)max_vel, hypot(tx - x, ty - y));
		cmd.angular.z = delta;
		cmd_pub.publish(cmd);
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "pure_pursuit");

	ros::NodeHandle nh;
	hybrid_astar::GlobalPath p;


	cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	target_point_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 10);

	ros::Subscriber map_sub = nh.subscribe("global_path", 1, callback_path);
	ros::Subscriber hitch_sub = nh.subscribe("joint_states", 1, callback_hitch);

	ros::Rate rate(5);

	ros::spin();
	rate.sleep();

	return 0;
}