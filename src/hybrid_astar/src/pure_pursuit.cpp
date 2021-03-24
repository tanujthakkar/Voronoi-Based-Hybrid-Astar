#include "../include/hybrid_astar/pure_pursuit.h"

using namespace std;


ros::Publisher cmd_pub;
ros::Publisher target_point_pub;

// Current Pose of the robot
float x;
float y;
float yaw;

// PID Constants
const float Kp = 0.3; // Proportional constant

// Pure Pursuit Constants
const float ld = 0.3; // look-ahead distance
const float kf = 0.1; // look forward gain
const float max_vel = 0.3; // maximum velocity
const float max_ang_vel = 0.3; // maximum angular


/*	
	Callback function for "global_path" to implement the pure pursuit controller
	
	Subscribes: global_path
	Publishers: target_point, cmd_vel
*/
void callback_path(const nav_msgs::Path p) {

	tf::TransformListener listener;
	tf::StampedTransform transform_robot;
	listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));

	int index = 0;
	int n = p.poses.size();

	float lf;
	float tx; // target x
	float ty; // target y
	float alpha;
	float delta;

	while(true) {
		
		listener.lookupTransform("map", "base_link", ros::Time(0), transform_robot);
		yaw = tf::getYaw(transform_robot.getRotation());
		x = transform_robot.getOrigin().x();
		y = transform_robot.getOrigin().y();
		
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

		if(hypot(p.poses[n-1].pose.position.x - x, p.poses[n-1].pose.position.y - y) < 0.2) {
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

		alpha = atan2(ty - y, tx - x) - yaw;
    	delta = atan2(2.0 * WHEELBASE * sin(alpha), lf);

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

	ros::Rate rate(10);

	ros::spin();
	rate.sleep();

	return 0;
}