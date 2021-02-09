#! /usr/bin/env python

# Importing libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates, ModelStates
from tf import transformations

import math

# Defining Robot State
position = Point()
yaw = 0

# Defining State Machine
state = 0

# Defining Waypoint Co-ords
waypoint = Point()
waypoint.x = 9
waypoint.y = 2.5
waypoint.z = 0
final_yaw = 0

# Defining Precision
yaw_precision = 3 * (math.pi / 180)
dist_precision = 0.3

# Publisher to control the robot
pub = None

# Getting robot position from Odometry
def callback_odom(msg):
	global position
	global yaw

	position = msg.pose.pose.position

	quaternion = (
		msg.pose.pose.orientation.x, 
		msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z,
		msg.pose.pose.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]


# Getting true robot position from Gazebo
def callback_true_pos(msg):
	global position
	global yaw

	position = msg.pose[2].position

	quaternion = (
		msg.pose[2].orientation.x, 
		msg.pose[2].orientation.y,
		msg.pose[2].orientation.z,
		msg.pose[2].orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]


def callback_waypoint(msg):
	global waypoint
	global final_yaw

	waypoint = msg.pose.position
	# final_yaw = msg.pose.orientation.w


# Correcting the heading (yaw) error
def correct_yaw(waypoint, f_yaw=None):
	global yaw
	global yaw_precision
	global state
	global pub

	desired_yaw = math.atan2(waypoint.y - position.y, waypoint.x - position.x)
	yaw_error = desired_yaw - yaw

	twist_msg = Twist()
	if(math.fabs(yaw_error) > yaw_precision):
		twist_msg.angular.z = 0.7 if yaw_error > 0 else -0.7

	pub.publish(twist_msg)

	if(math.fabs(yaw_error) <= yaw_precision):
		print("Following path...")
		change_state(1)

# Traverse along the straight line to the waypoint
def follow_path(waypoint):
	global yaw
	global yaw_precision
	global state
	global pub

	desired_yaw = math.atan2(waypoint.y - position.y, waypoint.x - position.x)
	yaw_error = desired_yaw - yaw
	pos_error = math.sqrt(pow(waypoint.y - position.y, 2) + pow(waypoint.x - position.x, 2))

	if(pos_error > dist_precision):
		twist_msg = Twist()
		twist_msg.linear.x = 0.5
		pub.publish(twist_msg)
	else:
		print("Position Error: ", pos_error)
		change_state(2)

	if(math.fabs(yaw_error) > yaw_precision):
		print("Yaw Error: ", yaw_error)
		print("Correcting yaw...")
		change_state(0)

# Reached Waypoint
def reached():
	global final_yaw

	pos_error = math.sqrt(pow(waypoint.y - position.y, 2) + pow(waypoint.x - position.x, 2))
	if(pos_error < dist_precision):
		# correct_yaw(waypoint, final_yaw)
		print("Reacher Waypoint!")
		twist_msg = Twist()
		twist_msg.linear.x = 0
		twist_msg.angular.z = 0
		pub.publish(twist_msg)
	else:
		change_state(0)


def change_state(to_state):
	global state
	state = to_state
	print("State: ", state)

# Defining a function to normalize yaw
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


# Defining a function to normalize yaw
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def main():
	global pub

	rospy.init_node('nav_to_waypoint')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	
	sub_odom = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, callback_odom)
	# sub_true = rospy.Subscriber('/gazebo/model_states', ModelStates, callback_true_pos)
	sub_waypoint = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_waypoint)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if(state == 0):
			correct_yaw(waypoint)
		elif(state == 1):
			follow_path(waypoint)
		elif(state == 2):
			reached()
		else:
			rospy.logger('ERROR STATE!')
		rate.sleep()


if __name__ == '__main__':
	main()