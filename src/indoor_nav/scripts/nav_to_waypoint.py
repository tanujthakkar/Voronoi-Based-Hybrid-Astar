#! /usr/bin/env python

# Importing libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from tf import transformations
from std_srvs.srv import *

import math

active = False

# Defining Robot State
position = Point()
yaw = 0

# Defining State Machine
state = 0

# Defining Waypoint Co-ords
waypoint = Point()
waypoint.x = rospy.get_param('way_x')
waypoint.y = rospy.get_param('way_y')
waypoint.z = 0

# Defining Precision
yaw_precision = 3 * (math.pi / 180)
dist_precision = 0.3

regions = None

# Publisher to control the robot
pub = None

# Defining a variable to store total yaw error for PID controller
total_yaw_error = 0


def nav_to_waypoint_switch(req):
	global active

	active = req.data
	response = SetBoolResponse()
	response.success = True
	response.message = "Nav to Waypoint Toggled"

	return response


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
	global yaw

	waypoint = msg.pose.position

	# quaternion = (
	# 	msg.pose.orientation.x, 
	# 	msg.pose.orientation.y,
	# 	msg.pose.orientation.z,
	# 	msg.pose.orientation.w)
	# euler = transformations.euler_from_quaternion(quaternion)
	# yaw = euler[2]


# Correcting the heading (yaw) error
def correct_yaw(waypoint):
	global yaw
	global yaw_precision
	global state
	global pub

	# p_gain = 0.5
	# i_gain = 0
	# d_gain = 0

	desired_yaw = math.atan2(waypoint.y - position.y, waypoint.x - position.x)
	yaw_error = normalize_angle(desired_yaw - yaw)

	rospy.loginfo("Yaw Error: [%.2f]", yaw_error)

	twist_msg = Twist()
	if(math.fabs(yaw_error) > yaw_precision):
		print("Correcting yaw...")
		# twist_msg.angular.z = (yaw_error*p_gain) if yaw_error > 0 else -(yaw_error*p_gain)
		twist_msg.angular.z = 0.7 if yaw_error > 0 else -0.7

	pub.publish(twist_msg)

	if(math.fabs(yaw_error) <= yaw_precision):
		print("Swtiching to path following...")
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
		print("Following path...")
		twist_msg = Twist()
		twist_msg.linear.x = 0.5
		pub.publish(twist_msg)
	else:
		print("Position Error: ", pos_error)
		change_state(2)

	if(math.fabs(yaw_error) > yaw_precision):
		print("Switching to Yaw correction")
		change_state(0)

	# if(regions['front'] < 0.5):
	# 	print("Switching to obstacle avoidance...")
	# 	change_state(3)


# Reached Waypoint
def reached():
	print("Reached Waypoint!")
	twist_msg = Twist()
	twist_msg.linear.x = 0
	twist_msg.angular.z = 0
	pub.publish(twist_msg)


# Obstacle Avoidance
def avoid_obstacle():
	global yaw
	global yaw_precision
	global state
	global pub
	global waypoint
	global position

	print("Avoiding obstacle...")
	desired_yaw = math.atan2(waypoint.y - position.y, waypoint.x - position.x)
	yaw_error = desired_yaw - yaw

	# temp_yaw_left = math.fabs(desired_yaw - (yaw + 1.54))
	# temp_yaw_right = math.fabs(desired_yaw - (yaw - 1.54))

	twist_msg = Twist()
	if((1.57 - yaw) < (1.57 + yaw) and regions['left'] > 0.2):
		print("Turning left...")
		twist_msg.angular.z = 0.5
		pub.publish(twist_msg)
	elif((1.57 - yaw) > (1.57 + yaw) and regions['right'] > 0.2):
		print("Turning right...")
		twist_msg.angular.z = -0.5
		pub.publish(twist_msg)

	if(regions['front'] > 0.5):
		# print("Swtiching to path following...")
		# change_state(1)
		print("Switching to yaw correction...")
		change_state(2)


# Change states
def change_state(to_state):
	global state
	state = to_state
	print("State: ", state)


# Defining a function to normalize yaw
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def main():
	global pub
	global active

	rospy.init_node('nav_to_waypoint')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
	
	# sub_odom = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, callback_odom)
	# sub_laser = rospy.Subscriber('/scan', LaserScan, callback_laser)
	# sub_waypoint = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_waypoint)
	sub_true = rospy.Subscriber('/gazebo/link_states', LinkStates, callback_true_pos)
		
	srv = rospy.Service('nav_to_waypoint_switch', SetBool, nav_to_waypoint_switch)

	rate = rospy.Rate(20)

	# Main loop which runs the state machine
	while not rospy.is_shutdown():

		if not active:
			continue
		else:
			if state == 0:
				correct_yaw(waypoint)
			elif state == 1:
				follow_path(waypoint)
			elif state == 2:
				reached()
			else:
				rospy.logerr("ERROR STATE!")

		rate.sleep()

if __name__ == '__main__':
	main()