#! /usr/bin/env python

# Importing libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import *

import math


# Defining service variables to states
srv_client_nav_to_waypoint = None
srv_client_follow_wall = None

# Defining robot state
yaw = 0
yaw_precision = 3 * (math.pi / 180)

position = Point()
initial_pos = Point()
initial_pos.x = rospy.get_param('initial_x')
initial_pos.y = rospy.get_param('initial_y')
initial_pos.z = 0

waypoint = Point()
waypoint.x = rospy.get_param('way_x')
waypoint.y = rospy.get_param('way_y')
waypoint.z = 0

regions = None
state_description = ['Navigate to waypoint', 'Follow wall']
state = 0

count_state_time = 0
count_loop = 0

# Defining callback functions for subscribers

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


# Reading LIDAR data
def callback_laser(msg):
	global regions
	regions = {
		'left': min(min(msg.ranges[120:214]), 30),
		'front_left': min(min(msg.ranges[215:309]), 30),
		'front': min(min(msg.ranges[310:404]), 30),
		'front_right': min(min(msg.ranges[405:499]), 30),
		'right': min(min(msg.ranges[500:590]), 30)
	}
	# print(regions)


# Switching state machine states
def change_state(to_state):
	global state
	global state_description
	global srv_client_nav_to_waypoint
	global srv_client_follow_wall

	count_state_time = 0
	state = to_state
	rospy.loginfo("State changes : %s", state_description[state])
	print("State: ", state)

	if state == 0:
		print("Switching to navigating to waypoint...")
		response = srv_client_nav_to_waypoint(True)
		response = srv_client_follow_wall(False)
	if state == 1:
		print("Switching to wall following...")
		response = srv_client_nav_to_waypoint(False)
		response = srv_client_follow_wall(True)


# Defining function to calculate distance to path
def distance_to_line(p):
	global initial_pos
	global waypoint

	p1 = initial_pos
	p2 = waypoint

	up_eq = math.fabs((p2.y - p1.y) * p.x - (p2.x - p1.x) * p.y + (p2.x * p1.y) - (p2.y * p1.x))
	lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
	distance = up_eq / lo_eq
	# print("Distance [%.2f]", distance)

	return distance


# Defining a function to normalize yaw
def normalize_angle(angle):
	if(math.fabs(angle) > math.pi):
		angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
	return angle


def main():
	global regions
	global position
	global waypoint
	global state
	global yaw
	global yaw_precision
	global srv_client_nav_to_waypoint
	global srv_client_follow_wall
	global count_state_time
	global count_loop

	rospy.init_node('state_machine')

	# sub_odom = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, callback_odom)
	sub_laser = rospy.Subscriber('/scan', LaserScan, callback_laser)
	# sub_waypoint = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_waypoint)
	sub_true = rospy.Subscriber('/gazebo/link_states', LinkStates, callback_true_pos)

	rospy.wait_for_service('/nav_to_waypoint_switch')
	rospy.wait_for_service('/follow_wall_switch')
	# rospy.wait_for_service('/gazebo/set_model_state')

	srv_client_nav_to_waypoint = rospy.ServiceProxy('/nav_to_waypoint_switch', SetBool)
	srv_client_follow_wall = rospy.ServiceProxy('/follow_wall_switch', SetBool)
	# srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	
	# Initialize navigation to waypoint
	change_state(0)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		if regions == None:
			continue

		dist_to_line = distance_to_line(position)

		if(state == 0):
			if(regions['front'] < 0.5):
				change_state(1)
		elif(state == 1):
			if count_state_time > 5 and dist_to_line < 0.2:
				change_state(0)

		count_loop = count_loop + 1

		if count_loop == 20:
			count_state_time = count_state_time + 1
			count_loop = 0

		rospy.loginfo("Distance to path: [%.2f], Position: [%.2f, %.2f]", distance_to_line(position), position.x, position.y)
		rate.sleep()


if __name__ == '__main__':
	main()