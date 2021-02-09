#! /usr/bin/env python

# Importing required libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active = False

# Defining a publisher to control the robot
pub = None

# Defining a dictionary to store scan data
regions = {
		'left': 0,
		'front_left': 0,
		'front': 0,
		'front_right': 0,
		'right': 0,
	}

# Defining state machine variable
state = 0


def follow_wall_swtich(req):
	global active

	active = req.data
	respone = SetBoolResponse()
	respone.success = True
	respone.message = "Follow Wall Toggled!"

	return respone

# Reading LIDAR data
def callback_laser(msg):
	global regions
	regions = {
		'left': min(min(msg.ranges[120:214]), 30),
		'front_left': min(min(msg.ranges[215:309]), 30),
		'front': min(min(msg.ranges[310:404]), 30),
		'front_right': min(min(msg.ranges[405:499]), 30),
		'right': min(min(msg.ranges[500:590]), 30),
	}

	follow_wall()


def follow_wall():
	global regions

	msg = Twist()
	linear_x = 0
	angular_z = 0
	state_description = ''

	f = 1.5
	t = 1.5

	if regions['front'] > f and regions['front_left'] > t and regions['front_right'] > t:
		state_description = 'case 1 - nothing'
		change_state(0, state_description)
	elif regions['front'] < f and regions['front_left'] > t and regions['front_right'] > t:
		state_description = 'case 2 - front'
		change_state(1, state_description)
	elif regions['front'] > f and regions['front_left'] > t and regions['front_right'] < t:
		state_description = 'case 3 - front_right'
		change_state(2, state_description)
	elif regions['front'] > f and regions['front_left'] < t and regions['front_right'] > t:
		state_description = 'case 4 - front_left'
		change_state(0, state_description)
	elif regions['front'] < f and regions['front_left'] > t and regions['front_right'] < t:
		state_description = 'case 5 - front and front_right'
		change_state(1, state_description)
	elif regions['front'] < f and regions['front_left'] < t and regions['front_right'] > t:
		state_description = 'case 6 - front and front_left'
		change_state(1, state_description)
	elif regions['front'] < f and regions['front_left'] < t and regions['front_right'] < t:
		state_description = 'case 7 - front and front_left and front_right'
		change_state(1, state_description)
	elif regions['front'] > f and regions['front_left'] < t and regions['front_right'] < t:
		state_description = 'case 8 - front_left and front_right'
		change_state(0, state_description)
	# elif regions[right] < t:
	# 	state_description = 'case 9 - front_right and right'
	# 	change_state(1, state_description)
	else:
		state_description = 'unknown case'
		rospy.loginfo(regions)


def find_wall():
	print("Finding wall...")
	msg = Twist()
	msg.linear.x = 0.3
	msg.angular.z = -0.3

	return msg


def turn_left():
	print("Turning left...")
	msg = Twist()
	msg.angular.z = 0.3

	return msg


def turn_right():
	print("Turning right...")
	msg = Twist()
	msg.angular.z = -0.3

	return msg


def go_straight():
	print("Going straight...")
	msg = Twist()
	msg.linear.x = 0.5

	return msg


def change_state(s, d):
	global state

	if s is not state:
		print("Changing state to : ", state, d)
		state = s


def main():
	global pub, active

	rospy.init_node('follow_wall')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
	sub = rospy.Subscriber('/scan', LaserScan, callback_laser)

	srv = rospy.Service('follow_wall_switch', SetBool, follow_wall_swtich)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		
		if not active:
			rate.sleep()
			continue

		msg = Twist()

		if state == 0:
			msg = find_wall()
		elif state == 1:
			msg = turn_left()
		# elif state == 3:
		# 	msg = turn_right()
		elif state == 2:
			msg = go_straight()
			pass
		else:
			rospy.logerr('ERROR STATE!')

		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	main()