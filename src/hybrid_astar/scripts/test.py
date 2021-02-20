#! /usr/bin/env python

# Importing libraries
import rospy
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf import transformations

import math

# Global Variables
start_pose = None

# Publishers
pub_start_pose = None

# Subscribers
sub_start_pose = None
sub_goal_pose = None
sub_map = None

def callback_start_pose(pose):
	global start_pose
	global pub_start_pose

	start_pose = pose
	pub_start_pose.publish(start_pose)

def callback_map(map):
	print(map)

def main():
	global pub_start_pose

	rospy.init_node('test')

	pub_start_pose = rospy.Publisher('start_pose', PoseWithCovarianceStamped, queue_size=1)
	sub_start_pose = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, callback_start_pose)
	sub_map = rospy.Subscriber('map', OccupancyGrid, callback_map)
	rospy.spin()

if __name__ == '__main__':
	main()