#! /usr/bin/env python

# Importing libraries
import sys
import signal
from math import pi
import pandas as pd
from random import uniform
from datetime import datetime

import rospy
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def main():

	rospy.init_node('test_review')

	start_pose_pub = rospy.Publisher('start_pose', PoseStamped, queue_size = 1)
	goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size = 1)

	rate = rospy.Rate(10)

	start_pose = PoseStamped()
	goal_pose = PoseStamped()
	path = Path()
	pose_stamped = PoseStamped()

	start_pose.header.stamp = rospy.Time.now()
	start_pose.header.frame_id = "/map"
	goal_pose.header.stamp = rospy.Time.now()
	goal_pose.header.frame_id = "/map"
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = "/map"
		
	df = pd.read_csv('20210329-234631.csv', usecols = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Path'])

	row = df.loc[0]
	for i in row[8]:
		print(i)

	# for x in range(len(row[8])):
	# 	pose_stamped.header.stamp = rospy.Time.now()
	# 	pose_stamped.header.frame_id = "/map"
	# 	pose_stamped.pose.position.x = row[8]

	# for i in range(1):
	# 	try:
	# 		row = df.loc[i]
	# 		if(1):
	# 			raw_input("Press Enter to continue...")
	# 			print(row)
	# 			start_pose.pose.position.x = row[0]
	# 			start_pose.pose.position.y = row[1]
	# 			quat = tf.transformations.quaternion_from_euler(0, 0, row[2])
	# 			start_pose.pose.orientation.x = quat[0]
	# 			start_pose.pose.orientation.y = quat[1]
	# 			start_pose.pose.orientation.z = quat[2]
	# 			start_pose.pose.orientation.w = quat[3]
	# 			start_pose_pub.publish(start_pose)

	# 			goal_pose.pose.position.x = row[4]
	# 			goal_pose.pose.position.y = row[5]
	# 			quat = tf.transformations.quaternion_from_euler(0, 0, row[6])
	# 			goal_pose.pose.orientation.x = quat[0]
	# 			goal_pose.pose.orientation.y = quat[1]
	# 			goal_pose.pose.orientation.z = quat[2]
	# 			goal_pose.pose.orientation.w = quat[3]
	# 			goal_pose_pub.publish(goal_pose)

	# 			for x in range(len(row[8][0])):
	# 				pass
	# 	except KeyboardInterrupt:
	# 		print("Exiting...")
	# 		sys.exit()

	rate.sleep()


if __name__ == '__main__':
	main()