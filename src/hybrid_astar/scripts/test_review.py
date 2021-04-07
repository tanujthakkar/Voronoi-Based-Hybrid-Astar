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
		
	df = pd.read_csv('20210329-234631.csv', usecols = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time'])
	df_r = pd.DataFrame([], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time'])
	df_r.to_csv('20210329-234631_review.csv', mode = 'a')

	for i in range(len(df)):
		row = df.loc[i]
		if(not row[7]):
			print(row)
			start_pose.pose.position.x = row[0]
			start_pose.pose.position.y = row[1]
			quat = tf.transformations.quaternion_from_euler(0, 0, row[2])
			start_pose.pose.orientation.x = quat[0]
			start_pose.pose.orientation.y = quat[1]
			start_pose.pose.orientation.z = quat[2]
			start_pose.pose.orientation.w = quat[3]
			start_pose_pub.publish(start_pose)
			
			goal_pose.pose.position.x = row[4]
			goal_pose.pose.position.y = row[5]
			quat = tf.transformations.quaternion_from_euler(0, 0, row[6])
			goal_pose.pose.orientation.x = quat[0]
			goal_pose.pose.orientation.y = quat[1]
			goal_pose.pose.orientation.z = quat[2]
			goal_pose.pose.orientation.w = quat[3]
			goal_pose_pub.publish(goal_pose)
			x = raw_input("Press Enter to continue...")
			if(x == 'y'):
				df_r = pd.DataFrame([[row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8], row[9], row[10]]], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time'])
				df_r.to_csv('20210329-234631_review.csv', mode = 'a', header = False)
		else:
			df_r = pd.DataFrame([[row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8], row[9], row[10]]], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time'])
			df_r.to_csv('20210329-234631_review.csv', mode = 'a', header = False)

	rate.sleep()


if __name__ == '__main__':
	main()