#! /usr/bin/env python

# Importing libraries
import sys
from math import pi
import pandas as pd
from random import uniform
from datetime import datetime

import rospy
import rosbag
import hybrid_astar
from hybrid_astar.srv import MonteCarloSim
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from hybrid_astar.msg import Test, TestSummary

def pi_to_pi(yaw):
	while(yaw > pi):
		yaw -= 2.0 * pi;

	while(yaw < -pi):
		yaw += 2.0 * pi;

	return yaw


def monte_carlo_sim():

	rospy.wait_for_service("monte_carlo_sim_service")
	test_call = rospy.ServiceProxy("monte_carlo_sim_service", hybrid_astar.srv.MonteCarloSim)

	total_tests = 0
	valid_tests = 0
	successful_tests = 0
	unsuccessful_tests = 0
	tests = []
	redundant_tests = 0

	now = datetime.now()
	dt_string = now.strftime("%Y%m%d-%H%M%S")

	bag = rosbag.Bag('../rosbags/' + dt_string + '.bag', 'w')
	# bag = rosbag.Bag('../rosbags/20210409-234343.bag', 'a')

	try:
		while(valid_tests < 1000):
			total_tests = total_tests + 1
			# sx = 31.50
			# sy = 10.40
			# syaw = -1.57
			# syaw_t = -1.57
			# gx = 24.03
			# gy = 16.65
			# gyaw = -3.10
				
			# Indoor Environment
			sx = uniform(1.5 ,20.0)
			sy = uniform(2.0 ,20.0)
			syaw = uniform(-3.14, 3.14)
			syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
			gx = uniform(1.5 ,20.0)
			gy = uniform(2.0 ,20.0)
			gyaw = uniform(-3.14, 3.14)

			# Hospital 04
			# if(uniform(0.0, 1.0) >= 0.5):
			# 	sx = uniform(0.68 ,30.0)
			# 	sy = uniform(0.75 ,24.0)
			# else:
			# 	sx = uniform(30.7 ,42.25)
			# 	sy = uniform(7.60 ,17.60)
			# syaw = uniform(-3.14, 3.14)
			# syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
			
			# if(uniform(0.0, 1.0) >= 0.5):
			# 	gx = uniform(0.68 ,30.0)
			# 	gy = uniform(0.75 ,24.0)
			# else:
			# 	gx = uniform(30.7 ,42.25)
			# 	gy = uniform(7.60 ,17.60)
			# gyaw = uniform(-3.14, 3.14)

			if([sx, sy, syaw, syaw_t, gx, gy, gyaw] in tests):
				redundant_tests = redundant_tests + 1
				continue
			tests.append([sx, sy, syaw, syaw_t, gx, gy, gyaw])
			test_response = test_call(sx, sy, syaw, syaw_t, gx, gy, gyaw)

			if(test_response.valid_start and test_response.valid_goal):
				valid_tests = valid_tests + 1
				print("Test ", valid_tests)
				if(test_response.solution_found):
					successful_tests = successful_tests + 1
				else:
					unsuccessful_tests = unsuccessful_tests + 1
				print("Successful Tests: ", successful_tests)
				print("Redundant Tests: ", redundant_tests)
				print("Total Tests: ", total_tests)
				test = Test()
				test.sx = sx
				test.sy = sy
				test.syaw = syaw
				test.syaw_t = syaw_t
				test.gx = gx
				test.gy = gy
				test.gyaw = gyaw
				test.solution_found = test_response.solution_found
				test.iterations = test_response.iterations
				test.nodes = test_response.nodes
				test.execution_time = test_response.execution_time
				test.path = test_response.path
				bag.write('tests', test)
				# print("Valid Start ", test_response.valid_start)
				# print("Valid Goal ", test_response.valid_goal)
				# print("Solution Found ", test_response.solution_found)
				# print("Path ", test_response.path)
				# print("Iterations: ", test_response.iterations)
				# print("Nodes: ", test_response.nodes + 1)
				# print("Execution Time: ", test_response.execution_time)
	except (KeyboardInterrupt, rospy.service.ServiceException):
		test_summary = TestSummary()
		test_summary.successful_tests = successful_tests
		test_summary.unsuccessful_tests = unsuccessful_tests
		test_summary.redundant_tests = redundant_tests
		test_summary.total_tests = total_tests
		bag.write('test_summary', test_summary)

		bag.close()
		print("Successful Tests: ", successful_tests)
		print("Redundant Tests: ", redundant_tests)
		print("Total Tests: ", total_tests)

	test_summary = TestSummary()
	test_summary.successful_tests = successful_tests
	test_summary.unsuccessful_tests = unsuccessful_tests
	test_summary.redundant_tests = redundant_tests
	test_summary.total_tests = total_tests
	bag.write('test_summary', test_summary)

	bag.close()
	print("Successful Tests: ", successful_tests)
	print("Redundant Tests: ", redundant_tests)
	print("Total Tests: ", total_tests)


def review():

	rospy.init_node('test_review')

	start_pose_pub = rospy.Publisher('start_pose', PoseStamped, queue_size = 1)
	goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size = 1)
	path_pub = rospy.Publisher('global_path', Path, queue_size = 1)

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

	bag = rosbag.Bag('../rosbags/20210409-234343.bag')

	for topic, msg, t in bag.read_messages(topics=['tests']):
		print(msg)
		x = raw_input("Press Enter to continue...")
		start_pose.pose.position.x = msg.sx
		start_pose.pose.position.y = msg.sy
		quat = tf.transformations.quaternion_from_euler(0, 0, msg.syaw)
		start_pose.pose.orientation.x = quat[0]
		start_pose.pose.orientation.y = quat[1]
		start_pose.pose.orientation.z = quat[2]
		start_pose.pose.orientation.w = quat[3]
		start_pose_pub.publish(start_pose)

		goal_pose.pose.position.x = msg.gx
		goal_pose.pose.position.y = msg.gy
		quat = tf.transformations.quaternion_from_euler(0, 0, msg.gyaw)
		goal_pose.pose.orientation.x = quat[0]
		goal_pose.pose.orientation.y = quat[1]
		goal_pose.pose.orientation.z = quat[2]
		goal_pose.pose.orientation.w = quat[3]
		goal_pose_pub.publish(goal_pose)

		path_pub.publish(msg.path)

	rate.sleep()


if __name__ == '__main__':
	monte_carlo_sim()
	# review()