#! /usr/bin/env python

# Importing libraries
import sys
from math import pi, sin, cos, tan, hypot
import pandas as pd
from random import uniform
from datetime import datetime
from time import sleep
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
import statistics

import rospy
import rosbag
import hybrid_astar
from hybrid_astar.srv import MonteCarloSim
import tf
from tf import transformations
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, PointStamped
from nav_msgs.msg import Path
from hybrid_astar.msg import Test, TestSummary

RW = 0.793
RL = 0.960
RF = 0.799
RB = 0.161

TW = 0.643
TL = 1.0
RTF = 0.025
RTB = 1.0

visualize_robot = True

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
	iteration_limits = 0
	tests = []

	now = datetime.now()
	dt_string = now.strftime("%Y%m%d-%H%M%S")

	bag = rosbag.Bag('../rosbags/' + dt_string + '.bag', 'w')
	# bag = rosbag.Bag('../rosbags/hybrid_astar.bag', 'a')
	iteration_limits_bag = rosbag.Bag('../rosbags/iteration_limits.bag', 'a')

	try:
		while(valid_tests < 1):
		# for sx, sy, syaw, syaw_t, gx, gy, gyaw in tests:

			total_tests = total_tests + 1

			# Hospital_04
			# if(uniform(0.0, 1.0) >= 0.5):
			# 	sx = uniform(4.13, 33.75)
			# 	sy = uniform(13.13, 36.80)
			# else:
			# 	sx = uniform(34.15, 45.75)
			# 	sy = uniform(19.92, 30.02)
			# sx = uniform(4.13, 45.71)
			# sy = uniform(13.13, 36.80)
			# syaw = uniform(-3.14, 3.14)
			# syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))

			# if(uniform(0.0, 1.0) >= 0.5):
			# 	gx = uniform(4.13, 33.75)
			# 	gy = uniform(13.13, 36.80)
			# else:
			# 	gx = uniform(34.15, 45.75)
			# 	gy = uniform(19.92, 30.02)
			# gx = uniform(4.13, 45.71)
			# gy = uniform(13.13, 36.80)
			# gyaw = uniform(-3.14, 3.14)
			# gyaw_t = uniform(min(pi_to_pi(gyaw - 1.395), pi_to_pi(gyaw + 1.395)), max(pi_to_pi(gyaw - 1.395), pi_to_pi(gyaw + 1.395)))
			# gyaw_t = gyaw

			syaw = 1.584515
			syaw_t = 1.584515
			sx = 5.350000
			sy = 14.750000

			gyaw = 2.365891
			gyaw_t = 2.365891
			gx = 31.450001
			gy = 35.099998

			test_response = test_call(sx, sy, syaw, syaw_t, gx, gy, gyaw, gyaw_t)

			if(test_response.valid_start and test_response.valid_goal):
				if(test_response.iteration_limit):
					iteration_limits = iteration_limits + 1
					test = Test()
					test.sx = sx
					test.sy = sy
					test.syaw = syaw
					test.syaw_t = syaw_t
					test.gx = gx
					test.gy = gy
					test.gyaw = gyaw
					test.gyaw_t = gyaw_t
					iteration_limits_bag.write('tests', test)
					continue

				valid_tests = valid_tests + 1
				print("Test ", valid_tests)
				print([sx, sy, syaw, syaw_t, gx, gy, gyaw, gyaw_t])
				if(test_response.solution_found):
					successful_tests = successful_tests + 1
				else:
					unsuccessful_tests = unsuccessful_tests + 1
				print("Successful Tests: ", successful_tests)
				print("Unsuccessful Tests: ", unsuccessful_tests)
				print("Iterations Limits: ", iteration_limits)
				print("Total Tests: ", total_tests)
				test = Test()
				test.sx = sx
				test.sy = sy
				test.syaw = syaw
				test.syaw_t = syaw_t
				test.gx = gx
				test.gy = gy
				test.gyaw = gyaw
				test.gyaw_t = gyaw_t
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
	# except(KeyboardInterrupt, rospy.service.ServiceException):
	except:
		test_summary = TestSummary()
		test_summary.successful_tests = successful_tests
		test_summary.unsuccessful_tests = unsuccessful_tests
		test_summary.iteration_limits = iteration_limits
		test_summary.total_tests = total_tests
		bag.write('test_summary', test_summary)

		print("Successful Tests: ", successful_tests)
		print("Unsuccessful Tests: ", unsuccessful_tests)
		print("Iterations Limits: ", iteration_limits)
		print("Total Tests: ", total_tests)
		bag.close()
		iteration_limits_bag.close()
		sys.exit("EXITING")


	test_summary = TestSummary()
	test_summary.successful_tests = successful_tests
	test_summary.unsuccessful_tests = unsuccessful_tests
	test_summary.iteration_limits = iteration_limits
	test_summary.total_tests = total_tests
	bag.write('test_summary', test_summary)

	print("Successful Tests: ", successful_tests)
	print("Unsuccessful Tests: ", unsuccessful_tests)
	print("Iterations Limits: ", iteration_limits)
	print("Total Tests: ", total_tests)
	bag.close()
	iteration_limits_bag.close()


def review():

	rospy.init_node('test_review')

	rospy.wait_for_service("monte_carlo_sim_service")
	test_call = rospy.ServiceProxy("monte_carlo_sim_service", hybrid_astar.srv.MonteCarloSim)

	start_pose_pub = rospy.Publisher('start_pose', PoseStamped, queue_size = 1)
	goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size = 1)
	
	hybrid_astar_path_pub = rospy.Publisher('hybrid_astar_path', Path, queue_size = 1)
	hybrid_astar_voronoi_path_pub = rospy.Publisher('hybrid_astar_voronoi_path', Path, queue_size = 1)

	rate = rospy.Rate(10)

	start_pose = PoseStamped()
	goal_pose = PoseStamped()

	start_pose.header.stamp = rospy.Time.now()
	start_pose.header.frame_id = "/map"
	goal_pose.header.stamp = rospy.Time.now()
	goal_pose.header.frame_id = "/map"

	hybrid_astar_path = Path()
	hybrid_astar_path.header.stamp = rospy.Time.now()
	hybrid_astar_path.header.frame_id = "/map"

	hybrid_astar_voronoi_path = Path()
	hybrid_astar_voronoi_path.header.stamp = rospy.Time.now()
	hybrid_astar_voronoi_path.header.frame_id = "/map"

	bag = read_bag = rosbag.Bag('../rosbags/hybrid_astar/hybrid_astar.bag')
	hybrid_astar_paths = []

	read_bag = rosbag.Bag('../rosbags/dubins_hybrid_astar_voronoi/dubins_hybrid_astar_voronoi.bag')
	hybrid_astar_voronoi_paths = []

	for topic, msg, t in bag.read_messages(topics=['tests']):
		hybrid_astar_paths.append(msg.path)

	for topic, msg, t in read_bag.read_messages(topics=['tests']):
		hybrid_astar_voronoi_paths.append(msg.path)

	i = 0
	for topic, msg, t in read_bag.read_messages(topics=['tests']):
		print(i)
		hybrid_astar_path_pub.publish(hybrid_astar_paths[i])
		hybrid_astar_voronoi_path_pub.publish(hybrid_astar_voronoi_paths[i])
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

		test_response = test_call(msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw, 0.0)
		i = i + 1
		x = raw_input("Press Enter to visualize...")
		if(x == 'q'):
			break
		if(x == 'y'):
			raw_input("Press Enter to continue...")


def graphs():

	dubins_bag = rosbag.Bag('../rosbags/dubins_hybrid_astar_voronoi/dubins_hybrid_astar_voronoi.bag')
	hybrid_astar_bag = rosbag.Bag('../rosbags/hybrid_astar/hybrid_astar.bag')

	iterations = []
	nodes = []
	lengths = []
	execution_times = []

	dubins_solution_score = 0
	hybrid_astar_solution_score = 0
	dubins_iteration_score = 0
	hybrid_astar_iteration_score = 0
	dubins_node_score = 0
	hybrid_astar_node_score = 0
	dubins_execution_time_score = 0
	hybrid_astar_execution_time_score = 0
	dubins_length_score = 0
	hybrid_astar_length_score = 0

	i = 0
	for topic, msg, t in dubins_bag.read_messages(topics=['tests']):
		i = i + 1
		if(msg.solution_found):
			# print("Test", i)
			# print([msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw, msg.solution_found, msg.iterations, msg.nodes, msg.execution_time])
			iterations.append(msg.iterations)
			nodes.append(msg.nodes)
			lengths.append(msg.path_length)
			execution_times.append(msg.execution_time)
	print(i)
	dubins_solution_score = i

	i = 0
	for topic, msg, t in hybrid_astar_bag.read_messages(topics=['tests']):
		
		if(msg.solution_found):
			hybrid_astar_solution_score = hybrid_astar_solution_score + 1
		
		print(iterations[i], msg.iterations)
		if(iterations[i] <= msg.iterations):
			dubins_iteration_score = dubins_iteration_score + 1
		else:
			hybrid_astar_iteration_score = hybrid_astar_iteration_score + 1

		print(nodes[i], msg.nodes)
		if(nodes[i] <= msg.nodes):
			dubins_node_score = dubins_node_score + 1
		else:
			hybrid_astar_node_score = hybrid_astar_node_score + 1

		print(execution_times[i], msg.execution_time)
		if(execution_times[i] <= msg.execution_time):
			dubins_execution_time_score = dubins_execution_time_score + 1
		else:
			hybrid_astar_execution_time_score = hybrid_astar_execution_time_score + 1

		print(lengths[i], msg.path_length)
		if(lengths[i] <= msg.path_length):
			dubins_length_score = dubins_length_score + 1
		else:
			hybrid_astar_length_score = hybrid_astar_length_score + 1

		# raw_input()
		print("Solution", dubins_solution_score, hybrid_astar_solution_score)
		print("Iterations", dubins_iteration_score, hybrid_astar_iteration_score)
		print("Nodes", dubins_node_score, hybrid_astar_node_score)
		print("Execution Time", dubins_execution_time_score, hybrid_astar_execution_time_score)
		print("Path Length", dubins_length_score, hybrid_astar_length_score)
		i = i + 1
	# # nodes = list(map(float, nodes))

	# plt.subplot(1,1,1)
	# # counts, edges, plot = plt.hist(np.array(iterations), bins=[0, 25000, 50000, 75000, 100000, 125000, 150000, 175000, 200000, 225000, 250000, 300000, 350000], alpha=0.5, color='r', label='Iterations')
	# counts, edges, plot = plt.hist(np.array(iterations), bins=[0, statistics.median(iterations)], alpha=0.5, color='r', label='Iterations')
	# plt.xlabel('Iterations')
	# plt.ylabel('Count')
	# print("Iterations")
	# print("Counts", counts)
	# print("Total Counts", sum(counts))
	# print("Median", statistics.median(iterations))
	# print("Edges" , edges)

	# plt.subplot(1,1,1)
	# # counts, edges, plot = plt.hist(np.array(iterations), bins=[0, 0.5, 1.5, 2, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5], alpha=0.5, color='r', label='Iterations')
	# counts, edges, plot = plt.hist(np.array(nodes), bins=[0, statistics.median(nodes)], alpha=0.5, color='g', label='Nodes')
	# plt.xlabel('Nodes [10^6]')
	# plt.ylabel('Count')
	# print("Nodes")
	# print("Counts", counts)
	# print("Total Counts", sum(counts))
	# print("Median", statistics.median(nodes))
	# print("Edges" , edges)

	# plt.subplot(1,1,1)
	# # plt.hist(np.array(execution_times), bins=20, alpha=0.5, color='black', label='Execution Times')
	# counts, edges, plot = plt.hist(np.array(execution_times), bins=[0, statistics.median(execution_times)], alpha=0.5, color='b', label='Execution Times')
	# plt.xlabel('Execution Time [s]')
	# plt.ylabel('Count')
	# print("Execution Time")
	# print("Counts", counts)
	# print("Total Counts", sum(counts))
	# print("Median", statistics.median(execution_times))
	# print("Edges" , edges)

	# plt.subplot(1,1,1)
	# # plt.hist(np.array(lengths), bins=20, alpha=0.5, color='b', label='Lengths')
	# counts, edges, plot = plt.hist(np.array(lengths), alpha=0.5, color='black', label='Length')
	# plt.xlabel('Length [m]')
	# plt.ylabel('Count')
	# print("Path Length")
	# print("Counts", counts)
	# print("Total Counts", sum(counts))
	# print("Median", statistics.median(lengths))
	# print("Edges" , edges)

	# sns.set_style('darkgrid')
	# sns.distplot(a)

	# plt.savefig("test.png",bbox_inches='tight')
	# plt.show()


if __name__ == '__main__':
	# monte_carlo_sim()
	review()
	# graphs()