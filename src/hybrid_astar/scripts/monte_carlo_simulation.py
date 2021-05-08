#! /usr/bin/env python

# Importing libraries
import sys
from math import pi, sin, cos, tan
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

	# bag = rosbag.Bag('../rosbags/' + dt_string + '.bag', 'w')
	iteration_limits_bag = rosbag.Bag('../rosbags/iteration_limits_bag_voronoi.bag', 'a')
	bag = rosbag.Bag('../rosbags/20210508-010746.bag', 'a')

	r_bag = rosbag.Bag('../rosbags/hospital_04/hospital_env_monte_carlo_results/hospital_env_monte_carlo_results.bag')

	try:
		# while(valid_tests < 1000):
		for topic, msg, t in r_bag.read_messages(topics=['tests']):
			total_tests = total_tests + 1
				
			# Indoor Environment
			# sx = uniform(1.5 ,20.0)
			# sy = uniform(2.0 ,20.0)
			# syaw = uniform(-3.14, 3.14)
			# syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
			# gx = uniform(1.5 ,20.0)
			# gy = uniform(2.0 ,20.0)
			# gyaw = uniform(-3.14, 3.14)

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


			if(total_tests < 161):
				continue

			sx = msg.sx
			sy = msg.sy
			syaw = msg.syaw
			syaw_t = msg.syaw_t
			gx = msg.gx
			gy = msg.gy
			gyaw = msg.gyaw

			test_response = test_call(sx, sy, syaw, syaw_t, gx, gy, gyaw)

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
					iteration_limits_bag.write('tests', test)
					continue

				valid_tests = valid_tests + 1
				print("Test ", valid_tests)
				# print([sx, sy, syaw, syaw_t, gx, gy, gyaw])
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


def create_polygon(l, w, cx, cy, yaw):

	polygon = PolygonStamped()
	polygon.header.stamp = rospy.Time.now()
	polygon.header.frame_id = "/map"
	# del polygon.polygon.points[:]

	length = l
	width = w

	p = Point32()
	p.z = 0.0
	
	# Top Right
	p.x = cx + ((length/2) * cos(yaw)) - ((width/2) * sin(yaw))
	p.y = cy + ((length/2) * sin(yaw)) + ((width/2) * cos(yaw))
	polygon.polygon.points.append(p)

	# Top Left
	p.x = cx - ((length/2) * cos(yaw)) - ((width/2) * sin(yaw))
	p.y = cy - ((length/2) * sin(yaw)) + ((width/2) * cos(yaw))
	polygon.polygon.points.append(p)

	# Bottom Left
	p.x = cx - ((length/2) * cos(yaw)) + ((width/2) * sin(yaw))
	p.y = cy - ((length/2) * sin(yaw)) - ((width/2) * cos(yaw))
	polygon.polygon.points.append(p)

	# Bottom Right
	p.x = cx + ((length/2) * cos(yaw)) + ((width/2) * sin(yaw))
	p.y = cy + ((length/2) * sin(yaw)) - ((width/2) * cos(yaw))
	polygon.polygon.points.append(p)

	return polygon


def review():

	rospy.init_node('test_review')

	rospy.wait_for_service("monte_carlo_sim_service")
	test_call = rospy.ServiceProxy("monte_carlo_sim_service", hybrid_astar.srv.MonteCarloSim)

	start_pose_pub = rospy.Publisher('start_pose', PoseStamped, queue_size = 1)
	goal_pose_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size = 1)
	path_pub = rospy.Publisher('global_path', Path, queue_size = 1)

	robot_center_pub = rospy.Publisher('robot_center', PointStamped, queue_size = 1)
	robot_polygon_pub = rospy.Publisher('robot_polygon', PolygonStamped, queue_size = 1)
	trailer_center_pub = rospy.Publisher('trailer_center', PointStamped, queue_size = 1)
	trailer_polygon_pub = rospy.Publisher('trailer_polygon', PolygonStamped, queue_size = 1)

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

	# bag = rosbag.Bag('../rosbags/20210414-172117.bag')
	bag = rosbag.Bag('../rosbags/hospital_04/hospital_env_monte_carlo_results/hospital_env_monte_carlo_results.bag')
	# bag = rosbag.Bag('../rosbags/iteration_limits_bag.bag')
	
	i = 0
	for topic, msg, t in bag.read_messages(topics=['tests']):
		i = i + 1
		if(i == 110):
		# if(i in [13, 16, 23, 34, 106, 133, 134]):
			print("Test", i)
			print([msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw, msg.solution_found, msg.iterations, msg.nodes, msg.execution_time])
			sleep(0.5)
			
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

			x = raw_input("Press Enter to visualize...")
			if(x == 'q'):
				break
			if(x == 'y'):
				test_response = test_call(msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw)
			raw_input("Press Enter to continue...")

	rate.sleep()

def graphs():

	bag = rosbag.Bag('../rosbags/hospital_04/hospital_env_monte_carlo_results/hospital_env_monte_carlo_results.bag')

	iterations = []
	nodes = []
	lengths = []
	execution_times = []

	i = 0
	for topic, msg, t in bag.read_messages(topics=['tests']):
		i = i + 1
		if(msg.solution_found):
			# print("Test", i)
			# print([msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw, msg.solution_found, msg.iterations, msg.nodes, msg.execution_time])
			iterations.append(msg.iterations)
			nodes.append(msg.nodes * 0.000001)
			lengths.append(len(msg.path.poses) * 0.1)
			execution_times.append(msg.execution_time/1000)

	# nodes = list(map(float, nodes))

	plt.subplot(1,1,1)
	# counts, edges, plot = plt.hist(np.array(iterations), bins=[0, 25000, 50000, 75000, 100000, 125000, 150000, 175000, 200000, 225000, 250000, 300000, 350000], alpha=0.5, color='r', label='Iterations')
	counts, edges, plot = plt.hist(np.array(iterations), bins=[0, 25000], alpha=0.5, color='r', label='Iterations')
	plt.xlabel('Iterations')
	plt.ylabel('Count')
	print(counts)
	print(sum(counts))
	print(statistics.median(iterations))
	print(edges)

	plt.subplot(1,1,1)
	# counts, edges, plot = plt.hist(np.array(iterations), bins=[0, 0.5, 1.5, 2, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5], alpha=0.5, color='r', label='Iterations')
	counts, edges, plot = plt.hist(np.array(nodes), bins=[0, 0.197197], alpha=0.5, color='g', label='Nodes')
	plt.xlabel('Nodes [10^6]')
	plt.ylabel('Count')
	print(counts)
	print(sum(counts))
	print(statistics.median(nodes))
	print(edges)

	# plt.subplot(1,1,1)
	# # plt.hist(np.array(lengths), bins=20, alpha=0.5, color='b', label='Lengths')
	# counts, edges, plot = plt.hist(np.array(lengths), alpha=0.5, color='black', label='Length')
	# plt.xlabel('Length [m]')
	# plt.ylabel('Count')
	# print(counts)
	# print(sum(counts))
	# print(statistics.median(lengths))
	# print(edges)

	plt.subplot(1,1,1)
	# plt.hist(np.array(execution_times), bins=20, alpha=0.5, color='black', label='Execution Times')
	counts, edges, plot = plt.hist(np.array(execution_times), bins=[0, 109], alpha=0.5, color='b', label='Execution Times')
	plt.xlabel('Execution Time [s]')
	plt.ylabel('Count')
	print(counts)
	print(sum(counts))
	print(statistics.median(execution_times))
	print(edges)

	# sns.set_style('darkgrid')
	# sns.distplot(a)

	# plt.savefig("test.png",bbox_inches='tight')
	# plt.show()


if __name__ == '__main__':
	iteration_limits_bag = rosbag.Bag('../rosbags/iteration_limits_bag_voronoi.bag', 'w')
	iteration_limits_bag.close()
	monte_carlo_sim()
	# review()
	# graphs()