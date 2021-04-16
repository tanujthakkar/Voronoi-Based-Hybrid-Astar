#! /usr/bin/env python

# Importing libraries
import sys
from math import pi, sin, cos, tan
import pandas as pd
from random import uniform
from datetime import datetime
from time import sleep

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
	iteration_limits_bag = rosbag.Bag('../rosbags/iteration_limits_bag.bag', 'a')
	# bag = rosbag.Bag('../rosbags/20210414-172117.bag', 'a')

	try:
		while(valid_tests < 1000):
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
			sx = uniform(4.13, 45.71)
			sy = uniform(13.13, 36.80)
			syaw = uniform(-3.14, 3.14)
			syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
			
			# if(uniform(0.0, 1.0) >= 0.5):
			# 	gx = uniform(4.13, 33.75)
			# 	gy = uniform(13.13, 36.80)
			# else:
			# 	gx = uniform(34.15, 45.75)
			# 	gy = uniform(19.92, 30.02)
			gx = uniform(4.13, 45.71)
			gy = uniform(13.13, 36.80)
			gyaw = uniform(-3.14, 3.14)

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
	except (KeyboardInterrupt, rospy.service.ServiceException):
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

	bag = rosbag.Bag('../rosbags/20210414-172117.bag')
	# bag = rosbag.Bag('../rosbags/hospital_04/20210412-230907/20210412-230907.bag')
	# bag = rosbag.Bag('../rosbags/iteration_limits_bag.bag')
	
	for topic, msg, t in bag.read_messages(topics=['tests']):
		if(1):
			print([msg.sx, msg.sy, msg.syaw, msg.syaw_t, msg.gx, msg.gy, msg.gyaw, msg.solution_found, msg.iterations, msg.nodes, msg.execution_time])
			x = raw_input("Press Enter to continue...")
			if(x == 'q'):
				break

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

			# if(x == 'y'):
			# 	deltar = (RF - RB) / 2.0
			# 	deltat = (RTF - RTB) / 2.0

			# 	robot_center = PointStamped()
			# 	robot_center.header.stamp = rospy.Time.now()
			# 	robot_center.header.frame_id = "/map"
			# 	robot_center.point.x = msg.sx
			# 	robot_center.point.y = msg.sy
			# 	robot_center_pub.publish(robot_center)

			# 	cx = robot_center.point.x
			# 	cy = robot_center.point.y
			# 	yaw = msg.syaw
			# 	robot_polygon_pub.publish(create_polygon(RL, RW, cx, cy, yaw))

			# 	yawt = syaw_t
			# 	trailer_center = PointStamped()
			# 	trailer_center.header.stamp = rospy.Time.now()
			# 	trailer_center.header.frame_id = "/map"
			# 	trailer_center.point.x = (msg.sx - deltar * cos(yaw)) + deltat * cos(yawt)
			# 	trailer_center.point.y = (msg.sy - deltar * sin(yaw)) + deltat * sin(yawt)
			# 	trailer_center_pub.publish(trailer_center)

			# 	ctx = trailer_center.point.x
			# 	cty = trailer_center.point.y
			# 	trailer_polygon_pub.publish(create_polygon(TL, TW, ctx, cty, yawt))

			# 	for pose in range(len(msg.path.poses)):
			# 		sleep(0.05)
			# 		robot_center.point.x = msg.path.poses[pose].pose.position.x
			# 		robot_center.point.y = msg.path.poses[pose].pose.position.y
			# 		robot_center_pub.publish(robot_center)

			# 		cx = msg.path.poses[pose].pose.position.x
			# 		cy = msg.path.poses[pose].pose.position.y
			# 		quat = (msg.path.poses[pose].pose.orientation.x,
			# 				msg.path.poses[pose].pose.orientation.y,
			# 				msg.path.poses[pose].pose.orientation.z,
			# 				msg.path.poses[pose].pose.orientation.w)
			# 		yaw = tf.transformations.euler_from_quaternion(quat)[2]
			# 		robot_polygon_pub.publish(create_polygon(RL, RW, cx, cy, yaw))

			# 		yawt = pi_2_pi(node.get_yaw_t(n-1) + (dir * MOVE_STEP / RTR) * sin(node.get_yaw(0) - node.get_yaw_t(0)))
			# 		trailer_center.point.x = (msg.path.poses[0].pose.position.x - deltar * cos(yawlist[i])) + deltat * cos(yawt[i])
			# 		trailer_center.point.y = (msg.path.poses[0].pose.position.y - deltar * sin(yawlist[i])) + deltat * sin(yawt[i])
			# 		trailer_center_pub.publish(trailer_center)

			# 		trailer_center.point.x = trailer_center.point.x
			# 		trailer_center.point.y = trailer_center.point.y
			# 		trailer_center_pub.publish(create_polygon(TL, TW, ctx, cty, yawt))

	rate.sleep()


if __name__ == '__main__':
	monte_carlo_sim()
	# review()