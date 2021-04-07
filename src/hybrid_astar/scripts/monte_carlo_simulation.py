#! /usr/bin/env python

# Importing libraries
import sys
from math import pi
import pandas as pd
from random import uniform
from datetime import datetime

import rospy
import hybrid_astar
from hybrid_astar.srv import MonteCarloSim

def pi_to_pi(yaw):
	while(yaw > pi):
		yaw -= 2.0 * pi;

	while(yaw < -pi):
		yaw += 2.0 * pi;

	return yaw


def main():

	rospy.wait_for_service("monte_carlo_sim_service")
	test = rospy.ServiceProxy("monte_carlo_sim_service", hybrid_astar.srv.MonteCarloSim)

	total_tests = 0
	valid_tests = 0
	successful_tests = 0
	tests = []
	redundant = 0

	now = datetime.now()
	dt_string = now.strftime("%Y%m%d-%H%M%S")
	df = pd.DataFrame([], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time', 'Path'])
	df.to_csv(dt_string + '.csv', mode = 'a')

	while(valid_tests < 10000):
		total_tests = total_tests + 1
		# sx = 31.50
		# sy = 10.40
		# syaw = -1.57
		# syaw_t = -1.57
		# gx = 24.03
		# gy = 16.65
		# gyaw = -3.10
		if(uniform(0.0, 1.0) >= 0.5):
			sx = uniform(0.68 ,30.0)
			sy = uniform(0.75 ,24.0)
		else:
			sx = uniform(30.7 ,42.25)
			sy = uniform(7.60 ,12.60)
		syaw = uniform(-3.14, 3.14)
		syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
		if(uniform(0.0, 1.0) >= 0.5):
			gx = uniform(0.68 ,30.0)
			gy = uniform(0.75 ,24.0)
		else:
			gx = uniform(30.7 ,42.25)
			gy = uniform(7.60 ,12.60)
		gyaw = uniform(-3.14, 3.14)
		if([sx, sy, syaw, syaw_t, gx, gy, gyaw] in tests):
			redundant = redundant + 1
			continue
		tests.append([sx, sy, syaw, syaw_t, gx, gy, gyaw])
		result = test(sx, sy, syaw, syaw_t, gx, gy, gyaw)
		if(result.valid_start and result.valid_goal):
			x = []
			y = []
			print('Test: ', valid_tests)
			# print("Valid Start ", result.valid_start)
			# print("Valid Goal ", result.valid_goal)
			# print("Solution Found ", result.solution_found)
			# print("Path ", result.path)
			# print("Iterations: ", result.iterations)
			# print("Nodes: ", result.nodes + 1)
			# print("Execution Time: ", result.execution_time)
			if(result.solution_found):
				successful_tests = successful_tests + 1
				solution_found = 'TRUE'
				for i in range(len(result.path.poses)):
					x.append(int(result.path.poses[i].pose.position.x))
					y.append(int(result.path.poses[i].pose.position.y))
			else:
				solution_found = 'FALSE'
			valid_tests = valid_tests + 1
			df = pd.DataFrame([[sx, sy, syaw, syaw_t, gx, gy, gyaw, solution_found, result.iterations, result.nodes + 1, result.execution_time, [x, y]]], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time', 'Path'])
			df.to_csv(dt_string + '.csv', mode = 'a', header = False)
	
	print(x, y)
	print("Successful Tests: ", successful_tests)
	print("Redundant Tests: ", redundant)
	print("Total Tests: ", total_tests)


if __name__ == '__main__':
	main()