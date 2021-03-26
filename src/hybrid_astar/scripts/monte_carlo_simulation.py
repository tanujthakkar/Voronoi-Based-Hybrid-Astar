#! /usr/bin/env python

# Importing libraries
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
	results = []

	now = datetime.now()
	dt_string = now.strftime("%Y%m%d-%H%M%S")
	df = pd.DataFrame([], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time', 'Path'])
	df.to_csv(dt_string + '.csv', mode = 'a')

	while(valid_tests < 5):
		total_tests = total_tests + 1
		# sx = 6.24
		# sy = 5.25
		# syaw = 0.0
		# syaw_t = 0.0
		# gx = 17.06
		# gy = 8.50
		# gyaw = 1.57
		sx = uniform(1.5 ,20.0)
		sy = uniform(2.0 ,20.0)
		syaw = uniform(-3.14, 3.14)
		syaw_t = uniform(min(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)), max(pi_to_pi(syaw - 1.395), pi_to_pi(syaw + 1.395)))
		gx = uniform(1.5 ,20.0)
		gy = uniform(2.0 ,20.0)
		gyaw = uniform(-3.14, 3.14)
		result = test(sx, sy, syaw, syaw_t, gx, gy, gyaw)
		if(result.valid_start and result.valid_goal):
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
				x = []
				y = []
				for i in range(len(result.path.poses)):
					x.append(result.path.poses[i].pose.position.x)
					y.append(result.path.poses[i].pose.position.y)
			else:
				solution_found = 'FALSE'
				x = []
				y = []
			valid_tests = valid_tests + 1
			df = pd.DataFrame([[sx, sy, syaw, syaw_t, gx, gy, gyaw, solution_found, result.iterations, result.nodes + 1, result.execution_time, [x, y]]], columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time', 'Path'])
			df.to_csv(dt_string + '.csv', mode = 'a', header = False)
	
	print("Successful Test: ", successful_tests)
	print("Total Tests: ", total_tests)


if __name__ == '__main__':
	main()