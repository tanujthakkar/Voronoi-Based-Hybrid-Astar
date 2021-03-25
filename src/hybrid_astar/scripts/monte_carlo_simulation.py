#! /usr/bin/env python

# Importing libraries
import math
import pandas as pd
from random import uniform
from datetime import datetime

import rospy
import hybrid_astar
from hybrid_astar.srv import MonteCarloSim

def main():

	rospy.wait_for_service("monte_carlo_sim_service")
	test = rospy.ServiceProxy("monte_carlo_sim_service", hybrid_astar.srv.MonteCarloSim)


	total_tests = 0
	successful_tests = 0
	results = []

	while(successful_tests < 10):
		total_tests = total_tests + 1
		sx = uniform(1.5 ,20.0)
		sy = uniform(2.0 ,20.0)
		syaw = uniform(-3.14, 3.14)
		syaw_t = uniform(min(syaw - 1.395, syaw + 1.395), max(syaw - 1.395, syaw + 1.395))
		gx = uniform(1.5 ,20.0)
		gy = uniform(2.0 ,20.0)
		gyaw = uniform(-3.14, 3.14)
		result = test(sx, sy, syaw, syaw_t, gx, gy, gyaw)
		if(result.valid_start and result.valid_goal):
			# print("Valid Start ", result.valid_start)
			# print("Valid Goal ", result.valid_goal)
			# print("Solution Found ", result.solution_found)
			# print("Path ", result.path)
			# print("Iterations: ", result.iterations)
			# print("Nodes: ", result.nodes + 1)
			# print("Execution Time: ", result.execution_time)
			x = []
			y = []
			for i in range(len(result.path.poses)):
				x.append(result.path.poses[i].pose.position.x)
				y.append(result.path.poses[i].pose.position.y)
			successful_tests = successful_tests + 1
			results.append([sx, sy, syaw, syaw_t, gx, gy, gyaw, result.solution_found, result.iterations, result.nodes + 1, result.execution_time, [x, y]])

	df = pd.DataFrame(results, columns = ['sx', 'sy', 'syaw', 'syaw_t', 'gx', 'gy', 'gyaw', 'Solution', 'Iterations', 'Nodes', 'Execution Time', 'Path'])
	now = datetime.now()
	dt_string = now.strftime("%Y%m%d-%H%M%S")
	df.to_csv(dt_string + '.csv')
	print("Total Tests: ", total_tests)


if __name__ == '__main__':
	main()