#! /usr/bin/env python

# Importing libraries
import rospy
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf import transformations

import math

d = 1
l = 1
t = 1

x = input("x : ")
y = input("y : ")
yaw = input("yaw : ")

x_t = x + d * cos()

if __name__ == '__main__':
	main()