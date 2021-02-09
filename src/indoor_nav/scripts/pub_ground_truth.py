#! /usr/bin/env python

# Importing libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates, ModelStates
from tf import transformations

# Defining Robot State
position = Point()
yaw = 0

# Publisher to control the robot
pub = None

# Getting true robot position from Gazebo
def callback_true_pos(msg):
	global position
	global yaw

	position = msg.pose[2].position

	quaternion = (
		msg.pose[2].orientation.x, 
		msg.pose[2].orientation.y,
		msg.pose[2].orientation.z,
		msg.pose[2].orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw = euler[2]


def main():
	global pub

	rospy.init_node('nav_to_waypoint')

	pub = rospy.Publisher('/ground_truth', Odometry, queue_size = 1)
	
	sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback_true_pos)

	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		rate.sleep()


if __name__ == '__main__':
	main()