#! /usr/bin/env python

# Importing libraries
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates, ModelStates
from tf import transformations

# Defining Waypoint Co-ords
waypoint = MoveBaseActionGoal()

pub = None

def main():
	global pub

	rospy.init_node('nav')

	# pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
	pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

	rate = rospy.Rate(20)
		
	msg = PoseStamped()
	msg.pose.position.x = -10
	msg.pose.position.y = -2
	msg.pose.orientation.z = -1.57
	# msg = MoveBaseActionGoal()
	# msg.goal.target_pose.pose.position.x = -10
	# msg.goal.target_pose.pose.position.y = -2
	# msg.goal.target_pose.pose.orientation.z = -1.57
	
	pub.publish(msg)

if __name__ == '__main__':
	main()