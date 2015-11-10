#!/usr/bin/python

import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal

rospy.init_node('visualize_joint_trajectory_action_goal')

def callback(goal):	
	print goal
	import ipdb; ipdb.set_trace()

rospy.subscribe("goal", FollowJointTrajectoryActionGoal, queue_size=10)
rospy.spin()