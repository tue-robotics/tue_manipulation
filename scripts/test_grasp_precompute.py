#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_arm_navigation')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg
from amigo_arm_navigation.msg._grasp_precomputeGoal import grasp_precomputeGoal
from amigo_arm_navigation.msg._grasp_precomputeAction import grasp_precomputeAction

if __name__ == '__main__':
	rospy.init_node('test_grasp_precompute_left', anonymous=True)
	
	ac_grasp_precompute_left = actionlib.SimpleActionClient("grasp_precompute_left", grasp_precomputeAction)
	ac_grasp_precompute_left.wait_for_server()#rospy.Duration(5.0)
	
	grasp_precompute_goal = grasp_precomputeGoal()
	grasp_precompute_goal.goal.header.frame_id = '/base_link'
	grasp_precompute_goal.goal.header.stamp = rospy.Time.now()
	
	grasp_precompute_goal.PERFORM_PRE_GRASP = True
	
	grasp_precompute_goal.goal.x = 0.5
	grasp_precompute_goal.goal.y = 0.7
	grasp_precompute_goal.goal.z = 0.9
	
	grasp_precompute_goal.goal.roll = 0
	grasp_precompute_goal.goal.pitch = 0
	grasp_precompute_goal.goal.yaw = 0
	
	print "Calling action"
	ac_grasp_precompute_left.send_goal(grasp_precompute_goal)
	print "Goal sent, waiting for result..."
	result = ac_grasp_precompute_left.wait_for_result(rospy.Duration(60.0))
	print "Action ended"
	if result:
		print "Grasp precompute successful"
	else:
		print "Grasp precompute unsuccessful"
	
	print "grasp_precomputeGoal is Finished"

