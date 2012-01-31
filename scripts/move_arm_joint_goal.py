#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_arm_navigation')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg

if __name__ == '__main__':

	rospy.init_node('pose_hand', anonymous=True)
	side = rospy.get_param("~side", "left")

	move_arm = actionlib.SimpleActionClient("move_" + side + "_arm", MoveArmAction)

	move_arm.wait_for_server()
	rospy.loginfo("Connected to move_"+side+"_arm server")

	goal = MoveArmGoal()

	goal.motion_plan_request.group_name = side + "_arm"
	goal.motion_plan_request.num_planning_attempts = 5
	goal.motion_plan_request.allowed_planning_time = rospy.Duration(10.0)

	goal.motion_plan_request.planner_id = ""
	goal.planner_service_name = "ompl_planning/plan_kinematic_path"

	joints =   ("shoulder_yaw_joint_left",
				"shoulder_pitch_joint_left",
				"shoulder_roll_joint_left",
				"elbow_pitch_joint_left",
				"elbow_roll_joint_left",
				"wrist_pitch_joint_left",
				"wrist_yaw_joint_left")
	
	# JointConstraint : (joint, position, tolerance_below, tolerance_above, weight)				
	goal.motion_plan_request.goal_constraints.joint_constraints = [JointConstraint(j, 0.0, 0.1, 0.1, 0.0) for j in joints]
	joint_goal1 = (-0.001, -0.4,  0.0, 1.2,  0.0,  0.8, 0.0)
	joint_goal2 = (-0.2, 0.13411, 0.480, 1.35367, -0.02917, 0.2149, -0.46194)
	joint_goal3 = (-0.001, -0.4,  0.0, 1.2,  0.0,  0.8, 0.0)	
	    
	
	goal_poses = (joint_goal1, joint_goal2, joint_goal3)	
	num_goals = len(goal_poses)
	
	for k in range(num_goals*1):
		goal_pos = goal_poses[k%num_goals]
 
		for i, value in enumerate(goal_pos):
			goal.motion_plan_request.goal_constraints.joint_constraints[i].position = value
       
		move_arm.send_goal(goal)
		finished_within_time = move_arm.wait_for_result(rospy.Duration(200.0))
 
		if not finished_within_time:
			move_arm.cancel_goal()
			rospy.loginfo('Timed out trying to achieve a joint goal')
		else:
			state = move_arm.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo('Action position %d finished' % (k+1))
			else:
				rospy.loginfo('Action position %d failed: %s' % (k+1) % str(state))
				
        # time between poses
		rospy.sleep(2)
	
print "Move_arm_joint_goal is Finished"

