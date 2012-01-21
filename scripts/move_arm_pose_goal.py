#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_arm_navigation')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
import geometry_msgs.msg

def euler_z_to_quaternion(angle):
	
	orientation_goal = geometry_msgs.msg.Quaternion()
	
	quaternion = tf.transformations.quaternion_from_euler(0,0,angle)
	orientation_goal.x = quaternion[0]
	orientation_goal.y = quaternion[1]
	orientation_goal.z = quaternion[2]
	orientation_goal.w = quaternion[3]
	
	
	return orientation_goal

def poseConstraintToPositionOrientationConstraints(pose_constraint):
	position_constraint = PositionConstraint()
	orientation_constraint = OrientationConstraint()
	position_constraint.header = pose_constraint.header
	position_constraint.link_name = pose_constraint.link_name
	position_constraint.position = pose_constraint.pose.position
	position_constraint.constraint_region_shape.type = 0
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
	position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

	position_constraint.constraint_region_orientation.x = 0.0
	position_constraint.constraint_region_orientation.y = 0.0
	position_constraint.constraint_region_orientation.z = 0.0
	position_constraint.constraint_region_orientation.w = 1.0

	position_constraint.weight = 1.0

	orientation_constraint.header = pose_constraint.header
	orientation_constraint.link_name = pose_constraint.link_name
	orientation_constraint.orientation = pose_constraint.pose.orientation
	orientation_constraint.type = pose_constraint.orientation_constraint_type

	orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
	orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
	orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
	orientation_constraint.weight = 1.0
     
	return (position_constraint, orientation_constraint)


def addGoalConstraintToMoveArmGoal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = poseConstraintToPositionOrientationConstraints(pose_constraint);
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

if __name__ == '__main__':

	rospy.init_node('pose_hand', anonymous=True)
	side = rospy.get_param("~side", "left")

	move_arm = actionlib.SimpleActionClient("move_" + side + "_arm", MoveArmAction)

	move_arm.wait_for_server()
	rospy.loginfo("Connected to move_"+side+"_arm server")

	goal = MoveArmGoal()

	goal.motion_plan_request.group_name = side + "_arm"
	goal.motion_plan_request.num_planning_attempts = 1
	goal.motion_plan_request.allowed_planning_time = rospy.Duration(10.0)

	goal.motion_plan_request.planner_id = ""
	goal.planner_service_name = "ompl_planning/plan_kinematic_path"

	desired_pose = SimplePoseConstraint()
	desired_pose.header.frame_id = "base_link"
	desired_pose.link_name = "grippoint_" + side;
	
	desired_pose.pose.position.x = -0.2;
	desired_pose.pose.position.y =  0.05;
	desired_pose.pose.position.z = 0.30;

	desired_pose.pose.orientation.x = 0.0;
	desired_pose.pose.orientation.y = 0.6;
	desired_pose.pose.orientation.z = 0.0;
	desired_pose.pose.orientation.w = 1.0;
	
	
	#desired_pose.pose.position.x = 0.4;
	#desired_pose.pose.position.y =  0.05;
	#desired_pose.pose.position.z = 0.30;

	#desired_pose.pose.orientation.x = 0.0;
	#desired_pose.pose.orientation.y = 0.1;
	#desired_pose.pose.orientation.z = 0.0;
	#desired_pose.pose.orientation.w = 0.9;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.2;
	desired_pose.absolute_pitch_tolerance = 0.2;
	desired_pose.absolute_yaw_tolerance = 0.2;

	addGoalConstraintToMoveArmGoal(desired_pose, goal)

	finished_within_time = False
	move_arm.send_goal(goal) 
	finished_within_time = move_arm.wait_for_result(rospy.Duration(200));
	if not finished_within_time:
		move_arm.cancel_goal()
		rospy.loginfo("Timed out achieving goal")
	else:
		state = move_arm.get_state()
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("Action succeeded!")
	else:
		rospy.loginfo("Action failed with error code: " + str(state))

	print "Finished"
