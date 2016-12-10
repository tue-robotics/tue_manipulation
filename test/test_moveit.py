import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

print("============ Start testing. First run: export ROS_NAMESPACE=amigo")

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander('left_arm')

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=10)

print("============ Reference frame: %s" % group.get_planning_frame())

group.set_pose_reference_frame('/amigo/base_link')

print("============ Reference frame: %s" % group.get_end_effector_link())

print("============ Robot Groups:")
print(robot.get_group_names())

print("============ Printing robot state")
print(robot.get_current_state())
print("============")


# print "============ Generating plan 1"
pose_target = group.get_random_pose()

group.set_pose_target(pose_target)

plan1 = group.plan()

print("============ Waiting while RVIZ displays plan1...")
rospy.sleep(5)

print("============ Execute plan")
group.go(wait=True)
