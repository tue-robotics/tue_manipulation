#! /usr/bin/env python
import roslib; roslib.load_manifest('amigo_arm_navigation')
import rospy
import sys
import kinematics_msgs.srv
import geometry_msgs.msg
import arm_navigation_msgs.srv
import tf

''' Returns and publishes IK solution for grippoint_left w.r.t. base_link '''

''' Global variables '''
planning_scene_service = None
info_service = None
ik_service = None

def rpy_to_quaternion(roll, pitch, yaw):
    
    rospy.logwarn("These should be euler angles")
    orientation_goal = geometry_msgs.msg.Quaternion()
    
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation_goal.x = quaternion[0]
    orientation_goal.y = quaternion[1]
    orientation_goal.z = quaternion[2]
    orientation_goal.w = quaternion[3]
    
    return orientation_goal
    
def get_ik(x, y, z, roll, pitch, yaw):
    
    ''' Get IK info '''
    info_req = kinematics_msgs.srv.GetKinematicSolverInfoRequest()
    ik_info = info_service.call(info_req)
    
    ''' Get IK solution '''
    ik_req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
    ik_req.timeout = rospy.Duration(2.0)
    ik_req.ik_request.ik_link_name = "grippoint_left"
    ik_req.ik_request.pose_stamped.header.frame_id = "/amigo/base_link"
    ik_req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
    
    for joint_name in ik_info.kinematic_solver_info.joint_names:
        ik_req.ik_request.ik_seed_state.joint_state.name.append(joint_name)
        ''' ToDo: fill in current joint angles '''
        ik_req.ik_request.ik_seed_state.joint_state.position.append(0.0)
        
    ik_req.ik_request.pose_stamped.pose.position.x = x
    ik_req.ik_request.pose_stamped.pose.position.y = y
    ik_req.ik_request.pose_stamped.pose.position.z = z
    ik_req.ik_request.pose_stamped.pose.orientation = rpy_to_quaternion(roll, pitch, yaw)
    
    rospy.loginfo("IK request = \n{0}\n".format(ik_req))
    ik_solution = ik_service.call(ik_req)
    rospy.loginfo("IK solution = \n{0}\n".format(ik_solution))

if __name__ == '__main__':

    rospy.init_node('test_IK')
    rospy.loginfo("Node initialized")
    
    planning_scene_service = rospy.ServiceProxy('/amigo/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
    info_service = rospy.ServiceProxy('/amigo/left_arm_kinematics/get_ik_solver_info', kinematics_msgs.srv.GetKinematicSolverInfo)
    ik_service = rospy.ServiceProxy('/amigo/left_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
    
    ''' Wait for services??? '''
    rospy.loginfo("Waiting for services")
    rospy.wait_for_service("/amigo/environment_server/set_planning_scene_diff", timeout=2.0)
    rospy.wait_for_service("/amigo/left_arm_kinematics/get_ik_solver_info", timeout=2.0)
    rospy.wait_for_service("/amigo/left_arm_kinematics/get_constraint_aware_ik", timeout=2.0)
    rospy.loginfo("Services connected")
    
    ''' Set planning scene diff '''
    planning_scene_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
    planning_scene_service.call(planning_scene_req)
    
    starttime = rospy.Time.now()
    get_ik(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]))
    endtime = rospy.Time.now()
    duration = endtime-starttime
    rospy.loginfo("IK call took {0} seconds".format(duration.to_sec()))
