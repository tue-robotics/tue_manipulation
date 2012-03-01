// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

#include <arm_navigation_msgs/MoveArmAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

using namespace std;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> Server;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> Client;

ros::ServiceClient query_client;
ros::ServiceClient ik_client;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
ros::ServiceClient set_planning_scene_diff_client;

void execute(const amigo_arm_navigation::grasp_precomputeGoalConstPtr& goal, Server* as)
{
	// Test if the planning scene is up and running
	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

	if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
	  ROS_ERROR("Can't get planning scene");
	  ros::shutdown();
	  exit(-1);
	}

	// Get Kinematics solver info for seed state
	kinematics_msgs::GetKinematicSolverInfo::Request request;
	kinematics_msgs::GetKinematicSolverInfo::Response response;
	if(query_client.call(request,response))
	{
	  for(unsigned int i=0; i<response.kinematic_solver_info.joint_names.size(); i++)
	  {
	    ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
	  }
	}
	else
	{
	  ROS_ERROR("Could not call query service");
	  ros::shutdown();
	  exit(-1);
	}

	//Call IK
	kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
	kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

	gpik_req.timeout = ros::Duration(5.0);
	gpik_req.ik_request.ik_link_name = "grippoint_left";

	gpik_req.ik_request.pose_stamped.header.frame_id = goal->goal.header.frame_id;
	gpik_req.ik_request.pose_stamped.pose.position.x = goal->goal.pose.position.x;
	gpik_req.ik_request.pose_stamped.pose.position.y = goal->goal.pose.position.y;
	gpik_req.ik_request.pose_stamped.pose.position.z = goal->goal.pose.position.z;

	gpik_req.ik_request.pose_stamped.pose.orientation.x = goal->goal.pose.orientation.x;
	gpik_req.ik_request.pose_stamped.pose.orientation.y = goal->goal.pose.orientation.y;
	gpik_req.ik_request.pose_stamped.pose.orientation.z = goal->goal.pose.orientation.z;
	gpik_req.ik_request.pose_stamped.pose.orientation.w = goal->goal.pose.orientation.w;

	gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	{    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
	}
	if(ik_client.call(gpik_req, gpik_res))
	{
	  if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
	  {
	    for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
	    {        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
	    }
	  }
	  else
	  {
	    ROS_ERROR("Inverse kinematics failed");
	  }
	}
	else
	{
	  ROS_ERROR("Inverse kinematics service call failed");
	}

    // Do lots of awesome groundbreaking robot stuff here
    as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_precompute_server");
  ros::NodeHandle n;

  // Initialize the grasp_precompute server
  Server server(n, "grasp_precompute", boost::bind(&execute, _1, &server), false);
  server.start();

  // Wait for the move arm server
  Client client("move_left_arm", true);
  client.waitForServer();

  // Initialize the IK clients
  query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("amigo_left_arm_kinematics/get_ik_solver_info");
  ik_client = n.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("amigo_left_arm_kinematics/get_constraint_aware_ik");
  set_planning_scene_diff_client = n.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  // Make move arm goal
  arm_navigation_msgs::MoveArmGoal goal;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  printf("Move arm succeeded");

  ros::spin();
  return 0;
}

//#include <ros/ros.h>
//#include <kinematics_msgs/GetKinematicSolverInfo.h>
//#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
//#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
//
//static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
//
//int main(int argc, char **argv){
//  ros::init (argc, argv, "get_fk");
//  ros::NodeHandle rh;
//
//  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
//  ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
//
//  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
//  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
//  ros::ServiceClient set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
//
//  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
//  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
//
//  if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
//    ROS_WARN("Can't get planning scene");
//    return -1;
//  }
//
//  // define the service messages
//  kinematics_msgs::GetKinematicSolverInfo::Request request;
//  kinematics_msgs::GetKinematicSolverInfo::Response response;
//
//  if(query_client.call(request,response))
//  {
//    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//    {
//      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
//    }
//  }
//  else
//  {
//    ROS_ERROR("Could not call query service");
//    ros::shutdown();
//    exit(-1);
//  }
//
//  // define the service messages
//  kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
//  kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;
//
//  gpik_req.timeout = ros::Duration(5.0);
//  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";
//
//  gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
//  gpik_req.ik_request.pose_stamped.pose.position.x = 0.75;
//  gpik_req.ik_request.pose_stamped.pose.position.y = -0.188;
//  gpik_req.ik_request.pose_stamped.pose.position.z = 0.0;
//
//  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
//  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
//  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
//  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
//  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_nam
//es.size());
//  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
//
//  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
//  {    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i
//].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
//  }
//  if(ik_client.call(gpik_req, gpik_res))
//  {
//    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
//    {
//      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
//      {        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_st
//ate.position[i]);
//      }
//    }
//    else
//    {
//      ROS_ERROR("Inverse kinematics failed");
//    }
//  }
//  else
//  {
//    ROS_ERROR("Inverse kinematics service call failed");
//  }
//  ros::shutdown();
//}
