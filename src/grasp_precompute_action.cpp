// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

#include <arm_navigation_msgs/MoveArmAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <tf/tf.h>

using namespace std;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> Server;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> Client;

ros::ServiceClient query_client;
ros::ServiceClient ik_client;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
ros::ServiceClient set_planning_scene_diff_client;

void execute(const amigo_arm_navigation::grasp_precomputeGoalConstPtr& goal, Server* as, Client* ac)
{
	// Test if the planning scene is up and running
	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

	printf("Hallo!\n");

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

	//for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	//{    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
	//}
	bool IK_FEASIBLE = false;
	double REQUESTED_YAW = tf::getYaw(gpik_req.ik_request.pose_stamped.pose.orientation);
	double SAMPLING_STEP = 0.1; // radians
	double YAW_DELTA = 0.0;
	int SAMPLING_DIRECTION = 1;
	double MAX_YAW_DELTA = 1.5708;
	tf::Quaternion quaternion;
	while(ros::ok() && !IK_FEASIBLE && YAW_DELTA < MAX_YAW_DELTA)
	{
		if(ik_client.call(gpik_req, gpik_res))
		{
			if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
			{
				printf("Desired IK feasible\n");
				IK_FEASIBLE = true;
			}
			else
			{
				printf("Desired IK not feasible: new yaw angle = %f\n",REQUESTED_YAW + (SAMPLING_DIRECTION * YAW_DELTA));
				YAW_DELTA = YAW_DELTA + SAMPLING_STEP;
				quaternion = tf::createQuaternionFromYaw(REQUESTED_YAW + (SAMPLING_DIRECTION * YAW_DELTA));
				gpik_req.ik_request.pose_stamped.pose.orientation.x = quaternion.getX();
				gpik_req.ik_request.pose_stamped.pose.orientation.y = quaternion.getY();
				gpik_req.ik_request.pose_stamped.pose.orientation.z = quaternion.getZ();
				gpik_req.ik_request.pose_stamped.pose.orientation.w = quaternion.getW();
                printf("x=%f y=%f z=%f w=%f",quaternion.getX(),quaternion.getY(),quaternion.getZ(),quaternion.getW());
				SAMPLING_DIRECTION = -1 * SAMPLING_DIRECTION;
			}
		}
		else
		{
			printf("Inverse kinematics service call failed");
		}
	}

    // Call upon move arm
	arm_navigation_msgs::MoveArmGoal magoal;

	magoal.motion_plan_request.group_name = "left_arm";
	magoal.motion_plan_request.num_planning_attempts = 5;
	magoal.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
	magoal.planner_service_name = "ompl_planning/plan_kinematic_path";

	arm_navigation_msgs::PositionConstraint position_constraint;
	position_constraint.header.frame_id = "base_link";
	position_constraint.link_name = "grippoint_left";
	position_constraint.position.x = goal->goal.pose.position.x;
	position_constraint.position.y = goal->goal.pose.position.y;
	position_constraint.position.z = goal->goal.pose.position.z;
	position_constraint.constraint_region_shape.type = 0;
	position_constraint.constraint_region_shape.dimensions.push_back(0.02);
	position_constraint.weight = 1.0;
	magoal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);

	arm_navigation_msgs::OrientationConstraint orientation_constraint;
	orientation_constraint.header.frame_id = "base_link";
	orientation_constraint.link_name = "grippoint_left";
	orientation_constraint.orientation.x = quaternion.getX();
	orientation_constraint.orientation.y = quaternion.getY();
	orientation_constraint.orientation.z = quaternion.getZ();
	orientation_constraint.orientation.w = quaternion.getW();
	orientation_constraint.absolute_roll_tolerance = 0.1;
	orientation_constraint.absolute_pitch_tolerance = 0.1;
	orientation_constraint.absolute_yaw_tolerance = 0.1;
	orientation_constraint.weight = 1.0;
	magoal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);

	ac->sendGoal(magoal);
	ac->waitForResult();
	if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
        as->setSucceeded();
        printf("Move arm succeeded\n");
	}
	else
	{
		as->setAborted();
		printf("Move arm failed\n");
	}

	ROS_INFO("Returning");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_precompute_server");
  ros::NodeHandle n;

  // Wait for the move arm server
  Client client("move_left_arm", true);
  client.waitForServer();

  // Initialize the grasp_precompute server
  Server server(n, "grasp_precompute", boost::bind(&execute, _1, &server, &client), false);
  server.start();

  // Initialize the IK clients
  query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("amigo_left_arm_kinematics/get_ik_solver_info");
  ik_client = n.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("amigo_left_arm_kinematics/get_constraint_aware_ik");
  set_planning_scene_diff_client = n.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  ros::spin();

  return 0;
}
