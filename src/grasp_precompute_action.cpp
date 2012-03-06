// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <amigo_actions/AmigoSpindleCommandAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <amigo_msgs/arm_joints.h>
#include <amigo_msgs/spindle_setpoint.h>

#include <tf/tf.h>

using namespace std;

double MAX_YAW_DELTA,YAW_SAMPLING_STEP,PRE_GRASP_DELTA,SPINDLE_MIN,SPINDLE_MAX,SPINDLE_SAMPLING_STEP;
int MAX_RESEND_ATTEMPTS;
string SIDE;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> Server;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> Client;
typedef actionlib::SimpleActionClient<amigo_actions::AmigoSpindleCommandAction> SpindleClient;

ros::Publisher *spindlepub;

ros::ServiceClient query_client;
ros::ServiceClient ik_client;

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
ros::ServiceClient set_planning_scene_diff_client;

amigo_msgs::arm_joints arm_joints;
double spindle_position;

void armcontrollerCB(const amigo_msgs::arm_jointsConstPtr &joint_meas)
{
	arm_joints = *joint_meas;
}

void spindlecontrollerCB(const std_msgs::Float64ConstPtr &spindle_meas)
{
	spindle_position = spindle_meas->data;
}

void execute(const amigo_arm_navigation::grasp_precomputeGoalConstPtr& goal, Server* as, Client* ac, SpindleClient* sc)
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

	// Define general move arm goal info
	arm_navigation_msgs::MoveArmGoal magoal;
	magoal.motion_plan_request.group_name = SIDE + "_arm";
	magoal.motion_plan_request.num_planning_attempts = 1;
	magoal.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
	magoal.planner_service_name = "ompl_planning/plan_kinematic_path";

	arm_navigation_msgs::PositionConstraint position_constraint;
	position_constraint.header.frame_id = goal->goal.header.frame_id;
	position_constraint.link_name = "grippoint_" + SIDE;
	position_constraint.constraint_region_shape.type = 0;
	position_constraint.constraint_region_shape.dimensions.push_back(0.02);
	magoal.motion_plan_request.goal_constraints.position_constraints.resize(1);
	position_constraint.weight = 1.0;

	arm_navigation_msgs::OrientationConstraint orientation_constraint;
	orientation_constraint.header.frame_id = goal->goal.header.frame_id;
	orientation_constraint.link_name = "grippoint_" + SIDE;
	orientation_constraint.absolute_roll_tolerance = 0.1;
	orientation_constraint.absolute_pitch_tolerance = 0.1;
	orientation_constraint.absolute_yaw_tolerance = 0.1;
	magoal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
	orientation_constraint.weight = 1.0;

	//Call IK
	kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
	kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

	gpik_req.timeout = ros::Duration(5.0);
	gpik_req.ik_request.ik_link_name = "grippoint_" + SIDE;
    gpik_req.ik_request.pose_stamped.header.frame_id = goal->goal.header.frame_id;

    // Define joint names and seed positions
	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	{
		//double joint_seed = (response.kinematic_solver_info.limits[i].max_position + response.kinematic_solver_info.limits[i].min_position)/2.0;
		double joint_seed = arm_joints.pos[i].data;
		string joint_name = response.kinematic_solver_info.joint_names[i];
		gpik_req.ik_request.ik_seed_state.joint_state.name.push_back(joint_name);
		gpik_req.ik_request.ik_seed_state.joint_state.position.push_back(joint_seed);
		//magoal.planning_scene_diff.robot_state.joint_state.name.push_back(joint_name);
		//magoal.planning_scene_diff.robot_state.joint_state.position.push_back(joint_seed);
	}

    // Define tf transform pose
    tf::Transform grasp_pose(tf::createQuaternionFromRPY(goal->goal.roll,goal->goal.pitch,goal->goal.yaw),tf::Point(goal->goal.x,goal->goal.y,goal->goal.z));
	tf::poseTFToMsg(grasp_pose, gpik_req.ik_request.pose_stamped.pose);

	tf::Transform new_grasp_pose;
	tf::Transform new_pre_grasp_pose;
	bool GRASP_FEASIBLE = false, SAMPLING_BOUNDARIES_REACHED=false, SPINDLE_REQUIRED = false;
	double YAW_DELTA = 0.0, HEIGHT_DELTA = 0.0, SPINDLE_DELTA = 0.0;
	int YAW_SAMPLING_DIRECTION = 1, HEIGHT_SAMPLING_DIRECTION = 1;
	while(ros::ok() && !GRASP_FEASIBLE && !SAMPLING_BOUNDARIES_REACHED )
	{
		printf("YAW_DELTA=%f \t HEIGHT_DELTA=%f\n",YAW_SAMPLING_DIRECTION * YAW_DELTA,HEIGHT_DELTA);
		// Define new_grasp_pose
		tf::Transform height_offset(tf::Quaternion(0,0,0,1),tf::Point(0,0,-HEIGHT_DELTA));
		tf::Transform yaw_offset(tf::createQuaternionFromYaw(YAW_SAMPLING_DIRECTION * YAW_DELTA),tf::Point(0,0,0));
		new_grasp_pose = grasp_pose * height_offset * yaw_offset;

		//Define pre_grasp_pose
		tf::Transform pre_grasp_offset(tf::Quaternion(0,0,0,1),tf::Point(-PRE_GRASP_DELTA,0,0));
		new_pre_grasp_pose = new_grasp_pose * pre_grasp_offset;

		// Check if they both are valid
		tf::poseTFToMsg(new_grasp_pose, gpik_req.ik_request.pose_stamped.pose);
		if(ik_client.call(gpik_req, gpik_res))
		{
			if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
			{
				printf("Grasp_pose feasible\n");
				if(goal->PERFORM_PRE_GRASP)
				{
					tf::poseTFToMsg(new_pre_grasp_pose, gpik_req.ik_request.pose_stamped.pose);
					if(ik_client.call(gpik_req, gpik_res))
					{
						if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
						{
							printf("Pre_grasp_pose feasible\n");
							GRASP_FEASIBLE = true;
							break;
						}
					}else{printf("IK call unsuccessful\n");}
				}
				else
				{
					GRASP_FEASIBLE = true;
					break;
				}
			}
		}else{printf("IK call unsuccessful\n");}

		// Change the yaw and the sampling direction
		YAW_DELTA = YAW_DELTA + YAW_SAMPLING_STEP;
		YAW_SAMPLING_DIRECTION = -1 * YAW_SAMPLING_DIRECTION;

		if(YAW_DELTA > MAX_YAW_DELTA)
		{
			// Re-initialize the YAW_DELTA to zero again
			YAW_DELTA = 0.0;

			// Define new height delta
			SPINDLE_REQUIRED = true;
			SPINDLE_DELTA = SPINDLE_DELTA + SPINDLE_SAMPLING_STEP;
			if( (spindle_position - SPINDLE_DELTA) < SPINDLE_MIN )
			{
				HEIGHT_SAMPLING_DIRECTION = 1;
			}else if( (spindle_position + SPINDLE_DELTA) > SPINDLE_MAX )
			{
				HEIGHT_SAMPLING_DIRECTION = -1;
			}else
			{
				HEIGHT_SAMPLING_DIRECTION = -1 * HEIGHT_SAMPLING_DIRECTION;
			}
			HEIGHT_DELTA = HEIGHT_SAMPLING_DIRECTION * SPINDLE_DELTA;
			if( (spindle_position - SPINDLE_DELTA) < SPINDLE_MIN &&  (spindle_position + SPINDLE_DELTA) > SPINDLE_MAX )
			{
				printf("Sampling boundaries reached\n");
				SAMPLING_BOUNDARIES_REACHED = true;
			}
		}
	}

	if(GRASP_FEASIBLE)
	{
		if(SPINDLE_REQUIRED)
		{
			printf("Spindle required\n");
			amigo_actions::AmigoSpindleCommandGoal spgoal;
            spgoal.spindle_height = spindle_position + HEIGHT_DELTA;
            sc->sendGoal(spgoal);
            sc->waitForResult();
    		if (sc->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    		{
    			printf("Spindle failed\n");
    			as->setAborted();
    		}
		}

		int num_attempts = 0;
		bool GRASP_SUCCESS = false;
		if(goal->PERFORM_PRE_GRASP)
		{
			// Execute pre-grasp goal
			position_constraint.position.x = new_pre_grasp_pose.getOrigin().getX();
			position_constraint.position.y = new_pre_grasp_pose.getOrigin().getY();
			position_constraint.position.z = new_pre_grasp_pose.getOrigin().getZ();
			magoal.motion_plan_request.goal_constraints.position_constraints.at(0) = position_constraint;

			orientation_constraint.orientation.x = new_pre_grasp_pose.getRotation().getX();
			orientation_constraint.orientation.y = new_pre_grasp_pose.getRotation().getY();
			orientation_constraint.orientation.z = new_pre_grasp_pose.getRotation().getZ();
			orientation_constraint.orientation.w = new_pre_grasp_pose.getRotation().getW();
			magoal.motion_plan_request.goal_constraints.orientation_constraints.at(0) = orientation_constraint;

			while(ros::ok() && !GRASP_SUCCESS && num_attempts < MAX_RESEND_ATTEMPTS)
			{
				num_attempts++;
				ac->sendGoal(magoal);
				ac->waitForResult();
				if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					printf("Pre-grasp succeeded\n");
					GRASP_SUCCESS = true;
				}
				else
				{
					printf("Pre-grasp failed, resending MAX_RESEND_ATTEMPTS\n");
					sleep(2);
				}
			}
			if(!GRASP_SUCCESS){
				printf("Pre-grasp failed after MAX_RESEND_ATTEMPTS");
			}
		}

		if(goal->PERFORM_PRE_GRASP == GRASP_SUCCESS)
		{
			// Execute grasp goal
			position_constraint.position.x = new_grasp_pose.getOrigin().getX();
			position_constraint.position.y = new_grasp_pose.getOrigin().getY();
			position_constraint.position.z = new_grasp_pose.getOrigin().getZ();
			magoal.motion_plan_request.goal_constraints.position_constraints.at(0) = position_constraint;

			orientation_constraint.orientation.x = new_grasp_pose.getRotation().getX();
			orientation_constraint.orientation.y = new_grasp_pose.getRotation().getY();
			orientation_constraint.orientation.z = new_grasp_pose.getRotation().getZ();
			orientation_constraint.orientation.w = new_grasp_pose.getRotation().getW();
			magoal.motion_plan_request.goal_constraints.orientation_constraints.at(0) = orientation_constraint;

			num_attempts = 0;
			GRASP_SUCCESS = false;
			while(ros::ok() && !GRASP_SUCCESS && num_attempts < MAX_RESEND_ATTEMPTS)
			{
				num_attempts++;
				ac->sendGoal(magoal);
				ac->waitForResult();
				if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					printf("Grasp succeeded\n");
					GRASP_SUCCESS = true;
					as->setSucceeded();
				}
				else
				{
					printf("Grasp failed, resending MAX_RESEND_ATTEMPTS\n");
					sleep(2);
				}
			}
			if(!GRASP_SUCCESS){
				printf("Grasp failed after MAX_RESEND_ATTEMPTS");
				as->setAborted();
			}
		}else{
			as->setAborted();
		}
	}else
	{
		printf("Grasp not feasible\n");
		as->setAborted();
	}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_precompute_server");
  ros::NodeHandle n("~");

  // Get the parameters
  n.param<string>("side", SIDE, "left"); //determine for which side this node operates
  n.param("max_yaw_delta", MAX_YAW_DELTA, 2.0); // maximum sampling offset from desired yaw [rad]
  n.param("yaw_sampling_step", YAW_SAMPLING_STEP, 0.2); // step-size for yaw sampling [rad]
  n.param("pre_grasp_delta", PRE_GRASP_DELTA, 0.05); // offset for pre-grasping in cartesian x-direction [m]
  n.param("spindle_min", SPINDLE_MIN, 0.0); // Spindle minimum [m]
  n.param("spindle_max", SPINDLE_MAX, 0.4); // Spindle maximum [m]
  n.param("spindle_sampling_step", SPINDLE_SAMPLING_STEP, 0.02); // step-size for spindle sampling [m]
  n.param("max_resend_attempts", MAX_RESEND_ATTEMPTS, 5); // maximum move_arm resend attempts

  // Wait for the move arm server
  Client client("move_" + SIDE + "_arm", true);
  client.waitForServer();

  // Wait for the spindle server
  SpindleClient spindleclient("spindle_server", true);
  spindleclient.waitForServer();

  // Initialize the grasp_precompute server
  Server server(n, "/grasp_precompute_" + SIDE, boost::bind(&execute, _1, &server, &client, &spindleclient), false);
  server.start();

  // Initialize the IK clients
  query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/get_ik_solver_info");
  ik_client = n.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("/get_constraint_aware_ik");
  set_planning_scene_diff_client = n.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  // Start listening to the current joint measurements
  ros::Subscriber armsub = n.subscribe("/joint_measurements", 1, armcontrollerCB);

  // Start listening to the current spindle measurement
  ros::Subscriber spindlesub = n.subscribe("/spindle_measurement", 1, spindlecontrollerCB);
  spindlepub = new ros::Publisher(n.advertise<amigo_msgs::spindle_setpoint>("/spindle_reference", 1));

  ros::spin();

  return 0;
}
