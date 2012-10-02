// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <amigo_actions/AmigoSpindleCommandAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>

#include <amigo_msgs/arm_joints.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

using namespace std;

double MAX_YAW_DELTA,YAW_SAMPLING_STEP,PRE_GRASP_DELTA,SPINDLE_MIN,SPINDLE_MAX,SPINDLE_SAMPLING_STEP;
string SIDE;

int NUM_GRASP_POINTS = 0, PRE_GRASP_INBETWEEN_SAMPLING_STEPS = 0;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> Server;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> Clientfake;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
typedef actionlib::SimpleActionClient<amigo_actions::AmigoSpindleCommandAction> SpindleClient;

ros::Publisher *IKpospub;

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
	// Check if desired coordinate frame is /base_link frame else bail because this is not yet implemented!
	// We still need to make a nice coordinate transformation here but this will raise quesitons about e.g. SnapMapICP
	if(goal->goal.header.frame_id.compare("/base_link")){
		ROS_WARN("Grasp not executed: desired base frame must be '/base_link'");
		as->setAborted();
		return;
	}

	// Test if the planning scene is up and running
	arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
	arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;

	if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
		ROS_WARN("Grasp not executed: cannot call planning scene");
	    as->setAborted();
		return;
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
		ROS_WARN("Grasp not executed: IK service cannot be called");
		as->setAborted();
		return;
	}

	// Define general joint trajectory action info
	control_msgs::FollowJointTrajectoryGoal jtagoal;

	// Define general visualization markers for feasible IK positions
	visualization_msgs::MarkerArray IKPosMarkerArray;

	// Define stringstream for IK pos ID
	ostringstream ss;

	//Call IK
	kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
	kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

	// Store the solutions
	arm_navigation_msgs::RobotState grasp_solution;
	vector<arm_navigation_msgs::RobotState> pre_grasp_solution;

	gpik_req.timeout = ros::Duration(5.0);
	gpik_req.ik_request.ik_link_name = "grippoint_" + SIDE;
        gpik_req.ik_request.pose_stamped.header.frame_id = goal->goal.header.frame_id;

        // Define joint names and seed positions
	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); ++i)
	{
		//double joint_seed = (response.kinematic_solver_info.limits[i].max_position + response.kinematic_solver_info.limits[i].min_position)/2.0;
		double joint_seed = arm_joints.pos[i].data;
		string joint_name = response.kinematic_solver_info.joint_names[i];
		gpik_req.ik_request.ik_seed_state.joint_state.name.push_back(joint_name);
		gpik_req.ik_request.ik_seed_state.joint_state.position.push_back(joint_seed);
		jtagoal.trajectory.joint_names.push_back(joint_name);
		//magoal.planning_scene_diff.robot_state.joint_state.name.push_back(joint_name);
		//magoal.planning_scene_diff.robot_state.joint_state.position.push_back(joint_seed);
	}

	// Check if a pre-grasp is required, resize Joint Trajectory Action
	if(!goal->PERFORM_PRE_GRASP)
	{
		NUM_GRASP_POINTS = 1;
	}
	else
	{
		NUM_GRASP_POINTS = PRE_GRASP_INBETWEEN_SAMPLING_STEPS+2; // Inbetween sample points + Pre-grasp + Grasp point
	}

	// Resize the Joint Trajectory Action goal accordingly to the number of grasp points
	jtagoal.trajectory.points.resize(NUM_GRASP_POINTS); // Number of sample points
	IKPosMarkerArray.markers.resize(NUM_GRASP_POINTS);
        pre_grasp_solution.resize(NUM_GRASP_POINTS);
	for(int i=0;i<(NUM_GRASP_POINTS);++i)
	{
		jtagoal.trajectory.points[i].positions.resize(response.kinematic_solver_info.joint_names.size());
		jtagoal.trajectory.points[i].velocities.resize(response.kinematic_solver_info.joint_names.size());
		jtagoal.trajectory.points[i].accelerations.resize(response.kinematic_solver_info.joint_names.size());

		IKPosMarkerArray.markers[i].type = 2; // 2=SPHERE
		IKPosMarkerArray.markers[i].scale.x = 0.01;
		IKPosMarkerArray.markers[i].scale.y = 0.01;
		IKPosMarkerArray.markers[i].scale.z = 0.01;
		IKPosMarkerArray.markers[i].color.r = 1.0f;
		IKPosMarkerArray.markers[i].color.g = 0.0f;
		IKPosMarkerArray.markers[i].color.b = 0.0f;
		IKPosMarkerArray.markers[i].color.a = 1.0;
		IKPosMarkerArray.markers[i].header.frame_id = goal->goal.header.frame_id;
	}

	///////////////////////////////////////////////////////////
	/// EVALUATE IF GRASP IS FEASIBLE AND CREATE JOINT ARRAY///
	///////////////////////////////////////////////////////////
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
		//printf("YAW_DELTA=%f \t HEIGHT_DELTA=%f\n",YAW_SAMPLING_DIRECTION * YAW_DELTA,HEIGHT_DELTA);
		// Define new_grasp_pose
		tf::Transform height_offset(tf::Quaternion(0,0,0,1),tf::Point(0,0,-HEIGHT_DELTA));
		tf::Transform yaw_offset(tf::createQuaternionFromYaw(YAW_SAMPLING_DIRECTION * YAW_DELTA),tf::Point(0,0,0));
		new_grasp_pose = grasp_pose * height_offset * yaw_offset;

		// Check if all grasp points are feasible
		GRASP_FEASIBLE = true;
		int k=0;
		for(int i=(NUM_GRASP_POINTS-1);i>=0;--i)
		{
			cout << i << " offset = " << -PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0) << endl;
			tf::Transform pre_grasp_offset(tf::Quaternion(0,0,0,1),tf::Point(-PRE_GRASP_DELTA*i/(PRE_GRASP_INBETWEEN_SAMPLING_STEPS+1.0),0,0));
			new_pre_grasp_pose = new_grasp_pose * pre_grasp_offset;
			tf::poseTFToMsg(new_pre_grasp_pose, gpik_req.ik_request.pose_stamped.pose);
			if(ik_client.call(gpik_req, gpik_res))
			{
				if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
				{
					//printf("Pre_grasp_pose feasible\n");
					pre_grasp_solution[k] = gpik_res.solution;
					++k;
					// Set the seed state for the next point to the solution of the previous point
					gpik_req.ik_request.ik_seed_state = gpik_res.solution;
					// Fill the IK markers
					IKPosMarkerArray.markers[i].id = i;
					tf::poseTFToMsg(grasp_pose * yaw_offset * pre_grasp_offset, IKPosMarkerArray.markers[i].pose);
				}else{
					GRASP_FEASIBLE = false;
					break;
				}
			}else{
				ROS_WARN("Grasp not executed: IK service cannot be called");
				as->setAborted();
				return;}
		}

		// If the grasp is not feasible, change the yaw and the sampling direction
		if(!GRASP_FEASIBLE)
		{
			ROS_INFO("Not all pre-grasp points feasible: resampling yaw");
			YAW_DELTA = YAW_DELTA + YAW_SAMPLING_STEP;
			YAW_SAMPLING_DIRECTION = -1 * YAW_SAMPLING_DIRECTION;

			if(YAW_DELTA > MAX_YAW_DELTA)
			{
				ROS_INFO("Not all pre-grasp points feasible over yaw range: resampling spindle");
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
					ROS_WARN("Sampling boundaries reached. No feasible sample found\n");
					SAMPLING_BOUNDARIES_REACHED = true;
					break;
				}
			}
		}
	}

	//////////////////////////////////////////////////////////
	//////////////// EXECUTING JOINT ARRAY ///////////////////
	//////////////////////////////////////////////////////////
	// SetAborted when grasp not feasible, else start actuating the robot when all grasp points are feasible
	bool GRASP_SUCCESS = true;
	if(!GRASP_FEASIBLE)
	{
		ROS_INFO("Grasp not feasible\n");
		GRASP_SUCCESS = false;
	}
	else
	{
		ROS_INFO("Solution found: moving arms");
		// Publish the IK markers
		IKpospub->publish(IKPosMarkerArray);
		// Check if spindle is required
		if(SPINDLE_REQUIRED)
		{
			ROS_INFO("Spindle going to be activated\n");
			amigo_actions::AmigoSpindleCommandGoal spgoal;
			spgoal.spindle_height = spindle_position + HEIGHT_DELTA;
			sc->sendGoal(spgoal);
			sc->waitForResult();
			if (sc->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_WARN("Spindle failed\n");
				GRASP_SUCCESS = false;
			}
		}

		// Continue if GRASP_SUCCESS is still true
		if(GRASP_SUCCESS)
		{
			// Fill in all the solutions to the JTA goal
			for(int j=0;j<NUM_GRASP_POINTS;++j)
			{
				for(unsigned int i=0;i<response.kinematic_solver_info.joint_names.size();++i)
				{
					jtagoal.trajectory.points[j].positions[i] = pre_grasp_solution[j].joint_state.position[i];
					//jtagoal.trajectory.points[j].positions[i]   = pre_grasp_solution[j].joint_state.position[i];
					//jtagoal.trajectory.points[0].velocities[i] = pre_grasp_solution.joint_state.velocity[i];
					//jtagoal.trajectory.points[0].accelerations[i] = 0.0;
				}
			}
			ac->sendGoal(jtagoal);
			ac->waitForResult();
			if (ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Execution of all grasp points succeeded\n");
				GRASP_SUCCESS = false;
			}
		}
	}

	//////////////////////////////////////////////////////////
	///////////////// FEEDBACK TO CLIENT /////////////////////
	//////////////////////////////////////////////////////////
	if(GRASP_SUCCESS)
	{
		as->setSucceeded();
	}else
	{
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
  n.param("pre_grasp_inbetween_sampling_steps", PRE_GRASP_INBETWEEN_SAMPLING_STEPS, 0); // offset for pre-grasping in cartesian x-direction [m]
  n.param("spindle_min", SPINDLE_MIN, 0.0); // Spindle minimum [m]
  n.param("spindle_max", SPINDLE_MAX, 0.4); // Spindle maximum [m]
  n.param("spindle_sampling_step", SPINDLE_SAMPLING_STEP, 0.02); // step-size for spindle sampling [m]

  // Wait for the joint trajectory action server
  Client client("joint_trajectory_action_" + SIDE, true);
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

  // IK marker publisher
  IKpospub = new ros::Publisher(n.advertise<visualization_msgs::MarkerArray>("/IK_Position_Markers", 1));

  ros::spin();

  return 0;
}
