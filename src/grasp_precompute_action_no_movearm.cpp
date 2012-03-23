// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <amigo_actions/AmigoSpindleCommandAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <amigo_msgs/spindle_setpoint.h>

#include <std_msgs/Float64.h>

#include <tf/tf.h>

using namespace std;

double MAX_YAW_DELTA,YAW_SAMPLING_STEP,PRE_GRASP_DELTA,SPINDLE_MIN,SPINDLE_MAX,SPINDLE_SAMPLING_STEP;
string SIDE;

typedef actionlib::SimpleActionServer<amigo_arm_navigation::grasp_precomputeAction> Server;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
typedef actionlib::SimpleActionClient<amigo_actions::AmigoSpindleCommandAction> SpindleClient;

ros::Publisher *spindlepub;

ros::ServiceClient query_client;
ros::ServiceClient ik_client;

double spindle_position;



void spindlecontrollerCB(const std_msgs::Float64ConstPtr &spindle_meas)
{
	spindle_position = spindle_meas->data;
}

// This callback is executed when a new goal is send to this node
void execute(const amigo_arm_navigation::grasp_precomputeGoalConstPtr& goal, Server* as, Client* jta, SpindleClient* sc)
{
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

	// Define joint trajectory action goal
	control_msgs::FollowJointTrajectoryGoal jtagoal;

	//Call IK
	kinematics_msgs::GetPositionIK::Request  gpik_req;
	kinematics_msgs::GetPositionIK::Response gpik_res;
	arm_navigation_msgs::RobotState grasp_solution;
	arm_navigation_msgs::RobotState pre_grasp_solution;

	gpik_req.timeout = ros::Duration(5.0);
	gpik_req.ik_request.ik_link_name = "grippoint_" + SIDE;
    gpik_req.ik_request.pose_stamped.header.frame_id = goal->goal.header.frame_id;

    // Define joint names and seed positions
	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	{
		double joint_seed = (response.kinematic_solver_info.limits[i].max_position + response.kinematic_solver_info.limits[i].min_position)/2.0;
		string joint_name = response.kinematic_solver_info.joint_names[i];
		gpik_req.ik_request.ik_seed_state.joint_state.name.push_back(joint_name);
		gpik_req.ik_request.ik_seed_state.joint_state.position.push_back(joint_seed);
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
		//printf("YAW_DELTA=%f \t HEIGHT_DELTA=%f\n",YAW_SAMPLING_DIRECTION * YAW_DELTA,HEIGHT_DELTA);
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
				//printf("Grasp_pose feasible\n");

				grasp_solution = gpik_res.solution;

				if(goal->PERFORM_PRE_GRASP)
				{
					tf::poseTFToMsg(new_pre_grasp_pose, gpik_req.ik_request.pose_stamped.pose);
					if(ik_client.call(gpik_req, gpik_res))
					{
						if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
						{
							//printf("Pre_grasp_pose feasible\n");
							pre_grasp_solution = gpik_res.solution;
							GRASP_FEASIBLE = true;
							break;
						}
					}else{
						ROS_ERROR("IK call unsuccessful");
						ros::shutdown();
						exit(-1);}
				}
				else
				{
					GRASP_FEASIBLE = true;
					break;
				}
			}
		}else{
			ROS_ERROR("IK call unsuccessful");
			ros::shutdown();
			exit(-1);}

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
				ROS_WARN("Sampling boundaries reached. No feasible sample found\n");
				SAMPLING_BOUNDARIES_REACHED = true;
			}
		}
	}

	if(GRASP_FEASIBLE)
	{
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
    			as->setAborted();
    		}
		}

		bool GRASP_SUCCESS = false;
		if(goal->PERFORM_PRE_GRASP)
		{
			// Execute pre-grasp goal i.e send the obtained solution to the joint_action_node
			jtagoal.trajectory.points.resize(1);
			for(unsigned int i=0;i<response.kinematic_solver_info.joint_names.size();++i)
			{
				jtagoal.trajectory.joint_names.push_back(pre_grasp_solution.joint_state.name[i]);
				jtagoal.trajectory.points[0].positions.push_back(pre_grasp_solution.joint_state.position[i]);
				jtagoal.trajectory.points[0].velocities.push_back(pre_grasp_solution.joint_state.velocity[i]);
			}

			jta->sendGoal(jtagoal);
			jta->waitForResult();
			if (jta->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Pre-grasp succeeded\n");
				GRASP_SUCCESS = true;
			}
			else
			{
				ROS_INFO("Pre-grasp sending to joint trajectory action failed \n");
			}
		}

		if(goal->PERFORM_PRE_GRASP == GRASP_SUCCESS)
		{
			GRASP_SUCCESS = false;
			// Execute grasp goal i.e send the obtained solution to the joint_action_node
			jtagoal.trajectory.points.resize(1);
			for(unsigned int i=0;i<response.kinematic_solver_info.joint_names.size();++i)
			{
				jtagoal.trajectory.joint_names.push_back(grasp_solution.joint_state.name[i]);
				jtagoal.trajectory.points[0].positions.push_back(grasp_solution.joint_state.position[i]);
				jtagoal.trajectory.points[0].velocities.push_back(grasp_solution.joint_state.velocity[i]);
			}

			jta->sendGoal(jtagoal);
			jta->waitForResult();
			if (jta->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Grasp succeeded\n");
				GRASP_SUCCESS = true;
				as->setSucceeded();
			}
			else
			{
				ROS_INFO("Grasp sending to joint trajectory action failed \n");
			}

			if(!GRASP_SUCCESS){
				ROS_INFO("Grasp failed");
				as->setAborted();
			}
		}else{
			as->setAborted();
		}
	}else
	{
		ROS_INFO("Grasp not feasible\n");
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

  // Wait for the joint_trajectory_action server
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
  ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>("/get_constraint_aware_ik");

  // Start listening to the current spindle measurement
  ros::Subscriber spindlesub = n.subscribe("/spindle_measurement", 1, spindlecontrollerCB);
  spindlepub = new ros::Publisher(n.advertise<amigo_msgs::spindle_setpoint>("/spindle_reference", 1));

  ros::spin();

  return 0;
}
