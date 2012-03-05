// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>

#include <amigo_msgs/arm_joints.h>

using namespace std;

typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> Client;

amigo_msgs::arm_joints arm_joints;

bool JOINTS_RECEIVED = false;

void controllerCB(const amigo_msgs::arm_jointsConstPtr &joint_meas)
{
	JOINTS_RECEIVED = true;
	arm_joints = *joint_meas;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_move_arm");
  ros::NodeHandle nh;
  Client client("move_left_arm", true);
  client.waitForServer();
  arm_navigation_msgs::MoveArmGoal goal;

  // Create ros subscriber
  ros::Subscriber sub = nh.subscribe("/arm_left_controller/joint_measurements", 1, controllerCB);

  // Get current joint coordinates
  ros::Rate r = 10;
  while (ros::ok() && !JOINTS_RECEIVED)
  {
	  ros::spinOnce();
	  r.sleep();
	  if(!JOINTS_RECEIVED){ROS_WARN("No joint measurement received");}
  }

  // Get Kinematic solver info
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  ros::ServiceClient query_client = nh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/amigo_left_arm_kinematics/get_ik_solver_info");
  if(query_client.call(request,response))
  {
	  printf("IK test call succesfull\n");
    for(unsigned int i=0; i<response.kinematic_solver_info.joint_names.size(); i++)
    {
      //ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      goal.planning_scene_diff.robot_state.joint_state.name.push_back(response.kinematic_solver_info.joint_names[i]);
      goal.planning_scene_diff.robot_state.joint_state.position.push_back(max(min(arm_joints.pos[i].data,response.kinematic_solver_info.limits[i].max_position),response.kinematic_solver_info.limits[i].min_position));
      printf("%s: value=%f min=%f max=%f final=%f\n",response.kinematic_solver_info.joint_names[i].c_str(),arm_joints.pos[i].data,
    		  response.kinematic_solver_info.limits[i].min_position,response.kinematic_solver_info.limits[i].max_position,goal.planning_scene_diff.robot_state.joint_state.position[i]);
    }
  }else
  {
	  printf("IK test call unsuccesfull\n");
  }

  // Define goal
  goal.motion_plan_request.group_name = "left_arm";
  goal.motion_plan_request.num_planning_attempts = 5;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
  goal.planner_service_name = "ompl_planning/plan_kinematic_path";

  arm_navigation_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = "base_link";
  position_constraint.link_name = "grippoint_left";
  position_constraint.position.x = 0.5;
  position_constraint.position.y = 0.08;
  position_constraint.position.z = 0.8;
  position_constraint.constraint_region_shape.type = 0;
  position_constraint.constraint_region_shape.dimensions.push_back(0.02);
  position_constraint.weight = 1.0;
  goal.motion_plan_request.goal_constraints.position_constraints.push_back(position_constraint);

  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = "base_link";
  orientation_constraint.link_name = "grippoint_left";
  orientation_constraint.orientation.x = 0.0;
  orientation_constraint.orientation.y = 0.0;
  orientation_constraint.orientation.z = 0.0;
  orientation_constraint.orientation.w = 1.0;
  orientation_constraint.absolute_roll_tolerance = 0.1;
  orientation_constraint.absolute_pitch_tolerance = 0.1;
  orientation_constraint.absolute_yaw_tolerance = 0.1;
  orientation_constraint.weight = 1.0;
  goal.motion_plan_request.goal_constraints.orientation_constraints.push_back(orientation_constraint);

  client.sendGoal(goal);

  client.waitForResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Move arm successful\n");
  else
	  printf("Move arm unsuccesfull\n");


  return 0;
}
