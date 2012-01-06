#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  
  ROS_INFO("Test for move right arm server");

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  arm_navigation_msgs::MoveArmGoal goalB;
  std::vector<std::string> names(7);
  names[0] = "shoulder_yaw_joint_right";
  names[1] = "shoulder_pitch_joint_right";
  names[2] = "shoulder_roll_joint_right";
  names[3] = "elbow_pitch_joint_right";
  names[4] = "elbow_roll_joint_right";
  names[5] = "wrist_pitch_joint_right";
  names[6] = "wrist_yaw_joint_right";

  goalB.motion_plan_request.group_name = "right_arm";
  goalB.motion_plan_request.num_planning_attempts = 1;
  goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goalB.motion_plan_request.planner_id= std::string("");
  goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

  for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
  {
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
  }
    
  goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -0.0;
  goalB.motion_plan_request.goal_constraints.joint_constraints[3].position = -0.0;
  goalB.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.0;
   
  if (nh.ok())
  {
    bool finished_within_time = false;
    move_arm.sendGoal(goalB);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}
