#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_client");

  if (argc != 2) {
      ROS_ERROR("Arguments: SPINDLE_HEIGHT. Please note that this test client has been developed for AMIGO");
	  return 1;
  }

  double spindle_height = atof(argv[1]);

  // create the action client
  //actionlib::SimpleActionClient<amigo_actions::AmigoSpindleCommandAction> ac("spindle_server");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("torso_server");
  ac.waitForServer();

  // send a goal to the action
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.header.stamp = ros::Time::now();
  goal.trajectory.joint_names.resize(1);
  goal.trajectory.joint_names[0] = "torso_joint";
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(1);
  goal.trajectory.points[0].positions[0] = spindle_height;


  actionlib::SimpleClientGoalState state = ac.sendGoalAndWait(goal);
  ROS_WARN("Current state: %s", state.toString().c_str());

  ros::shutdown();

  return 0;
}
