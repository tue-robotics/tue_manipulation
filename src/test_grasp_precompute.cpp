// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <amigo_arm_navigation/grasp_precomputeAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<amigo_arm_navigation::grasp_precomputeAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_grasp");
  Client client("grasp_precompute", true); // true -> don't need ros::spin()
  client.waitForServer();
  amigo_arm_navigation::grasp_precomputeGoal goal;

  // Define goal
  goal.goal.header.frame_id = "base_link";
  goal.goal.pose.position.x = 0.5;
  goal.goal.pose.position.y = 0.1;
  goal.goal.pose.position.z = 0.8;

  goal.goal.pose.orientation.x = 0;
  goal.goal.pose.orientation.y = 0;
  goal.goal.pose.orientation.z = 0;
  goal.goal.pose.orientation.w = 1;

  // Fill in goal here
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Grasp precompute successful\n");
  else
	  printf("Grasp precompute unsuccesfull\n");


  return 0;
}
