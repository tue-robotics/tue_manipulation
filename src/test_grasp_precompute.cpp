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

  ros::Time::waitForValid(ros::WallDuration(2));

  goal.goal.header.stamp=ros::Time::now();
  goal.goal.header.frame_id = "base_link";
  goal.goal.x = 0.5;
  goal.goal.y = 0.3;
  goal.goal.z = 1.15;

  goal.goal.roll  = 0;
  goal.goal.pitch = 0;
  goal.goal.yaw   = 0;

  client.sendGoal(goal);
  client.waitForResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Grasp precompute successful\n");
  else
	  printf("Grasp precompute unsuccesfull\n");

  return 0;
}