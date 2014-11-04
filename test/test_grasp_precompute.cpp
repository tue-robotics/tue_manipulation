// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tue_manipulation/GraspPrecomputeAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<tue_manipulation::GraspPrecomputeAction> Client;

int main(int argc, char** argv)
{
  if (argc < 2) {
      std::cout << "Usage: 'rosrun tue_manipulation test_grasp_precompute [robot_name]', with robot name either amigo or sergio" << std::endl;
      return 1;
  }
  std::string robot_name(argv[1]);
    
  ros::init(argc, argv, "test_grasp");
  Client client("grasp_precompute_left", true); // true -> don't need ros::spin()
  client.waitForServer();
  tue_manipulation::GraspPrecomputeGoal goal;

  ros::Time::waitForValid(ros::WallDuration(2));

  goal.goal.header.stamp=ros::Time::now();
  goal.goal.header.frame_id = "/"+robot_name+"/base_link";
  goal.PERFORM_PRE_GRASP = true;
  goal.goal.x = 0.5;
  goal.goal.y = 0.2;
  goal.goal.z = 0.9;

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
