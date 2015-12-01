// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tue_manipulation_msgs/GraspPrecomputeAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<tue_manipulation_msgs::GraspPrecomputeAction> Client;

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  if (argc < 3) {
      std::cout << "Usage: 'rosrun tue_manipulation test_grasp_precompute [robot_name] [left|right]', with robot name either amigo or sergio" << std::endl;
      return 1;
  }
  std::string side(argv[2]);
  std::string robot_name(argv[1]);

  ros::init(argc, argv, "test_grasp");
  std::string actionlibname = robot_name + "/" + side + "_arm/grasp_precompute";
  Client client( actionlibname ); // true -> don't need ros::spin()
  boost::thread spin_thread(&spinThread);
  printf("Looking for Actionlib server\n%s", actionlibname.c_str());
  client.waitForServer(ros::Duration(2,0));
  if (!client.isServerConnected()) {
    printf(" --> Not found\n");
    return 1;
  }

  printf(" --> Found\n");
  tue_manipulation_msgs::GraspPrecomputeGoal goal;

  ros::Time::waitForValid(ros::WallDuration(2));

  goal.goal.header.stamp=ros::Time::now();
  goal.goal.header.frame_id = "/"+robot_name+"/base_link";
  goal.PERFORM_PRE_GRASP = true;
  goal.goal.x = 0.5;
  goal.goal.y = 0.0;
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
