#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_navigation_msgs/MoveArmAction.h>

typedef actionlib::SimpleActionServer<arm_navigation_msgs::MoveArmAction> Server;

void callback(const arm_navigation_msgs::MoveArmGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pre_grasp_server");
  ros::NodeHandle n;
  Server server(n, "/pre_grasp_server", boost::bind(&callback, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
