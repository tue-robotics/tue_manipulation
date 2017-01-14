// Author: Tim Clephas

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

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
      std::cout << "Usage: 'rosrun tue_manipulation test_moveit [robot_name] [left|right] __ns:=[robot_name]', with robot name either amigo or sergio" << std::endl;
      return 1;
  }
  std::string side(argv[2]);
  std::string robot_name(argv[1]);

  ros::init(argc, argv, "random_move");
  ros::NodeHandle nh;

  /// MoveIt
  ROS_INFO("Step 1");
  moveit::planning_interface::MoveGroupInterface::Options options(side+"_arm", "/amigo/robot_description", nh);

  ROS_INFO("Step 2");
  moveit::planning_interface::MoveGroupInterface group(options);
  ROS_INFO("Step 3");

  geometry_msgs::PoseStamped target_pose = group.getRandomPose();
  /*target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;*/

  ROS_INFO("Step 4");
  group.setPoseTarget(target_pose);

  ROS_INFO("Step 5");
  group.move();

  ROS_INFO("Step 6");

  //moveit::planning_interface::MoveGroupInterface group(side + "_arm");

  //group->setPoseTarget(group->getRandomPose());
  //moveit::core::RobotState::setToRandomPositions(group)	

  ROS_INFO("Step last");
  return 0;
}
