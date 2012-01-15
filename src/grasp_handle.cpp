#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

#include <sensor_msgs/PointCloud2.h>

//actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmActionConstPtr> *move_arm;

class graspHandle{

};

/*void callback(const sensor_msgs::PointCloud2& cloud)
{
	ROS_WARN("JOOW!");

	//actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_left_arm",true);
	/*arm_navigation_msgs::MoveArmGoal goalA;

	  move_arm->waitForServer();
	  ROS_INFO("Connected to server");

	  goalA.motion_plan_request.group_name = "right_arm";
	  goalA.motion_plan_request.num_planning_attempts = 1;
	  goalA.motion_plan_request.planner_id = std::string("");
	  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	  goalA.motion_plan_request.allowed_planning_time = ros::Duration(20.0);

  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "grippoint_left";

  desired_pose.pose.position.x = 0.38;
  desired_pose.pose.position.y = 0.20;
  desired_pose.pose.position.z = 0.30;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

    bool finished_within_time = false;
    move_arm->sendGoal(goalA);
    finished_within_time = move_arm->waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm->cancelGoal();
      ROS_INFO("Timed out reaching handle");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm->getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }

    ros::shutdown();
}*/

int main(int argc, char **argv){
  ros::init (argc, argv, "grasp_handle");
  ros::NodeHandle nh;

  graspHandle;

  ros::Subscriber sub = nh.subscribe("/handle_detector/handle_projected_inliers/output", 1, callback);
  
  ros::spin();

}
