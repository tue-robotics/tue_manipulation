#include <ros/ros.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <amigo_msgs/arm_joints.h>

std::vector<double> arm_joints;
bool new_cmd;

void validityCallback(const amigo_msgs::arm_joints::ConstPtr& msg) {
	
	for (int i=0;i<7;i++)
	  arm_joints.push_back(msg->pos[i].data);
	
	new_cmd = true;
}

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::service::waitForService("environment_server_right_arm/get_state_validity");
  ros::ServiceClient check_state_validity_client_ = rh.serviceClient<planning_environment_msgs::GetStateValidity>("environment_server_right_arm/get_state_validity");
  ros::Subscriber check_state_validity_server_ = rh.subscribe("arm_validity",1,validityCallback);

  planning_environment_msgs::GetStateValidity::Request req;
  planning_environment_msgs::GetStateValidity::Response res;

  req.robot_state.joint_state.name.push_back("shoulder_yaw_right");
  req.robot_state.joint_state.name.push_back("shoulder_pitch_right");
  req.robot_state.joint_state.name.push_back("shoulder_roll_right");
  req.robot_state.joint_state.name.push_back("elbow_pitch_right");
  req.robot_state.joint_state.name.push_back("elbow_roll_right");
  req.robot_state.joint_state.name.push_back("wrist_pitch_right");
  req.robot_state.joint_state.name.push_back("wrist_yaw_right");
  req.robot_state.joint_state.position.resize(7,0.0);


  new_cmd = false;	
  ros::Rate rate(2.0);

  while (rh.ok()){
	  if (new_cmd){
		  
		//these set whatever non-zero joint angle values are desired
		for (int i = 0; i < 7 ; i++)
		  req.robot_state.joint_state.position[i] = arm_joints[i];
		
		req.robot_state.joint_state.header.stamp = ros::Time::now();
		req.check_collisions = true;
		if(check_state_validity_client_.call(req,res))
		{
			if(res.error_code.val == res.error_code.SUCCESS)
				ROS_INFO("Requested state is not in collision");
			else
				ROS_INFO("Requested state is in collision. Error code: %d",res.error_code.val);
		}
		else
		{
			ROS_ERROR("Service call to check state validity failed %s",check_state_validity_client_.getService().c_str());
			return false;
		}
		
	  arm_joints.clear();
	  new_cmd = false;
	  }
		
      ros::spinOnce();
	  rate.sleep();

  }
  ros::shutdown();
}
