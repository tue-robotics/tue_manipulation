#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "dummy_joint_publisher");
	ros::NodeHandle n("~");

	int num_joints = 7;

	if (argc != (num_joints + 1)) {
		ROS_ERROR("Args: POS_1 ... POS_7");
		return -1;
	}

	sensor_msgs::JointState left_msg;
	left_msg.name.push_back("shoulder_yaw_joint_left");
    left_msg.name.push_back("shoulder_pitch_joint_left");
	left_msg.name.push_back("shoulder_roll_joint_left");
    left_msg.name.push_back("elbow_pitch_joint_left");
    left_msg.name.push_back("elbow_roll_joint_left");
    left_msg.name.push_back("wrist_pitch_joint_left");
    left_msg.name.push_back("wrist_yaw_joint_left");
	
	sensor_msgs::JointState right_msg;
	right_msg.name.push_back("shoulder_yaw_joint_right");
    right_msg.name.push_back("shoulder_pitch_joint_right");
	right_msg.name.push_back("shoulder_roll_joint_right");
    right_msg.name.push_back("elbow_pitch_joint_right");
    right_msg.name.push_back("elbow_roll_joint_right");
    right_msg.name.push_back("wrist_pitch_joint_right");
    right_msg.name.push_back("wrist_yaw_joint_right");

	//joints.time.data =  atof(argv[2]);
    
	int i_arg = 1;
	for(int i = 0; i < num_joints; ++i) {
		left_msg.position.push_back(atof(argv[i+1]));
		right_msg.position.push_back(atof(argv[i+1]));
	}

	// Publisher
	ros::Publisher pub_left  = n.advertise<sensor_msgs::JointState>("/amigo/left_arm/references", 1);
    ros::Publisher pub_right = n.advertise<sensor_msgs::JointState>("/amigo/right_arm/references", 1);
    
	// Loop 20 Hz
	int rate = 1;
	ros::Rate r(rate);

	while(n.ok()) {
		pub_left.publish(left_msg);
		pub_right.publish(right_msg);
		r.sleep();
	}
	//ros::spin();

}

