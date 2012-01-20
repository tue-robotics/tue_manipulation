#include "ros/ros.h"

#include "amigo_msgs/arm_joints.h"
#include "std_msgs/Float64.h"

//std_msgs/Float64 time
//std_msgs/Float64[7] pos
//std_msgs/Float64[7] vel
//std_msgs/Float64[7] acc

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

	amigo_msgs::arm_joints joints;

	//joints.time.data =  atof(argv[2]);
    joints.time.data =  0.0;
    
	int i_arg = 1;
	for(int i = 0; i < num_joints; ++i) {
		std_msgs::Float64 f;
		f.data = atof(argv[i_arg++]);
		joints.pos[i] = f;
	}

	for(int i = 0; i < num_joints; ++i) {
		std_msgs::Float64 f;
		//f.data = atof(argv[i_arg++]);
		f.data = 0.0;
		joints.vel[i] = f;
	}

	for(int i = 0; i < num_joints; ++i) {
		std_msgs::Float64 f;
		//f.data = atof(argv[i_arg++]);
		f.data = 0.0;
		joints.acc[i] = f;
	}

	// Publisher
	ros::Publisher pub_left  = n.advertise<amigo_msgs::arm_joints>("/arm_left_controller/joint_coordinates", 100);
    ros::Publisher pub_right = n.advertise<amigo_msgs::arm_joints>("/arm_right_controller/joint_coordinates", 100);
    
	// Loop 20 Hz
	int rate = 1;
	ros::Rate r(rate);

	while(n.ok()) {
		pub_left.publish(joints);
		pub_right.publish(joints);
		r.sleep();
	}
	//ros::spin();

}

