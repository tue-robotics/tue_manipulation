#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl/jntarray.hpp>
#include <amigo_msgs/arm_joints.h>


bool new_traj = false;
std::vector<double> time_vector;
std::vector<KDL::JntArray> joint_matrix;
trajectory_msgs::JointTrajectory traj;
bool cancel = false;

void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{   
   
   traj = *msg;
   
   if (traj.points.size() == 0){
     ROS_WARN("Amigo trajectory publisher: Joint trajectory cancelled"); 
     cancel = true;
   }
   else{
     cancel = false;
     ROS_INFO("Amigo trajectory publisher: Joint trajectory received");
   }
  
   new_traj = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "joint_trajectory_publisher");

  ros::NodeHandle n;
 
  ros::Publisher joints_pub = n.advertise<amigo_msgs::arm_joints>("joint_coordinates", 50);
  ros::Subscriber jointstate_sub = n.subscribe("controller_command", 5, trajectoryCallback);
  
  double time_start;
  amigo_msgs::arm_joints joint_msg;
  
  ros::Rate rate(5);
  
  
  
  while (n.ok()){


    //if new trajectory received
	if (new_traj){
	  time_start = ros::Time::now().toSec();
	  
      int num_points = traj.points.size();
      int num_joints = traj.joint_names.size();
      
      for (int i = 0; i < num_points ; i++){

		//spin to get possible cancel request
		ros::spinOnce();
		
		//if cancelled, do not publish  
		if (cancel){
		  break;
		}  
		
		//get time to publish 	
		double traj_time = traj.points[i].time_from_start.toSec();
		
		//publish at right time
		while ( (ros::Time::now().toSec() - time_start ) < traj_time && n.ok()){
			//wait here
		}
		
		for (int j =0; j < num_joints ; j++){
			joint_msg.pos[j].data = traj.points[i].positions[j];
			
		}
		//publish
		joints_pub.publish(joint_msg);
		ROS_DEBUG("Published point %d!",i);
		
	  }
		
	  ROS_INFO("Finished publishing trajectory");   
	  new_traj = false;   

	   
	}  
		
    ros::spinOnce();
	rate.sleep();

    
  } 
  
}
