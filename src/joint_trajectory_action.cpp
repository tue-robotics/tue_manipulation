#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <amigo_msgs/tip_ref.h>
#include <geometry_msgs/Quaternion.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <tf/transform_datatypes.h>


const double DEFAULT_GOAL_THRESHOLD = 0.1;

class JointTrajectoryExecuter
{
private:

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
  
public:

  JointTrajectoryExecuter(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, ros::this_node::getName(),
                   boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                   false),
    has_active_goal_(false), goal_nr(0), fk_computed(false)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!pn.getParam("joints", joint_names))
    {
      ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
      exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
      exit(1);
    }
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                  pn.getNamespace().c_str());
        exit(1);
      }

      joint_names_.push_back((std::string)name_value);
    }

	if(!pn.getParam("tol_x",tol_x))
	  ROS_ERROR("Load x tolerance parameters on parameter server");
	if(!pn.getParam("tol_y",tol_y))
	  ROS_ERROR("Load y tolerance parameters on parameter server");
	if(!pn.getParam("tol_z",tol_z))
	  ROS_ERROR("Load z tolerance parameters on parameter server");
	if(!pn.getParam("tol_roll",tol_roll))
	  ROS_ERROR("Load roll tolerance parameters on parameter server");
	if(!pn.getParam("tol_pitch",tol_pitch))
	  ROS_ERROR("Load pitch tolerance parameters on parameter server");
	if(!pn.getParam("tol_yaw",tol_yaw))
	  ROS_ERROR("Load yaw tolerance parameters on parameter server");
	if(!pn.getParam("root_frame",root_frame))
	  ROS_ERROR("Load root_frame on parameter server");
	if(!pn.getParam("tip_frame",tip_frame))
	  ROS_ERROR("Load tip_frame on parameter server");
	  
	
    //set topics/srvs
    pub_controller_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("controller_command", 1);
    tip_sub = node_.subscribe("measured_tip_position", 1, &JointTrajectoryExecuter::getTipPosition, this);
    fk_client = node_.serviceClient<kinematics_msgs::GetPositionFK>("get_fk");

    action_server_.start();
    

    
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.shutdown();
  }

private:
  
  
  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }


  void goalCB(GoalHandle gh)
  { goal_nr++;
	double now = ros::Time::now().toSec();  

	ROS_INFO("Amigo_joint_trajectory action: Received goal number %d at time %f",goal_nr, now);
    // Ensures that the joints in the goal match the joints we are commanding.
	ROS_INFO_STREAM("number of joints is  " << joint_names_.size());
	for (uint i = 0; i < joint_names_.size(); ++i){
	ROS_INFO_STREAM("The joint names are " << joint_names_[i]);
	}

	ROS_INFO_STREAM("number of goal joints is  " << gh.getGoal()->trajectory.joint_names.size());
	for (uint i = 0; i < gh.getGoal()->trajectory.joint_names.size(); ++i){
	ROS_INFO_STREAM("The incoming goal joint names are " << gh.getGoal()->trajectory.joint_names[i]);
	}


    if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    
    // get trajectory
    current_traj_ = active_goal_.getGoal()->trajectory;
    
    //get FK
    //fk_computed = false;
    //fk_computed = getForwardKinematics(current_traj_, fk_response);
   // if (fk_computed){
	  //publish trajectory 	
      pub_controller_command_.publish(current_traj_);
   // }
    
  }

  void cancelCB(GoalHandle gh)
  {
	double now = ros::Time::now().toSec(); 
	ROS_WARN("Amigo_joint_trajectory action: Goal number %d is cancelled at time %f",goal_nr,now);   
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  void getTipPosition(const amigo_msgs::tip_refConstPtr &msg){
    
    if (!has_active_goal_)
      return;
      
    if (!fk_computed)
      return;
    
    std::vector<int> status_tip;
    status_tip.resize(6);
    
    int reached = 0;
    
    //populate feedback msg
    ///feedback_.position = *msg;
  
    //publish feedback
    ///as_.publishFeedback(feedback_);

    //convert quat to rpy
	geometry_msgs::Quaternion quat_msg;
	quat_msg.x = fk_coords[3];
	quat_msg.y = fk_coords[4];
	quat_msg.z = fk_coords[5];
	quat_msg.w = fk_coords[6];

    tf::Quaternion quat;
	tf::quaternionMsgToTF(quat_msg,quat);
    double fk_response_roll, fk_response_pitch, fk_response_yaw;
    btMatrix3x3(quat).getRPY(fk_response_roll, fk_response_pitch, fk_response_yaw);


    //determine if current position is within tolerance
    if (msg->x <= (fk_coords[0] + tol_x/2) && msg->x >= (fk_coords[0] - tol_x/2))
      status_tip[0] = 1;
    if (msg->y <= (fk_coords[1] + tol_y/2) && msg->y >= (fk_coords[1] - tol_y/2))
      status_tip[1] = 1;  
    if (msg->z <= (fk_coords[2] + tol_z/2) && msg->z >= (fk_coords[2] - tol_z/2))
      status_tip[2] = 1;
    if (msg->roll <= (fk_response_roll + tol_roll/2) && msg->roll >= (fk_response_roll - tol_roll/2))
      status_tip[3] = 1;
    if (msg->pitch <= (fk_response_pitch + tol_pitch/2) && msg->pitch >= (fk_response_pitch - tol_pitch/2))
      status_tip[4] = 1;
    if (msg->yaw <= (fk_response_yaw + tol_yaw/2) && msg->yaw >= (fk_response_yaw - tol_yaw/2))
      status_tip[5] = 1;

	///ROS_INFO("curr: %f, %f, %f, %f, %f, %f",msg->x,msg->y,msg->z,msg->roll,msg->pitch,msg->yaw);
	///ROS_INFO("req : %f, %f, %f, %f, %f, %f",fk_coords[0],fk_coords[1],fk_coords[2],
	///fk_response_roll,fk_response_pitch,fk_response_yaw);
	

	
	for (int i=0;i<6;i++)
      reached += status_tip[i];  

    ///ROS_INFO("status tip: %d %d %d %d %d %d",status_tip[0],status_tip[1],status_tip[2]
       ///                                     ,status_tip[3],status_tip[4],status_tip[5]);

    //if position reached  
    if (reached == 6){
      ///result_.position = feedback_.position;
      ///ROS_INFO("%s: Succeeded", action_name_.c_str());
      ROS_INFO("joint trajectory action succeeded");
      // set the action state to succeeded
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
  
  bool getForwardKinematics(trajectory_msgs::JointTrajectory& traj, kinematics_msgs::GetPositionFK::Response fk_response){
    
    ROS_INFO("Requesting forward kinematics");
    int num_points = traj.points.size();
    int num_joints = traj.joint_names.size();
    
    //create fk request and response
    kinematics_msgs::GetPositionFK::Request  fk_request;
    
    //populate request
    fk_request.header.frame_id = root_frame;
    fk_request.fk_link_names.resize(1);
    fk_request.fk_link_names[0] = tip_frame;
    
    fk_request.robot_state.joint_state.position.resize(num_joints);
    fk_request.robot_state.joint_state.name = traj.joint_names;
    
    for (int i =0 ; i < num_joints ;i++)
      fk_request.robot_state.joint_state.position[i] = traj.points[num_points-1].positions[i];
      
    //wait for fk server connection
    ros::service::waitForService("get_fk");
      
    if(fk_client.call(fk_request, fk_response)){
  
      if(fk_response.error_code.val == fk_response.error_code.SUCCESS){
        
        for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++){
	      /*
	      ROS_INFO("Forward kinematics:");
          ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
          ROS_INFO_STREAM("Desired Position: " << 
          fk_response.pose_stamped[i].pose.position.x << "," <<  
          fk_response.pose_stamped[i].pose.position.y << "," << 
          fk_response.pose_stamped[i].pose.position.z);
          ROS_INFO("Desired Orientation: %f %f %f %f",
          fk_response.pose_stamped[i].pose.orientation.x,
          fk_response.pose_stamped[i].pose.orientation.y,
          fk_response.pose_stamped[i].pose.orientation.z,
          fk_response.pose_stamped[i].pose.orientation.w);
          */ 
          fk_coords[0] = fk_response.pose_stamped[i].pose.position.x;
          fk_coords[1] = fk_response.pose_stamped[i].pose.position.y;
          fk_coords[2] = fk_response.pose_stamped[i].pose.position.z;
          fk_coords[3] = fk_response.pose_stamped[i].pose.orientation.x;
          fk_coords[4] = fk_response.pose_stamped[i].pose.orientation.y;
          fk_coords[5] = fk_response.pose_stamped[i].pose.orientation.z;
          fk_coords[6] = fk_response.pose_stamped[i].pose.orientation.w;
        } 
        
      }
      else{
        ROS_ERROR("Forward kinematics failed");
        return false;
	  }
    
    }
    else{
      ROS_ERROR("Forward kinematics service call failed");
      return false;
    }
	fk_computed = true;
    return true;
  }
  
  
  
  int goal_nr;
  bool fk_computed;
  ros::NodeHandle node_;
  JTAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber tip_sub;
  ros::ServiceClient fk_client;
  std::string root_frame;
  std::string tip_frame;

  double fk_coords[7];
  double tol_x, tol_y, tol_z, tol_roll, tol_pitch, tol_yaw;
  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;

  kinematics_msgs::GetPositionFK::Response fk_response;
  std::vector<std::string> joint_names_;
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_action_node");
  ros::NodeHandle node;//("~");
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}
