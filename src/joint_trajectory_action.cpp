// Author: Stuart Glaser && Rob Janssen

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <amigo_msgs/arm_joints.h>

using namespace std;

class JointTrajectoryExecuter
{
private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
public:
  JointTrajectoryExecuter(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "joint_trajectory_action",
                   boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                   false),
    has_active_goal_(false),
    current_point(0)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names;
    if (!pn.getParam("joint_names", joint_names))
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

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      double ig, fg,t;
      pn.param(ns + "/intermediate_goal", ig, -1.0);
      pn.param(ns + "/final_goal", fg, -1.0);
      pn.param(ns + "/trajectory", t, -1.0);
      intermediate_goal_constraints_[joint_names_[i]] = ig;
      final_goal_constraints_[joint_names_[i]] = fg;
      trajectory_constraints_[joint_names_[i]] = t;
    }

    // Here we start sending the references
    pub = node_.advertise<amigo_msgs::arm_joints>("joint_references", 1);
    // Here we start listening for the measured positions
    sub = node_.subscribe("joint_measurements", 1, &JointTrajectoryExecuter::controllerCB, this);

    action_server_.start();
  }

  ~JointTrajectoryExecuter()
  {
    pub.shutdown();
    sub.shutdown();
  }

private:

  void goalCB(GoalHandle gh)
  {
    current_point = 0;
    now = ros::Time::now();
    
    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      // Stops the controller.
      amigo_msgs::arm_joints empty;
      pub.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

  }

  void cancelCB(GoalHandle gh)
  {
    if (active_goal_ == gh)
    {
      // Stops the controller.
      amigo_msgs::arm_joints empty;
      pub.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  ros::NodeHandle node_;
  JTAS action_server_;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Time now;

  bool has_active_goal_;
  int current_point;
  GoalHandle active_goal_;

  std::vector<std::string> joint_names_;
  std::map<std::string,double> intermediate_goal_constraints_;
  std::map<std::string,double> final_goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;

  void controllerCB(const amigo_msgs::arm_jointsConstPtr &joint_meas)
  {

	  if (!has_active_goal_)
		  return;

	  int i=0,converged_joints=0;
	  float abs_error=0.0;
	  amigo_msgs::arm_joints joint_ref;

	  // Check if the time constraint is not violated
	  if(ros::Time::now().toSec() > goal_time_constraint_ + now.toSec())
	  {
		  ROS_WARN("Aborting because the time constraint was violated");
		  active_goal_.setAborted();
		  has_active_goal_=false;
		  return;
	  }

	  for (i = 0; i < (int)joint_names_.size(); ++i)
	  {
		  joint_ref.time.data   = ros::Time::now().toSec() + active_goal_.getGoal()->trajectory.points[current_point].time_from_start.toSec();
		  joint_ref.pos[i].data = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
		  joint_ref.vel[i].data = active_goal_.getGoal()->trajectory.points[current_point].velocities[i];
		  joint_ref.acc[i].data = active_goal_.getGoal()->trajectory.points[current_point].accelerations[i];

		  abs_error = fabs(joint_ref.pos[i].data - joint_meas->pos[i].data);

		  // Check if the joints stay within their constraints
		  if(abs_error > trajectory_constraints_[joint_names_[i]])
		  {
			  ROS_WARN("Aborting because the trajectory constraint was violated");
			  active_goal_.setAborted();
			  has_active_goal_=false;
			  return;
		  }
		  // Check if this joint has converged
		  if(current_point < (int)active_goal_.getGoal()->trajectory.points.size())
		  {
		  	if(abs_error < intermediate_goal_constraints_[joint_names_[i]])
		  	{
				  converged_joints = converged_joints + 1;
		  	}
 		  } 
		  else
		  {
			  	if(abs_error < final_goal_constraints_[joint_names_[i]])
			  	{
					  converged_joints = converged_joints + 1;
			  	}
		  }
	  }

	  pub.publish(joint_ref);

	  if(converged_joints==(int)joint_names_.size())
	  {
		  now = ros::Time::now();
		  current_point = current_point + 1;
	  }

	  if(current_point==(int)active_goal_.getGoal()->trajectory.points.size())
	  {
		  active_goal_.setSucceeded();
		  has_active_goal_ = false;
	  }

  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_action_node");
  ros::NodeHandle node;//("~");
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}
