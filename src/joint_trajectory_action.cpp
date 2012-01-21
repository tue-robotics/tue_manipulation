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
    new_active_goal_(false)
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
      double g, t;
      pn.param(ns + "/goal", g, -1.0);
      pn.param(ns + "/trajectory", t, -1.0);
      goal_constraints_[joint_names_[i]] = g;
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
    new_active_goal_ = true;
    
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

  bool has_active_goal_;
  bool new_active_goal_;
  GoalHandle active_goal_;
  amigo_msgs::arm_joints joint_ref;

  std::vector<std::string> joint_names_;
  std::map<std::string,double> goal_constraints_;
  std::map<std::string,double> trajectory_constraints_;
  double goal_time_constraint_;

  void controllerCB(const amigo_msgs::arm_jointsConstPtr &joint_meas)
  {		 
		if (!has_active_goal_)
		return;
		
	    ros::Time now;
	    unsigned int i=0,j=0,converged=0;
	    float abs_error=0.0;
		
		new_active_goal_ = true;
		   
    	// Go over each point
		/*while (ros::ok() && j < active_goal_.getGoal()->trajectory.points.size() && new_active_goal_)
		{
			for (i = 0; i < joint_names_.size(); ++i)
			{
				joint_ref.time.data   = ros::Time::now().toSec() + active_goal_.getGoal()->trajectory.points[j].time_from_start.toSec();
				joint_ref.pos[i].data = active_goal_.getGoal()->trajectory.points[j].positions[i];
				joint_ref.vel[i].data = active_goal_.getGoal()->trajectory.points[j].velocities[i];
				joint_ref.acc[i].data = active_goal_.getGoal()->trajectory.points[j].accelerations[i];
			}
			pub.publish(joint_ref);
	        now = ros::Time::now();		
			converged=0;
			while(ros::ok() && !converged && new_active_goal_)
			{
				// Check if the time for this point was not yet violated
				if(ros::Time::now() > now + active_goal_.getGoal()->trajectory.points[j].time_from_start)
				{
					ROS_WARN("Aborting because the time constraint was violated");
					active_goal_.setAborted();
					has_active_goal_=false;
					return;
				}
				// Check if no joints go outside their boundaries and if they have reached their goal
				for (size_t i = 0; i < joint_names_.size(); ++i)
				{
					abs_error = fabs(joint_ref.pos[i].data - joint_meas->pos[i].data);
					if(abs_error > active_goal_.getGoal()->path_tolerance[i].position)
					{
						ROS_WARN("Aborting because the trajectory constraint was violated");
						active_goal_.setAborted();
						has_active_goal_=false;
						return;
					}
					if(abs_error < active_goal_.getGoal()->goal_tolerance[i].position)
					{
						converged = converged + 1;
					}
				}
			}
			j = j + 1;			
		}  */
		active_goal_.setSucceeded();
		has_active_goal_ = false;
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
