// Author: Stuart Glaser && Rob Janssen && Janno Lunenburg

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <diagnostic_msgs/DiagnosticArray.h>

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

            joint_index_[(std::string)name_value] = joint_names_.size();
            joint_names_.push_back((std::string)name_value);

        }

        pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

        // Gets the constraints for each joint.
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            ROS_WARN_ONCE("ToDo: parse from URDF as much as possible");
            std::string ns = std::string("constraints/") + joint_names_[i];
            double ig, fg, t, mip, map;
            pn.param(ns + "/intermediate_goal", ig, -1.0);
            pn.param(ns + "/final_goal", fg, -1.0);
            pn.param(ns + "/trajectory", t, -1.0);
            pn.param(ns + "/min_pos", mip, -1.0);
            pn.param(ns + "/max_pos", map, -1.0);
            intermediate_goal_constraints_[joint_names_[i]] = ig;
            final_goal_constraints_[joint_names_[i]] = fg;
            trajectory_constraints_[joint_names_[i]] = t;
            joint_min_constraints_[joint_names_[i]] = mip;
            joint_max_constraints_[joint_names_[i]] = map;
            ROS_DEBUG("Joint %s, min = %f, max = %f, int = %f, final = %f, traj = %f", joint_names_[i].c_str(), mip, map, ig, fg, t);
        }
        nr_torso_joints_ = joint_names.size() - 7;// Assume 7 DoF arm... // ToDo: make nice

        // Here we start sending the references
        pub = node_.advertise<sensor_msgs::JointState>("references", 1);
        torso_pub = node_.advertise<sensor_msgs::JointState>("torso/references",1);
        // Here we start listening for the measured positions
        sub = node_.subscribe("measurements", 1, &JointTrajectoryExecuter::armCB, this);
        torso_sub = node_.subscribe("torso/measurements", 1, &JointTrajectoryExecuter::armCB, this);

        // Diagnostics sub
        diag_sub = node_.subscribe("hardware_status", 1, &JointTrajectoryExecuter::diagnosticsCB, this);

        // Start with hardware status OK
        torso_status = 2;
        arm_status = 2;
        torso_diag_name = "spindle"; // ToDo: don't hardcode
        std::string leftstr = "left";
        std::string rightstr= "right";
        for (unsigned int i = 0; i < joint_names_.size(); i++ ) {
            if (joint_names_[i].find(leftstr) != std::string::npos) {
                arm_diag_name = "left_arm";
                break;
            } else if (joint_names_[i].find(rightstr) != std::string::npos) {
                arm_diag_name = "right_arm";
                break;
            }
        }
        ROS_INFO("Torso diag name = %s, arm diag name = %s", torso_diag_name.c_str(), arm_diag_name.c_str());

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

            // Torso
            sensor_msgs::JointState torso_msg;
            for (uint i = 0; i < nr_torso_joints_; i++) {
                torso_msg.name.push_back(joint_names_[i]);
                torso_msg.position.push_back(cur_pos_[joint_names_[i]]);
            }
            torso_pub.publish(torso_msg);

            // arm
            sensor_msgs::JointState arm_msg;
            for (uint i = 0; i < 7; i++) {
                arm_msg.name.push_back(joint_names_[i+nr_torso_joints_]);
                arm_msg.position.push_back(cur_pos_[joint_names_[i+nr_torso_joints_]]);
            }
            pub.publish(arm_msg);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
            ROS_WARN("Canceling previous goal");
        }

        number_of_goal_joints_ = gh.getGoal()->trajectory.joint_names.size();

        // Check feasibility of arm joint goals
        for (uint i = 0; i < number_of_goal_joints_; i++) {
            std::string joint_name = gh.getGoal()->trajectory.joint_names[i];
            for (uint j = 0; j < gh.getGoal()->trajectory.points.size(); j++) {
                double ref = gh.getGoal()->trajectory.points[j].positions[i];
                if (ref < joint_min_constraints_[joint_name] || ref > joint_max_constraints_[joint_name]) {
                    ROS_WARN("Reference for joint %s is %f but should be between %f and %f.",joint_name.c_str(),ref,joint_min_constraints_[joint_name],joint_max_constraints_[joint_name]);
                    gh.setRejected();
                    has_active_goal_=false;
                    return;
                }
            }
        }

        ///ROS_INFO("Number of goal joints = %i",number_of_goal_joints_);
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;

        // Start by assuming hardware works
        arm_status = 2;
        torso_status = 2;

    }

    void cancelCB(GoalHandle gh)
    {
        if (active_goal_ == gh)
        {
            // Stops the controller.

            //spindle
            sensor_msgs::JointState torso_msg;
            for (uint i = 0; i < nr_torso_joints_; i++) {
                torso_msg.name.push_back(joint_names_[i]);
                torso_msg.position.push_back(cur_pos_[joint_names_[i]]);
            }
            torso_pub.publish(torso_msg);

            // arm
            sensor_msgs::JointState arm_msg;
            for (uint i = 0; i < 7; i++) {
                arm_msg.name.push_back(joint_names_[i+nr_torso_joints_]);
                arm_msg.position.push_back(cur_pos_[joint_names_[i+nr_torso_joints_]]);
            }
            pub.publish(arm_msg);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
    }


    ros::NodeHandle node_;
    JTAS action_server_;
    ros::Publisher pub;
    ros::Subscriber sub;

    ros::Publisher torso_pub;
    ros::Subscriber torso_sub;
    ros::Subscriber diag_sub;

    unsigned int torso_status, arm_status;
    std::string torso_diag_name; // Name of the torso in the diagnostics message array
    std::string arm_diag_name; // Name of the arm in the diagnostics message array

    ros::Time now;

    bool has_active_goal_;
    int current_point;
    GoalHandle active_goal_;
    ///bool goal_includes_spindle_;
    uint number_of_goal_joints_;
    uint nr_torso_joints_;

    std::vector<std::string> joint_names_;
    std::map<std::string, unsigned int> joint_index_;
    std::map<std::string,double> intermediate_goal_constraints_;
    std::map<std::string,double> final_goal_constraints_;
    std::map<std::string,double> trajectory_constraints_;
    std::map<std::string,double> joint_min_constraints_;
    std::map<std::string,double> joint_max_constraints_;
    std::map<std::string,double> cur_pos_;                          // Current position
    std::map<std::string,double> ref_pos_;                          // Desired position
    double goal_time_constraint_;

    void armCB(const sensor_msgs::JointState& joint_meas)
    {

        for(unsigned int i = 0; i < joint_meas.name.size(); ++i) {
            std::map<std::string, unsigned int>::iterator it_joint = joint_index_.find(joint_meas.name[i]);
            if (it_joint != joint_index_.end()) {
                cur_pos_[joint_meas.name[i]] = joint_meas.position[i];
            } else {
                ROS_ERROR("Unknown joint name: %s", joint_meas.name[i].c_str());
            }
        }

        // If no active goal --> Do nothing
        if (!has_active_goal_)
            return;

        controllerCB();

    }

    void controllerCB() {

        int converged_joints=0;
        float abs_error=0.0;

        // Check hardware status
        // ToDo: are we happy with this?
        if ( arm_status == 2 && torso_status == 2 ) {
            //ROS_INFO("Hardware status OK");
        } else if ( arm_status == 4 || torso_status == 4 ) {
            ROS_WARN("Arm (%u) or torso (%u) is in error, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
            active_goal_.setAborted();
            has_active_goal_=false;
            return;
        } else if ( arm_status == 0 || torso_status == 0 ) {
            ROS_WARN("Arm (%u) or torso (%u) is stale, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
            active_goal_.setAborted();
            has_active_goal_=false;
            return;
        } else if ( arm_status == 3 || torso_status == 3 ) {
            ROS_WARN("Arm (%u) or torso (%u) is still homing, joint trajectory goal may not be reached", arm_status, torso_status);
        } else if ( arm_status == 1 || torso_status == 1 ) {
            ROS_WARN("Arm (%u) or torso (%u) is in idle, joint trajectory goal cannot be reached, aborting", arm_status, torso_status);
            active_goal_.setAborted();
            has_active_goal_=false;
            return;
        }

        // Check if the time constraint is not violated
        if(ros::Time::now().toSec() > goal_time_constraint_ + now.toSec())
        {
            ROS_WARN("Aborting because the time constraint was violated");
            active_goal_.setAborted();
            has_active_goal_=false;
            return;
        }

        ///ROS_INFO("Number of joints received goal = %i",active_goal_.getGoal()->trajectory.joint_names.size());
        ///for (uint ii = 0; ii < active_goal_.getGoal()->trajectory.joint_names.size(); ii++) ROS_INFO("Joint name = %s",active_goal_.getGoal()->trajectory.joint_names[ii].c_str());

        for (unsigned int i = 0; i < active_goal_.getGoal()->trajectory.joint_names.size(); i++) {
            std::string joint_name = active_goal_.getGoal()->trajectory.joint_names[i];

            // Compute absolute error
            double ref_pos = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
            ref_pos_[joint_name] = ref_pos; // Required to push reference
            double cur_pos = cur_pos_[joint_name];
            abs_error = fabs(ref_pos - cur_pos);
            //ROS_DEBUG("%s: r: %f\t q: %f\t e: %f",joint_name.c_str(), ref_pos_[joint_name], cur_pos_[joint_name], abs_error);

            // Check trajectory constraint
            if(abs_error > trajectory_constraints_[joint_name]) {
                ROS_WARN("Aborting because the trajectory constraint of %s (%f) was violated (%f)", joint_name.c_str(), trajectory_constraints_[joint_name], abs_error);
                active_goal_.setAborted();
                has_active_goal_=false;
                return;
            }

            // Check if this joint has converged
            if(current_point < ((int)active_goal_.getGoal()->trajectory.points.size()-1))
            {
                if(abs_error < intermediate_goal_constraints_[joint_name])
                {
                    //ROS_DEBUG("intermediate goal constraints for %s converged", joint_name.c_str());
                    converged_joints += 1;
                }
            }
            else
            {
                if(abs_error < final_goal_constraints_[joint_name])
                {
                    //ROS_DEBUG("final goal constraints for %s converged", joint_name.c_str());
                    converged_joints += 1;
                }
            }

        }

        // Joint trajectory action should work for both seven (old situation, only arm) and eight (new situation, incl torso) joints

        // Only publish torso if requested
        if( number_of_goal_joints_ > 7) {
            sensor_msgs::JointState torso_msg;
            for (unsigned int i = 0; i < nr_torso_joints_; i++) {
                torso_msg.name.push_back(joint_names_[i]);
                torso_msg.position.push_back(ref_pos_[joint_names_[i]]);
            }
            torso_pub.publish(torso_msg);
        }

        // Always publish arm msg
        sensor_msgs::JointState arm_msg;
        for (unsigned int i = 0; i < 7; i++) {
            arm_msg.name.push_back(joint_names_[i+nr_torso_joints_]);
            arm_msg.position.push_back(ref_pos_[joint_names_[i+nr_torso_joints_]]);
        }
        pub.publish(arm_msg);

        //if(converged_joints==(int)number_of_goal_joints_)
        if (converged_joints==(int)number_of_goal_joints_)
        {
            now = ros::Time::now();
            current_point = current_point + 1;
            ROS_INFO("every joint has converged, go to the next point");
        }

        if(current_point==(int)active_goal_.getGoal()->trajectory.points.size())
        {
            ROS_INFO("active goal succeeded");
            active_goal_.setSucceeded();
            has_active_goal_ = false;
        }

        ROS_DEBUG("Converged joints = %i of %i, current_point = %i of %i", converged_joints, (int)number_of_goal_joints_, current_point, (int)active_goal_.getGoal()->trajectory.points.size());
    }

    void diagnosticsCB(const diagnostic_msgs::DiagnosticArray& diag_array) {
        // Only process data if there is an active goal
        if (has_active_goal_) {
            // Loop through message
            for (unsigned int i = 0; i < diag_array.status.size(); i++ ) {
                // Check if there is a torso or an arm status status
                if (diag_array.status[i].name == torso_diag_name) {
                    torso_status = diag_array.status[i].level;
                } else if (diag_array.status[i].name == arm_diag_name) {
                    arm_status = diag_array.status[i].level;
                }
            }
            //ROS_INFO("Arm status %s = %u, torso status %s = %u", arm_diag_name.c_str(), arm_status, torso_diag_name.c_str(), torso_status);
        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action_node");
    ros::NodeHandle node("~");
    JointTrajectoryExecuter jte(node);

    ros::spin();

    return 0;
}


