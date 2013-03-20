// Author: Stuart Glaser && Rob Janssen && Janno Lunenburg

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <amigo_msgs/arm_joints.h>
#include <amigo_msgs/spindle_setpoint.h>
#include <std_msgs/Float64.h>

using namespace std;

class JointTrajectoryExecuter
{
private:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
    typedef JTAS::GoalHandle GoalHandle;
    std::vector<double> ref_pos_, cur_pos_;
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
        ///ROS_INFO("Intermediate goal constraints %f, %f, %f, %f, %f, %f, %f, %f", intermediate_goal_constraints_[0], intermediate_goal_constraints_[1], intermediate_goal_constraints_[2], intermediate_goal_constraints_[3], intermediate_goal_constraints_[4], intermediate_goal_constraints_[5], intermediate_goal_constraints_[6], intermediate_goal_constraints_[7]);
        ///ROS_INFO("Final goal constraints        %f, %f, %f, %f, %f, %f, %f, %f", final_goal_constraints_[0], final_goal_constraints_[1], final_goal_constraints_[2], final_goal_constraints_[3], final_goal_constraints_[4], final_goal_constraints_[5], final_goal_constraints_[6], final_goal_constraints_[7]);
        ///ROS_INFO("Trajectory constraints        %f, %f, %f, %f, %f, %f, %f, %f", trajectory_constraints_[0], trajectory_constraints_[1], trajectory_constraints_[2], trajectory_constraints_[3], trajectory_constraints_[4], trajectory_constraints_[5], trajectory_constraints_[6], trajectory_constraints_[7]);

        // Here we start sending the references
        pub = node_.advertise<amigo_msgs::arm_joints>("joint_references", 1);
        torso_pub = node_.advertise<amigo_msgs::spindle_setpoint>("/spindle_controller/spindle_coordinates",1);
        // Here we start listening for the measured positions
        sub = node_.subscribe("joint_measurements", 1, &JointTrajectoryExecuter::armCB, this);
        torso_sub = node_.subscribe("/spindle_position", 1, &JointTrajectoryExecuter::torsoCB, this);

        ref_pos_.resize(joint_names_.size());
        cur_pos_.resize(joint_names_.size());

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
            amigo_msgs::spindle_setpoint torso_msg;
            amigo_msgs::arm_joints arm_msg;

            torso_msg.pos = cur_pos_[0];
            for (uint i = 0; i < 7; i++) arm_msg.pos[i].data = cur_pos_[i+1];
            torso_pub.publish(torso_msg);
            pub.publish(arm_msg);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
            ROS_WARN("Canceling previous goal");
        }

        // By default: spindle is not used, only if explicitly stated in message
        ///goal_includes_spindle_ = false;
        number_of_goal_joints_ = 7;
        for (uint i = 0; i < gh.getGoal()->trajectory.joint_names.size(); i++) {
            if (std::strcmp(gh.getGoal()->trajectory.joint_names[i].c_str(),"spindle_joint")) number_of_goal_joints_ = 8;
        }
        //ROS_INFO("Number of goal joints = %i",number_of_goal_joints_);
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;

    }

    void cancelCB(GoalHandle gh)
    {
        if (active_goal_ == gh)
        {
            // Stops the controller.
            amigo_msgs::spindle_setpoint torso_msg;
            amigo_msgs::arm_joints arm_msg;

            torso_msg.pos = cur_pos_[0];
            for (uint i = 0; i < 7; i++) arm_msg.pos[i].data = cur_pos_[i+1];
            torso_pub.publish(torso_msg);
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

    ros::Time now;

    bool has_active_goal_;
    int current_point;
    GoalHandle active_goal_;
    ///bool goal_includes_spindle_;
    uint number_of_goal_joints_;

    std::vector<std::string> joint_names_;
    std::map<std::string,double> intermediate_goal_constraints_;
    std::map<std::string,double> final_goal_constraints_;
    std::map<std::string,double> trajectory_constraints_;
    double goal_time_constraint_;

    void armCB(const amigo_msgs::arm_jointsConstPtr &joint_meas)
    {

        ///ROS_INFO("Arm message received");
        for (uint i = 0; i < 7; i++) {
            cur_pos_[i+1] = joint_meas->pos[i].data;
        }
        ///ROS_INFO("Arm message copied");

        // If no active goal --> Do nothing
        if (!has_active_goal_)
            return;

        controllerCB();

        ///amigo_msgs::arm_joints joint_ref;


    }

    void torsoCB(const std_msgs::Float64ConstPtr &torso_meas) {

        cur_pos_[0] = torso_meas->data;
        ///ROS_INFO("Torso message copied");

        ///ROS_INFO("Torso message received");
        // If no active goal --> do nothing
        if (!has_active_goal_)
            return;

        controllerCB();

    }

    void controllerCB() {

        int i=0,converged_joints=0;
        float abs_error=0.0;

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

        ///for (i = 0; i < (int)joint_names_.size(); ++i)
        for (i = 0; i < (int)number_of_goal_joints_; ++i)
        {
            ///joint_ref.time.data   = ros::Time::now().toSec() + active_goal_.getGoal()->trajectory.points[current_point].time_from_start.toSec();
            ///joint_ref.pos[i].data = active_goal_.getGoal()->trajectory.points[current_point].positions[i];
            //joint_ref.vel[i].data = active_goal_.getGoal()->trajectory.points[current_point].velocities[i];
            //joint_ref.acc[i].data = active_goal_.getGoal()->trajectory.points[current_point].accelerations[i];
            ref_pos_[i] = active_goal_.getGoal()->trajectory.points[current_point].positions[i];

            abs_error = fabs(ref_pos_[i] - cur_pos_[i]);

            // Check if the joints stay within their constraints
            if (number_of_goal_joints_ == 8) {
                if(abs_error > trajectory_constraints_[joint_names_[i]]) {
                    ROS_WARN("Aborting because the trajectory constraint was violated");
                    active_goal_.setAborted();
                    has_active_goal_=false;
                    return;
                }
            }
            else if(number_of_goal_joints_ == 7) {
                if(abs_error > trajectory_constraints_[joint_names_[i+1]]) {
                    ROS_WARN("Aborting because the trajectory constraint was violated");
                    active_goal_.setAborted();
                    has_active_goal_=false;
                    return;
                }
            }

            // Check if this joint has converged
            if(current_point < ((int)active_goal_.getGoal()->trajectory.points.size()-1))
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

        // Joint trajectory action should work for both seven (old situation, only arm) and eight (new situation, incl torso) joints
        amigo_msgs::spindle_setpoint torso_msg;
        amigo_msgs::arm_joints arm_msg;

        // Only arms
        if (number_of_goal_joints_ == 7) {
            for (uint i = 0; i < 7; i++) {
                arm_msg.pos[i].data = ref_pos_[i];
            }
            pub.publish(arm_msg);
        }
        else if(number_of_goal_joints_ == 8) {
            torso_msg.pos = ref_pos_[0];
            torso_msg.vel = 0.0;
            torso_msg.acc = 0.0;
            torso_msg.stop = 0;
            for (uint i = 0; i < 7; i++) {
                arm_msg.pos[i].data = ref_pos_[i+1];
            }
            ///ROS_INFO("Publish torso msg");
            torso_pub.publish(torso_msg);
            ///ROS_INFO("Publish arm msg");
            pub.publish(arm_msg);
        }
        else ROS_WARN("It is unclear which joints to actuate, won't publish");

        ///ROS_INFO("Publishing done");

        if(converged_joints==(int)number_of_goal_joints_)
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

