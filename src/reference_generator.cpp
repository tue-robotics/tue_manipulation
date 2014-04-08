#include "reference_generator.h"

using namespace std;

// --------------------------------------------------------------------------------

JointTrajectoryExecuter::JointTrajectoryExecuter(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "joint_trajectory_action",
                   boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                   boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1),
                   false),
    has_active_goal_(false)
{
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    // Gets all of the joints
    XmlRpc::XmlRpcValue joint_names_param;
    if (!pn.getParam("joint_names", joint_names_param)) {
        ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
        exit(1);
    }

    if (joint_names_param.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
        exit(1);
    }

    std::vector<std::string> joint_names;

    for (int i = 0; i < joint_names_param.size(); ++i) {
        XmlRpcValue &name_value = joint_names_param[i];
        if (name_value.getType() != XmlRpcValue::TypeString) {
            ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                      pn.getNamespace().c_str());
            exit(1);
        }

        joint_names.push_back((std::string)name_value);
    }

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names.size(); ++i) {
        std::string ns = std::string("constraints/") + joint_names[i];
        double ig, fg,t, max_vel;
        pn.param(ns + "/intermediate_goal", ig, -1.0);
        pn.param(ns + "/final_goal", fg, -1.0);
        pn.param(ns + "/trajectory", t, -1.0);
        pn.param(ns + "/max_vel", max_vel, 1.0);
        intermediate_goal_constraints_[joint_names[i]] = ig;
        final_goal_constraints_[joint_names[i]] = fg;
        trajectory_constraints_[joint_names[i]] = t;
        max_vel_constraints_[joint_names_param[i]] = max_vel;
    }

    ///ROS_INFO("Intermediate goal constraints %f, %f, %f, %f, %f, %f, %f, %f", intermediate_goal_constraints_[0], intermediate_goal_constraints_[1], intermediate_goal_constraints_[2], intermediate_goal_constraints_[3], intermediate_goal_constraints_[4], intermediate_goal_constraints_[5], intermediate_goal_constraints_[6], intermediate_goal_constraints_[7]);
    ///ROS_INFO("Final goal constraints        %f, %f, %f, %f, %f, %f, %f, %f", final_goal_constraints_[0], final_goal_constraints_[1], final_goal_constraints_[2], final_goal_constraints_[3], final_goal_constraints_[4], final_goal_constraints_[5], final_goal_constraints_[6], final_goal_constraints_[7]);
    ///ROS_INFO("Trajectory constraints        %f, %f, %f, %f, %f, %f, %f, %f", trajectory_constraints_[0], trajectory_constraints_[1], trajectory_constraints_[2], trajectory_constraints_[3], trajectory_constraints_[4], trajectory_constraints_[5], trajectory_constraints_[6], trajectory_constraints_[7]);

    // Here we start sending the references
    arm_pub = node_.advertise<trajectory_msgs::JointTrajectory>("/references", 1);
    torso_pub = node_.advertise<trajectory_msgs::JointTrajectory>("/amigo/torso/references",1);
    // Here we start listening for the measured positions
    arm_sub_ = node_.subscribe("/measurements", 1, &JointTrajectoryExecuter::measurementCB, this);
    torso_sub = node_.subscribe("/amigo/torso/measurements", 1, &JointTrajectoryExecuter::measurementCB, this);

    // Diagnostics sub
    diag_sub = node_.subscribe("/hardware_status", 1, &JointTrajectoryExecuter::diagnosticsCB, this);

    // Start with hardware status OK
    torso_status = 2;
    arm_status = 2;
    torso_diag_name = "spindle"; // ToDo: don't hardcode
    std::string leftstr = "left";
    std::string rightstr= "right";
    for (unsigned int i = 0; i < joint_names.size(); i++ ) {
        if (joint_names[i].find(leftstr) != std::string::npos) {
            arm_diag_name = "left_arm";
            break;
        } else if (joint_names[i].find(rightstr) != std::string::npos) {
            arm_diag_name = "right_arm";
            break;
        }
    }
    ROS_INFO("Torso diag name = %s, arm diag name = %s", torso_diag_name.c_str(), arm_diag_name.c_str());

    action_server_.start();
}

// --------------------------------------------------------------------------------

JointTrajectoryExecuter::~JointTrajectoryExecuter() {
    arm_pub.shutdown();
    arm_sub_.shutdown();
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::sendStop() {

    trajectory_msgs::JointTrajectory arm_msg, torso_msg;
    trajectory_msgs::JointTrajectoryPoint arm_point_msg, torso_point_msg;
    for(std::map<std::string, double>::const_iterator it = current_joint_pos_.begin(); it != current_joint_pos_.end(); ++it) {
        const std::string& joint_name = it->first;
        double joint_pos = it->second;

        if (joint_name == "torso_joint") {
            torso_msg.joint_names.push_back(joint_name);
            torso_point_msg.positions.push_back(joint_pos);
        } else {
            arm_msg.joint_names.push_back(joint_name);
            arm_point_msg.positions.push_back(joint_pos);
        }
    }

    arm_msg.points.push_back(arm_point_msg);
    torso_msg.points.push_back(torso_point_msg);

    if (!arm_msg.joint_names.empty()) {
        arm_pub.publish(arm_msg);
    }

    if (!torso_msg.joint_names.empty()) {
        torso_pub.publish(torso_msg);
    }

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
    ROS_WARN("Canceling previous goal");
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::goalCB(GoalHandle gh) {
    t_start_ = ros::Time::now();

    // Cancels the currently active goal.
    if (has_active_goal_) {
        // Stops the controller.
        sendStop();
    }

    trajectory_input_ = gh.getGoal()->trajectory;

    JointPosState cur_pos_ordered;
    JointPosState max_vel_ordered;

    for (unsigned int i = 0; i < trajectory_input_.joint_names.size(); ++i) {
        const std::string& joint_name = trajectory_input_.joint_names[i];

        std::map<std::string, double>::iterator it_joint = current_joint_pos_.find(joint_name);
        if (it_joint != current_joint_pos_.end()) {
            cur_pos_ordered.push_back(it_joint->second);
        } else {
            ROS_ERROR("No measurement for joint %s", joint_name.c_str());
            gh.setRejected();
            return;
        }

        std::map<std::string, double>::iterator it_joint2 = max_vel_constraints_.find(joint_name);
        if (it_joint != max_vel_constraints_.end()) {
            max_vel_ordered.push_back(it_joint2->second);
        } else {
            ROS_ERROR("No max velocity constraint for joint %s", joint_name.c_str());
            gh.setRejected();
            return;
        }
    }

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Start by assuming hardware works
    arm_status = 2;
    torso_status = 2;

    // generate interpolated trajectory
    trajectory_msgs::JointTrajectory trajectory_interpolated_msg;
    interpolateTrajectory(cur_pos_ordered, max_vel_ordered, trajectory_input_, trajectory_interpolated_msg);

    // send trajectory to controllers
    publishReference(trajectory_interpolated_msg);
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::cancelCB(GoalHandle gh) {
    if (active_goal_ == gh) {
        // Stops the controller.
        sendStop();
    }
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::measurementCB(const sensor_msgs::JointState& joint_meas) {

    // --------------------------------------------------
    // SET CURRENT POS
    // --------------------------------------------------

    for(unsigned int i = 0; i < joint_meas.name.size(); ++i) {
        current_joint_pos_[joint_meas.name[i]] = joint_meas.position[i];
    }

    // If no active goal --> Do nothing
    if (!has_active_goal_)
        return;

    // --------------------------------------------------
    // CHECK DIAGNOSTICS
    // --------------------------------------------------

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

    // --------------------------------------------------
    // CHECK TIME CONSTRAINT
    // --------------------------------------------------

    // Check if the time constraint is not violated
    if(ros::Time::now().toSec() > goal_time_constraint_ + t_start_.toSec()) {
        ROS_WARN("Aborting because the time constraint was violated");        
        active_goal_.setAborted();
        has_active_goal_=false;
        return;
    }

    // --------------------------------------------------
    // CHECK IF GOAL REACHED
    // --------------------------------------------------

    unsigned int converged_joints = 0;

    const trajectory_msgs::JointTrajectoryPoint& goal_pos = trajectory_input_.points.back();

    for (unsigned int i = 0; i < trajectory_input_.joint_names.size(); ++i) {
        const std::string& joint_name = trajectory_input_.joint_names[i];
        double goal = goal_pos.positions[i];

        std::map<std::string, double>::iterator it_current_pos = current_joint_pos_.find(joint_name);
        if (it_current_pos == current_joint_pos_.end()) {
            ROS_WARN("No measurements received for joint %s.", joint_name.c_str());
            break;
        }

        double current = it_current_pos->second;
        double abs_error = std::abs(goal - current);
        ROS_DEBUG("%s: r: %f\t q: %f\t e: %f", joint_name.c_str(), goal, current, abs_error);

        // Check if this joint has converged
        if (abs_error < final_goal_constraints_[joint_name]) {
            converged_joints = converged_joints + 1;
        }
    }

    ROS_DEBUG("Converged joints = %i of %i", converged_joints, (int)trajectory_input_.joint_names.size());

    if (converged_joints == trajectory_input_.joint_names.size()) {
        active_goal_.setSucceeded();
        has_active_goal_ = false;
    }
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::diagnosticsCB(const diagnostic_msgs::DiagnosticArray& diag_array) {
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

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::interpolateTrajectory(const JointPosState& current_pos, const JointVelState& max_vel,
                                                    const trajectory_msgs::JointTrajectory& jt,
                                                    trajectory_msgs::JointTrajectory& jt_interpolated) {

    jt_interpolated.joint_names = jt.joint_names;

    JointPosState pos = current_pos;
    for(unsigned int i = 0; i < jt.points.size(); ++i) {
        const JointPosState& goal_pos = jt.points[i].positions;

        // find out how long it will take to reach the goal
        double max_time = 0;
        for(unsigned int j = 0; j < goal_pos.size(); ++i) {
            double diff = std::abs(goal_pos[j] - pos[j]);
            double time = diff / max_vel[j];
            max_time = std::max(max_time, time);
        }

        double dt = 0.001;

        // perform linear interpolation on all joints
        for(double t = 0; t < max_time; t += dt) {
            trajectory_msgs::JointTrajectoryPoint ref;
            ref.time_from_start = ros::Duration(t);
            ref.positions.resize(goal_pos.size());

            for(unsigned int j = 0; j < goal_pos.size(); ++i) {
                ref.positions[j] = (goal_pos[j] - pos[j]) * t / max_time;
            }
            jt_interpolated.points.push_back(ref);
        }

        pos = goal_pos;
    }
}

void JointTrajectoryExecuter::publishReference(const trajectory_msgs::JointTrajectory& jt) {
    trajectory_msgs::JointTrajectory arm_msg, torso_msg;

    arm_msg.points.resize(jt.points.size());
    torso_msg.points.resize(jt.points.size());

    unsigned int torso_joint_index = jt.joint_names.size();

    // find which index the torso joint is
    for(unsigned int j = 0; j < jt.joint_names.size(); ++j) {
        if (jt.joint_names[j] == "torso_joint") {
            torso_msg.joint_names.push_back(jt.joint_names[j]);
            torso_joint_index = j;
        } else {
            arm_msg.joint_names.push_back(jt.joint_names[j]);
        }
    }

    for(unsigned int i = 0; i < jt.points.size(); ++i) {
        for(unsigned int j = 0; j < jt.joint_names.size(); ++j) {
            if (j == torso_joint_index) {
                torso_msg.points[i].positions.push_back(jt.points[i].positions[j]);
                torso_msg.points[i].time_from_start = jt.points[i].time_from_start;
            } else {
                arm_msg.points[i].positions.push_back(jt.points[i].positions[j]);
                arm_msg.points[i].time_from_start = jt.points[i].time_from_start;
            }
        }
    }

    if (!arm_msg.joint_names.empty()) {
        arm_pub.publish(arm_msg);
    }

    if (!torso_msg.joint_names.empty()) {
        torso_pub.publish(torso_msg);
    }
}

// --------------------------------------------------------------------------------

// --------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_action_node");
    ros::NodeHandle node;//("~");
    JointTrajectoryExecuter jte(node);

    ros::spin();

    return 0;
}
