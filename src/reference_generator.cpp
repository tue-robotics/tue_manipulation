#include "reference_generator.h"

double DT = 0.001;

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
        JointState& joint_info = joint_measurements_[joint_names[i]];
        std::string ns = std::string("constraints/") + joint_names[i];
        double ig, fg, t;
        pn.param(ns + "/intermediate_goal", ig, -1.0);
        pn.param(ns + "/final_goal", fg, -1.0);
        pn.param(ns + "/trajectory", t, -1.0);
        pn.param(ns + "/max_vel", joint_info.max_vel, 1.0);
        pn.param(ns + "/max_acc", joint_info.max_acc, 1.0);
        intermediate_goal_constraints_[joint_names[i]] = ig;
        final_goal_constraints_[joint_names[i]] = fg;
        trajectory_constraints_[joint_names[i]] = t;
    }

    ///ROS_INFO("Intermediate goal constraints %f, %f, %f, %f, %f, %f, %f, %f", intermediate_goal_constraints_[0], intermediate_goal_constraints_[1], intermediate_goal_constraints_[2], intermediate_goal_constraints_[3], intermediate_goal_constraints_[4], intermediate_goal_constraints_[5], intermediate_goal_constraints_[6], intermediate_goal_constraints_[7]);
    ///ROS_INFO("Final goal constraints        %f, %f, %f, %f, %f, %f, %f, %f", final_goal_constraints_[0], final_goal_constraints_[1], final_goal_constraints_[2], final_goal_constraints_[3], final_goal_constraints_[4], final_goal_constraints_[5], final_goal_constraints_[6], final_goal_constraints_[7]);
    ///ROS_INFO("Trajectory constraints        %f, %f, %f, %f, %f, %f, %f, %f", trajectory_constraints_[0], trajectory_constraints_[1], trajectory_constraints_[2], trajectory_constraints_[3], trajectory_constraints_[4], trajectory_constraints_[5], trajectory_constraints_[6], trajectory_constraints_[7]);

    // Here we start sending the references
    arm_pub = node_.advertise<trajectory_msgs::JointTrajectory>("/references", 1);
    torso_pub = node_.advertise<trajectory_msgs::JointTrajectory>("/amigo/torso/ref_trajectory",1);
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

double abs(double v, double& s) {
    if (v >= 0) {
        s = 1.0;
        return v;
    } else {
        s = -1.0;
        return -v;
    }
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::fillStop(trajectory_msgs::JointTrajectory& traj_msg, double dt, std::vector<JointState>& end_joint_states) {

    bool error = false;
    for(unsigned int j = 0; j < traj_msg.joint_names.size(); ++j) {
        const std::string& joint_name = traj_msg.joint_names[j];

        std::map<std::string, JointState>::iterator it_joint = joint_measurements_.find(joint_name);
        if (it_joint != joint_measurements_.end()) {
            const JointState& state = it_joint->second;
            if (state.pos_initialized && state.vel_initialized) {
                end_joint_states.push_back(it_joint->second);
            } else {
                ROS_WARN("No measurement data for joint %s", joint_name.c_str());
                error = true;
            }
        } else {
            ROS_WARN("No joint info available for joint %s", joint_name.c_str());
            error = true;
        }
    }

    if (error) {
        return;
    }

    ROS_INFO("--------------- START ---------------");
    for(unsigned int j = 0; j < end_joint_states.size(); ++j) {
        const JointState& state = end_joint_states[j];
        ROS_INFO("Joint %d: pos = %f, vel = %f", j, state.pos, state.vel);
    }

    while (true) {
        unsigned int num_converged = 0;
        trajectory_msgs::JointTrajectoryPoint p;

        for(unsigned int j = 0; j < end_joint_states.size(); ++j) {
            JointState& state = end_joint_states[j];
//            ROS_INFO("    Joint %d: pos = %f, vel = %f", j, state.pos, state.vel);

            double v_sign;
            double v_abs = abs(state.vel, v_sign);

            v_abs -= state.max_acc * dt;
            if (v_abs <= 0) {
                v_abs = 0;
                ++num_converged;
            }

            state.vel = v_sign * v_abs;
            state.pos += state.vel * dt;

            p.positions.push_back(state.pos);
            p.velocities.push_back(state.vel);
        }

//        ROS_INFO("Converged: %d / %d", num_converged, end_joint_states.size());

        traj_msg.points.push_back(p);

        if (num_converged == end_joint_states.size()) {
            break;
        }
    }    
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::goalCB(GoalHandle gh) {
    t_start_ = ros::Time::now();

    // Cancels the currently active goal.
    if (has_active_goal_) {
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }

    trajectory_input_ = gh.getGoal()->trajectory;

    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;

    // Start by assuming hardware works
    arm_status = 2;
    torso_status = 2;

    trajectory_msgs::JointTrajectory trajectory_interpolated_msg;

    // create trajectory to safely stop arms from moving (if they are)
    std::vector<JointState> joint_states; // will contain the expected joint states after stopping
    trajectory_interpolated_msg.joint_names = trajectory_input_.joint_names;
    fillStop(trajectory_interpolated_msg, DT, joint_states);

    // generate interpolated trajectory with joint_states as starting state
    interpolateTrajectory(joint_states, trajectory_input_, trajectory_interpolated_msg);

    // send trajectory to controllers
    publishReference(trajectory_interpolated_msg);
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::cancelCB(GoalHandle gh) {
    if (active_goal_ == gh) {
        trajectory_msgs::JointTrajectory traj;
        traj.joint_names = trajectory_input_.joint_names;

        std::vector<JointState> joint_infos;
        fillStop(traj, DT, joint_infos);

        publishReference(traj);

        // Marks the current goal as canceled.
        active_goal_.setCanceled();
        has_active_goal_ = false;
    }
}

// --------------------------------------------------------------------------------

void JointTrajectoryExecuter::measurementCB(const sensor_msgs::JointState& joint_meas) {

    // --------------------------------------------------
    // SET CURRENT POS
    // --------------------------------------------------

    for(unsigned int i = 0; i < joint_meas.name.size(); ++i) {
        std::map<std::string, JointState>::iterator it_joint = joint_measurements_.find(joint_meas.name[i]);
        if (it_joint != joint_measurements_.end()) {
            double pos = joint_meas.position[i];

            JointState& joint_info = it_joint->second;

            // check if velocity was send in message
            if (i < joint_meas.velocity.size()) {
                joint_info.vel = joint_meas.velocity[i];
                joint_info.vel_initialized = true;
            } else if (joint_info.pos_initialized) {
                // if not, estimate it based on last measured position and timestamp
                double dt = joint_meas.header.stamp.toSec() - joint_info.t_last_update;
                if (dt > 0) {
                    joint_info.vel = (pos - joint_info.pos) / dt;
                    joint_info.vel_initialized = true;
                } else {
                    ROS_ERROR("Cannot estimate joint velocitiies: joint measurements do not contain timestamps!");
                }
            }

            joint_info.pos = pos;
            joint_info.pos_initialized = true;
            joint_info.t_last_update = joint_meas.header.stamp.toSec();
        } else {
            ROS_WARN("Measurement received for unknown joint: %s", joint_meas.name[i].c_str());
        }
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

        std::map<std::string, JointState>::iterator it_current_pos = joint_measurements_.find(joint_name);
        if (it_current_pos == joint_measurements_.end()) {
            ROS_WARN("No measurements received for joint %s.", joint_name.c_str());
            break;
        }

        double current = it_current_pos->second.pos;
        double abs_error = std::abs(goal - current);
        ROS_DEBUG("%s: r: %f\t q: %f\t e: %f", joint_name.c_str(), goal, current, abs_error);

        // Check if this joint has converged
        if (abs_error < final_goal_constraints_[joint_name]) {
            converged_joints = converged_joints + 1;
        }
    }

    ROS_DEBUG("Converged joints = %i of %i", converged_joints, (int)trajectory_input_.joint_names.size());

    if (converged_joints == trajectory_input_.joint_names.size()) {
		ROS_INFO("Joints converged!");
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

// assumes current velocity of all goal joints is 0
void JointTrajectoryExecuter::interpolateTrajectory(const std::vector<JointState>& joint_states_start,
                                                    const trajectory_msgs::JointTrajectory& jt,
                                                    trajectory_msgs::JointTrajectory& jt_interpolated) {


    ROS_INFO("--------------- STILL ---------------");
    for(unsigned int j = 0; j < joint_states_start.size(); ++j) {
        const JointState& state = joint_states_start[j];
        ROS_INFO("Joint %d: pos = %f, vel = %f", j, state.pos, state.vel);
    }

    jt_interpolated.joint_names = jt.joint_names;
    std::vector<JointState> joint_states = joint_states_start;

    for(unsigned int i = 0; i < jt.points.size(); ++i) {
        const std::vector<double>& goal_pos = jt.points[i].positions;

        std::vector<double> times(goal_pos.size());

        // find out how long it will take to reach the goal
        // taking into account acceleration and deceleration
        double max_time = 0;
        for(unsigned int j = 0; j < goal_pos.size(); ++j) {           
            const JointState& state = joint_states[j];

            double diff = std::abs(goal_pos[j] - state.pos);

            double t_acc = state.max_vel / state.max_acc;
            double x_acc = state.max_acc * t_acc * t_acc / 2;

            double time;
            if (x_acc < diff / 2) {
                // reach full velocity
                double x_max_vel = diff - (2 * x_acc);
                time = x_max_vel / state.max_vel + 2 * t_acc;
            } else {
                // do not reach full velocity
                double t_half = sqrt(2 * (diff / 2) / state.max_acc);
                time = 2 * t_half;
            }

            times[j] = time;

            max_time = std::max(max_time, time);
        }
        
        ROS_INFO("--------------- Point %d ---------------", i);

        // scale the maximum velocity and acceleration of each joint based on the longest time
        for(unsigned int j = 0; j < joint_states.size(); ++j) {
            JointState& state = joint_states[j];
            double time_factor = times[j] / max_time;
            state.max_acc = joint_states_start[j].max_acc * time_factor * time_factor;
            state.max_vel = joint_states_start[j].max_vel * time_factor;
            state.vel = 0;
        }

        for(double t = 0; t < max_time; t += DT) {

            trajectory_msgs::JointTrajectoryPoint p;
            for(unsigned int j = 0; j < joint_states.size(); ++j) {
                JointState& state = joint_states[j];

                double v_sign;
                double v_abs = abs(state.vel, v_sign);

                // calculate distance to goal
                double dx = goal_pos[j] - state.pos;
                double dx_sign;
                double dx_abs = abs(dx, dx_sign);

                // calculate deceleration time and distance
                double dt_dec = v_abs / state.max_acc;
                double dx_dec = 0.5 * state.max_acc * dt_dec * dt_dec;

                if (dx_abs <= dx_dec) {
                    // decelerate
                    v_abs = std::max(0.0, v_abs - state.max_acc * DT);
                } else {
                    // accelerate
                    v_abs = std::min(state.max_vel, v_abs + state.max_acc * DT);
                }

                state.vel = dx_sign * v_abs;
                state.pos += state.vel * DT;

                p.positions.push_back(state.pos);
                p.velocities.push_back(state.vel);
            }

            jt_interpolated.points.push_back(p);
        }

        for(unsigned int j = 0; j < joint_states.size(); ++j) {
            const JointState& state = joint_states[j];
            ROS_INFO("Joint %d: pos = %f, goal = %f", j, state.pos, goal_pos[j]);
        }
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

    double t = 0;
    for(unsigned int i = 0; i < jt.points.size(); ++i) {
        for(unsigned int j = 0; j < jt.joint_names.size(); ++j) {
            if (j == torso_joint_index) {
                torso_msg.points[i].positions.push_back(jt.points[i].positions[j]);
                torso_msg.points[i].time_from_start = ros::Duration(t);
            } else {
                arm_msg.points[i].positions.push_back(jt.points[i].positions[j]);
                arm_msg.points[i].time_from_start = ros::Duration(t);
            }
        }
        t += DT;
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
