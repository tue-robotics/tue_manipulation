#ifndef AMIGO_ARM_NAVIGATION_REFERENCE_GENERATOR_H_
#define AMIGO_ARM_NAVIGATION_REFERENCE_GENERATOR_H_

// Author: Stuart Glaser && Rob Janssen && Janno Lunenburg && Sjoerd van den Dries

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

// Input
#include <control_msgs/FollowJointTrajectoryAction.h>

// Output
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Float64.h>

#include <diagnostic_msgs/DiagnosticArray.h>

using namespace std;

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
typedef JTAS::GoalHandle GoalHandle;

struct JointState {
    JointState() : pos_initialized(false), vel_initialized(false) {}

    double pos, vel;
    bool pos_initialized, vel_initialized;
    double max_acc;
    double max_vel;
    double t_last_update;
};


class JointTrajectoryExecuter {

public:

    JointTrajectoryExecuter(ros::NodeHandle &n);

    ~JointTrajectoryExecuter();

private:

    ros::NodeHandle node_;
    JTAS action_server_;
    ros::Publisher arm_pub;
    ros::Subscriber arm_sub_;

    ros::Publisher torso_pub;
    ros::Subscriber torso_sub;
    ros::Subscriber diag_sub;

    unsigned int torso_status, arm_status;
    std::string torso_diag_name; // Name of the torso in the diagnostics message array
    std::string arm_diag_name;   // Name of the arm in the diagnostics message array

    ros::Time t_start_;

    bool has_active_goal_;
    GoalHandle active_goal_;

    std::map<std::string, JointState> joint_measurements_;
    std::map<std::string, double> intermediate_goal_constraints_;
    std::map<std::string, double> final_goal_constraints_;
    std::map<std::string, double> trajectory_constraints_;
//    std::map<std::string, double> max_vel_constraints_;
//    std::map<std::string, double> max_acc_constraints_;
    double goal_time_constraint_;

    trajectory_msgs::JointTrajectory trajectory_input_;

    void goalCB(GoalHandle gh);

    void cancelCB(GoalHandle gh);

    void measurementCB(const sensor_msgs::JointState& joint_meas);

    void diagnosticsCB(const diagnostic_msgs::DiagnosticArray& diag_array);

    void interpolateTrajectory(const std::vector<JointState>& joint_infos,
                               const trajectory_msgs::JointTrajectory& jt,
                               trajectory_msgs::JointTrajectory& jt_interpolated);

    void fillStop(trajectory_msgs::JointTrajectory& traj_msg, double dt, std::vector<JointState>& end_joint_states);

    void publishReference(const trajectory_msgs::JointTrajectory& joint_trajector);
};

#endif
