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

typedef std::vector<double> JointState;
typedef std::vector<JointState > JointTrajectory;
typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
typedef JTAS::GoalHandle GoalHandle;


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

    std::map<std::string, double> current_joint_pos_;
    std::map<std::string, double> intermediate_goal_constraints_;
    std::map<std::string, double> final_goal_constraints_;
    std::map<std::string, double> trajectory_constraints_;
    std::map<std::string, double> max_vel_constraints_;
    std::map<std::string, double> max_acc_constraints_;
    double goal_time_constraint_;

    trajectory_msgs::JointTrajectory trajectory_input_;

    void sendStop();

    void goalCB(GoalHandle gh);

    void cancelCB(GoalHandle gh);

    void measurementCB(const sensor_msgs::JointState& joint_meas);

    void diagnosticsCB(const diagnostic_msgs::DiagnosticArray& diag_array);

    void interpolateTrajectory(const JointState& current_pos, const JointState& max_vel, const JointState& max_acc,
                               const trajectory_msgs::JointTrajectory& joint_trajectory,
                               trajectory_msgs::JointTrajectory& joint_trajectory_interpolated);

    void publishReference(const trajectory_msgs::JointTrajectory& joint_trajector);
};

#endif
