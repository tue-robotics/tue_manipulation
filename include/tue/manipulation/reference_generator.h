#ifndef TUE_MANIPULATION_TRAJECTORY_ACTION_SERVER_H_
#define TUE_MANIPULATION_TRAJECTORY_ACTION_SERVER_H_

#include <control_msgs/FollowJointTrajectoryAction.h>

#include "tue/manipulation/reference_interpolator.h"
//#include "tue/manipulation/graph_viewer.h"

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

struct JointGoal
{
    double time_since_start;

    unsigned int sub_goal_idx;

    // Maps joint indices in the goal to indices in the internal representation
    std::vector<unsigned int> joint_index_mapping;

    control_msgs::FollowJointTrajectoryGoal goal_msg;

    std::vector<bool> is_smooth_point;

    unsigned int num_goal_joints;
};

// ----------------------------------------------------------------------------------------------------

struct JointInfo
{
    JointInfo() : max_vel(0), max_acc(0), min_pos(0), max_pos(0), is_idle(true), is_initialized(false) {}

    double max_vel;
    double max_acc;
    double min_pos;
    double max_pos;
    bool is_idle;
    bool is_initialized;

    ReferenceInterpolator interpolator;
};

// ----------------------------------------------------------------------------------------------------

class ReferenceGenerator
{

public:

    ReferenceGenerator();

    ~ReferenceGenerator();

    void setJointNames(const std::vector<std::string>& joint_names);

    void initJoint(const std::string& name, double max_vel, double max_acc, double min_pos, double max_pos);

    void initJoint(unsigned int idx, double max_vel, double max_acc, double min_pos, double max_pos);

    void setPositionLimits(unsigned int idx, double min_pos, double max_pos)
    {
        joint_info_[idx].min_pos = min_pos;
        joint_info_[idx].max_pos = max_pos;
    }

    void setMaxVelocity(unsigned int idx, double max_vel)
    {
        joint_info_[idx].max_vel = max_vel;
    }

    void setMaxAcceleration(unsigned int idx, double max_acc)
    {
        joint_info_[idx].max_acc = max_acc;
    }

    bool setJointState(const std::string& joint_name, double pos, double vel);

    bool setGoal(const control_msgs::FollowJointTrajectoryGoal& goal, std::stringstream& ss);

    bool calculatePositionReferences(double dt, std::vector<double>& references);

    // Returns joint index for a given joint name. If joint does not exist, returns -1
    int joint_index(const std::string& name) const
    {
        std::map<std::string, unsigned int>::const_iterator it = joint_name_to_index_.find(name);
        if (it == joint_name_to_index_.end())
            return -1;
        return it->second;
    }

    const std::string& joint_name(unsigned int idx) const
    {
        return joint_names_[idx];
    }

    const std::vector<std::string>& joint_names() const { return joint_names_; }

    bool all_joints_idle() const
    {
        for(std::vector<JointInfo>::const_iterator it = joint_info_.begin(); it != joint_info_.end(); ++it)
            if (!it->is_idle)
                return false;

        return true;
    }

private:

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Joint information

    std::vector<std::string> joint_names_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    std::vector<JointInfo> joint_info_;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Goal trajectory / set points

    std::vector<JointGoal> goals_;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    GraphViewer graph_viewer_;

};

} // end namespace tue

} // end namespace manipulation

#endif
