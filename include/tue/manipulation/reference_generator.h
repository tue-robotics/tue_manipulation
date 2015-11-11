#ifndef TUE_MANIPULATION_TRAJECTORY_ACTION_SERVER_H_
#define TUE_MANIPULATION_TRAJECTORY_ACTION_SERVER_H_

#include <control_msgs/FollowJointTrajectoryAction.h>

#include "tue/manipulation/reference_interpolator.h"

namespace tue
{
namespace manipulation
{

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
        min_positions_[idx] = min_pos;
        max_positions_[idx] = max_pos;
    }

    void setMaxVelocity(unsigned int idx, double max_vel)
    {
        max_velocities_[idx] = max_vel;
    }

    void setMaxAcceleration(unsigned int idx, double max_acc)
    {
        max_accelerations_[idx] = max_acc;
    }

    bool setGoal(const control_msgs::FollowJointTrajectoryGoal& goal, std::stringstream& ss);

    bool calculatePositionReferences(const std::vector<double>& positions, double dt,
                                     std::vector<double>& references);

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

    bool is_idle() const { return is_idle_; }

private:

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Joint information

    std::vector<std::string> joint_names_;

    std::map<std::string, unsigned int> joint_name_to_index_;

    std::vector<double> max_velocities_;

    std::vector<double> max_accelerations_;

    std::vector<double> min_positions_;

    std::vector<double> max_positions_;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Goal trajectory / set points

    double time_since_start_;

    unsigned int sub_goal_idx_;

    // Maps joint indices in the goal to indices in the internal representation
    std::vector<unsigned int> joint_index_mapping_;

    control_msgs::FollowJointTrajectoryGoal goal_;

    std::vector<bool> is_smooth_point_;

    unsigned int num_goal_joints_;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Joint state and interpolation

    bool is_idle_;

    std::vector<ReferenceInterpolator> interpolators_;

    static double NO_VALUE;

};

} // end namespace tue

} // end namespace manipulation

#endif
