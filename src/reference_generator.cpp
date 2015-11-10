#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

double ReferenceGenerator::NO_SETPOINT = -1000; // TODO: make this nicer

// ----------------------------------------------------------------------------------------------------

ReferenceGenerator::ReferenceGenerator() : is_idle_(true)
{
}

// ----------------------------------------------------------------------------------------------------

ReferenceGenerator::~ReferenceGenerator()
{
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::initJoint(const std::string& name, double max_vel, double max_acc,
                                       double min_pos, double max_pos)
{
    int idx = this->joint_index(name);
    if (idx >= 0)
    {
        initJoint(idx, max_vel, max_acc, min_pos, max_pos);
    }
    else
    {
        joint_name_to_index_[name] = joint_names_.size();
        joint_names_.push_back(name);
        max_velocities_.push_back(max_vel);
        max_accelerations_.push_back(max_acc);
        min_positions_.push_back(min_pos);
        max_positions_.push_back(max_pos);
    }
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::initJoint(unsigned int idx, double max_vel, double max_acc,
                                       double min_pos, double max_pos)
{
    max_velocities_[idx] = max_vel;
    max_accelerations_[idx] = max_acc;
    min_positions_[idx] = min_pos;
    max_positions_[idx] = max_pos;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::setJointNames(const std::vector<std::string>& joint_names)
{
    joint_names_ = joint_names;

    // Create mapping from joint name to index
    joint_name_to_index_.clear();
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
        joint_name_to_index_[joint_names_[i]] = i;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& goal, std::stringstream& ss)
{
    unsigned int num_goal_joints = goal.trajectory.joint_names.size();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check feasibility of joint goals

    bool goal_ok = true;
    for (unsigned int i = 0; i < num_goal_joints; ++i)
    {
        const std::string& joint_name = goal.trajectory.joint_names[i];

        if (joint_index(joint_name) < 0)
        {
            ss << "Unknown joint: '" << joint_name << "'.\n";
            goal_ok = false;
        }
    }

    if (!goal_ok)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Set goal

    goal_ = goal;
    sub_goal_.clear();
    sub_goal_idx_ = 0;

    if (goal.trajectory.points.empty())
    {
        is_idle_ = true;
    }
    else
    {

        is_idle_ = false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::calculatePositionReferences(const std::vector<double>& positions, double dt,
                                                         std::vector<double>& references)
{   
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Initialize interpolators, if needed

    if (interpolators_.empty())
    {
        interpolators_.resize(positions.size());
        for(unsigned int i = 0; i < interpolators_.size(); ++i)
        {
            ReferenceInterpolator& r = interpolators_[i];
            r.setRefGen(positions[i]);
        }
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -

    if (is_idle_ || goal_.trajectory.points.empty())
    {
        references = positions;
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Check if sub goal has been reached

    bool sub_goal_reached = false;
    if (!sub_goal_.empty())
    {
        sub_goal_reached = true;
        for(unsigned int i = 0; i < positions.size(); ++i)
        {
            if (sub_goal_[i] != NO_SETPOINT && std::abs(positions[i] - sub_goal_[i]) > 0.01)
            {
                sub_goal_reached = false;
                break;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // If it has been reached, go to the next one

    if (sub_goal_reached)
    {
        ++sub_goal_idx_;
        sub_goal_.clear();

        if (sub_goal_idx_ >= goal_.trajectory.points.size())
        {
            // Reached final goal
            references = positions;
            is_idle_ = true;
            return true;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine sub goal (if necessary)

    if (sub_goal_.empty())
    {
        sub_goal_.resize(positions.size(), NO_SETPOINT);
        for(unsigned int i = 0; i < goal_.trajectory.joint_names.size(); ++i)
        {
            const std::string& joint_name = goal_.trajectory.joint_names[i];
            int joint_idx = joint_index(joint_name);
            if (joint_idx >= 0)
            {
                sub_goal_[joint_idx] = goal_.trajectory.points[sub_goal_idx_].positions[i];
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    if (references.size() != positions.size())
        references.resize(positions.size());

    for(unsigned int i = 0; i < positions.size(); ++i)
    {
        ReferenceInterpolator& r = interpolators_[i];

        double p_wanted = sub_goal_[i];
        if (p_wanted == NO_SETPOINT)
        {
            references[i] = positions[i];
        }
        else
        {
            PosVelAcc ref = r.generateReference(p_wanted, max_velocities_[i], max_accelerations_[i], dt, false, 0.01);
            references[i] = ref.pos;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

