#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

double ReferenceGenerator::NO_VALUE = -1000; // TODO: make this nicer

// ----------------------------------------------------------------------------------------------------

void interpolateCubic(trajectory_msgs::JointTrajectoryPoint& p_out,
                      const trajectory_msgs::JointTrajectoryPoint& p0,
                      const trajectory_msgs::JointTrajectoryPoint& p1,
                      double t_abs)
{
    double T = (p1.time_from_start - p0.time_from_start).toSec();
    double t = t_abs - p0.time_from_start.toSec();
    unsigned int njoints = p0.positions.size();

    std::vector<double> q(njoints, 0.0);
    std::vector<double> qdot(njoints, 0.0);
    std::vector<double> qddot(njoints, 0.0);

    // Interpolate for every joint
    for(unsigned int k = 0; k < njoints; ++k)
    {
        double a = p0.positions[k];
        double b = p0.velocities[k];
        double c = (-3*p0.positions[k] + 3*p1.positions[k] - 2*T*p0.velocities[k] - T*p1.velocities[k]) / T*T;
        double d = (2*p0.positions[k] - 2*p1.positions[k] + T*p0.velocities[k] + T*p1.velocities[k]) / T*T*T;
        q[k] = a + b*t + c*t*t + d*t*t*t;
        qdot[k] = b + 2*c*t + 3*d*t*t;
        qddot[k] = 2*c + 6*d*t;
    }

    p_out.positions = q;
    p_out.velocities = qdot;
    p_out.accelerations = qddot;
    p_out.time_from_start = ros::Duration(t_abs);
}

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
    num_goal_joints_ = goal.trajectory.joint_names.size();

    joint_index_mapping_.resize(num_goal_joints_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check feasibility of joint goals

    bool goal_ok = true;
    for (unsigned int i = 0; i < num_goal_joints_; ++i)
    {
        const std::string& joint_name = goal.trajectory.joint_names[i];

        int idx = joint_index(joint_name);

        if (idx < 0)
        {
            ss << "Unknown joint: '" << joint_name << "'.\n";
            goal_ok = false;
        }

        joint_index_mapping_[i] = idx;
    }

    if (!goal_ok)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Set goal

    goal_ = goal;
    sub_goal_idx_ = 0;
    time_since_start_ = 0;

    if (goal.trajectory.points.empty())
    {
        is_idle_ = true;
        return true;
    }

    is_idle_ = false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Determine which points are smooth

    is_smooth_point_.reserve(goal.trajectory.points.size());
    is_smooth_point_[0] = false;
    for(unsigned int i = 1; i < goal.trajectory.points.size(); ++i)
    {
        const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[i];

        is_smooth_point_[i] = (sub_goal.velocities.size() == num_goal_joints_
                               && sub_goal.time_from_start.toSec() > 0);
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
            r.reset(positions[i], 0);
        }
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -

    if (is_idle_ || sub_goal_idx_ >= goal_.trajectory.points.size())
    {
        references = positions;
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Go to next unreached sub goal

    while(true)
    {
        const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

        bool sub_goal_reached = false;

        if (is_smooth_point_[sub_goal_idx_])
        {
            if (time_since_start_ > sub_goal.time_from_start.toSec())
                sub_goal_reached = true;
        }
        else
        {
            sub_goal_reached = true;
            for(unsigned int i = 0; i < num_goal_joints_; ++i)
            {
                unsigned int joint_idx = joint_index_mapping_[i];

                if (std::abs(sub_goal.positions[i] - positions[joint_idx]) > 0.01)
                {
                    sub_goal_reached = false;
                    time_since_start_ = sub_goal.time_from_start.toSec();
                    break;
                }
            }
        }

        if (!sub_goal_reached)
        {
            break;
        }
        else
        {
            // If it has been reached, go to the next one
            ++sub_goal_idx_;

            if (sub_goal_idx_ >= goal_.trajectory.points.size())
            {
                // Reached final goal
                references = positions;
                is_idle_ = true;
                return true;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

    references = positions;

    bool use_cubic_interpolation = false;
    if (sub_goal_idx_ > 0 && sub_goal.velocities.size() == num_goal_joints_ && sub_goal.time_from_start.toSec() > 0)
    {
        const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal_.trajectory.points[sub_goal_idx_];

        if (prev_sub_goal.velocities.size() == num_goal_joints_ && prev_sub_goal.time_from_start.toSec() > 0)
        {
            use_cubic_interpolation = true;
            for(unsigned int i = 0; i < num_goal_joints_; ++i)
            {
                trajectory_msgs::JointTrajectoryPoint p_interpolated;
                interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, time_since_start_);

                unsigned int joint_idx = joint_index_mapping_[i];
                references[joint_idx] = p_interpolated.positions[i];
                interpolators_[joint_idx].reset(p_interpolated.positions[i],
                                                p_interpolated.velocities[i]);
            }
        }
    }

    if (!use_cubic_interpolation)
    {
        for(unsigned int i = 0; i < num_goal_joints_; ++i)
        {
            double p_wanted = sub_goal.positions[i];

            unsigned int joint_idx = joint_index_mapping_[i];

            ReferenceInterpolator& r = interpolators_[joint_idx];
            ReferencePoint ref = r.generateReference(p_wanted, max_velocities_[joint_idx],
                                                     max_accelerations_[joint_idx], dt, false, 0.01);
            references[joint_idx] = ref.pos;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

