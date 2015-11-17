#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

double ReferenceGenerator::NO_VALUE = -1000; // TODO: make this nicer

// ----------------------------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& out, const std::vector<double>& v)
{
    if (v.empty())
        return out;

    out << v[0];
    for(unsigned int i = 1; i < v.size(); ++i)
        out << " " << v[i];

    return out;
}

// ----------------------------------------------------------------------------------------------------

// Interpolate using Hermite curve
void interpolateCubic(trajectory_msgs::JointTrajectoryPoint& p_out,
                      const trajectory_msgs::JointTrajectoryPoint& p0,
                      const trajectory_msgs::JointTrajectoryPoint& p1,
                      double t_abs)
{
    double T = (p1.time_from_start - p0.time_from_start).toSec();
    double t = t_abs - p0.time_from_start.toSec();
    unsigned int njoints = p0.positions.size();

    p_out.positions.resize(njoints);
    p_out.velocities.resize(njoints);
    p_out.accelerations.resize(njoints);

    // Transform time to [0, 1]
    double f = t / T;

    // Pre-calculate some things
    double f2 = f * f;
    double f3 = f * f2;

    // Interpolate for every joint
    for(unsigned int k = 0; k < njoints; ++k)
    {
        p_out.positions[k] = (2 * f3 - 3 * f2 + 1) * p0.positions[k]
                + (f3 - 2 * f2 + f) * (p0.velocities[k] * T)
                + (-2 * f3 + 3 * f2) * p1.positions[k]
                + (f3 - f2) * (p1.velocities[k] * T);

        p_out.velocities[k] = (6 * f2 - 6 * f) * p0.positions[k] / T
                + (3 * f2 - 4 * f + 1) * p0.velocities[k]
                + (-6 * f2 + 6 * f) * p1.positions[k] / T
                + (3 * f2 - 2 * f) * p1.velocities[k];

        p_out.accelerations[k] = ((12 * f - 6) * p0.positions[k]
                + (6 * f - 4) * (p0.velocities[k] * T)
                + (-12 * f + 6) * p1.positions[k]
                + (6 * f - 2) * (p1.velocities[k] * T) / T); // Not sure this is right!
    }

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
        interpolators_.push_back(ReferenceInterpolator());

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

bool ReferenceGenerator::setJointState(const std::string& joint_name, double pos, double vel)
{
    std::cout << "Lets set " << joint_name << " to " << pos << " and " << vel << std::endl;
    int idx = this->joint_index(joint_name);
    std::cout << "idx = " << idx << std::endl;
    if (idx < 0)
        return false;

    ReferenceInterpolator& r = interpolators_[idx];
    std::cout << "rrrrrrr" << std::endl;
    r.reset(pos, vel);
    std::cout << "eset!" << std::endl;
    is_idle_ = true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& goal, std::stringstream& ss)
{
    std::cout << "Welcome to the wonderful world of interpolation!" << std::endl;
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

    std::cout << "Congratulation, you passed our initial screening" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Set goal

    goal_ = goal;
    sub_goal_idx_ = 0;
    time_since_start_ = 0;

//    graph_viewer_.clear();

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
    time_since_start_ += dt;

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
        bool sub_goal_reached = false;

        const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

        if (is_smooth_point_[sub_goal_idx_] && is_smooth_point_[sub_goal_idx_ - 1])
        {
            if (time_since_start_ >= sub_goal.time_from_start.toSec())
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
                    break;
                }
            }

            if (sub_goal_reached)
                time_since_start_ = sub_goal.time_from_start.toSec();
        }

        if (sub_goal_reached)
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

//            graph_viewer_.addPoint(0, 1, time_since_start_, sub_goal.positions[0], sub_goal.velocities[0]);

        }
        else
        {
            break;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    references = positions;

    const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

    if (is_smooth_point_[sub_goal_idx_] && is_smooth_point_[sub_goal_idx_ - 1])
    {
//        std::cout << "CUBIC!" << std::endl;

        // Use cubic interpolation

        const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal_.trajectory.points[sub_goal_idx_ - 1];

        trajectory_msgs::JointTrajectoryPoint p_interpolated;
        interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, time_since_start_);

        for(unsigned int i = 0; i < num_goal_joints_; ++i)
        {
            unsigned int joint_idx = joint_index_mapping_[i];
            references[joint_idx] = p_interpolated.positions[i];
            interpolators_[joint_idx].reset(p_interpolated.positions[i],
                                            p_interpolated.velocities[i]);
        }

//        graph_viewer_.addPoint(0, 0, time_since_start_, p_interpolated.positions[0], p_interpolated.velocities[0]);
    }
    else
    {
//        graph_viewer_.clear();

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

//    graph_viewer_.view();

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
