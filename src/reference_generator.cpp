#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

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

ReferenceGenerator::ReferenceGenerator()
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
    if (idx < 0)
    {
        joint_name_to_index_[name] = joint_names_.size();
        joint_names_.push_back(name);
        idx = joint_info_.size();
        joint_info_.push_back(JointInfo());
    }

    initJoint(idx, max_vel, max_acc, min_pos, max_pos);
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::initJoint(unsigned int idx, double max_vel, double max_acc,
                                   double min_pos, double max_pos)
{
    JointInfo& j = joint_info_[idx];
    j.max_vel = max_vel;
    j.max_acc = max_acc;
    j.min_pos = min_pos;
    j.max_pos = max_pos;
    j.is_idle = true;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::setJointNames(const std::vector<std::string>& joint_names)
{
    joint_names_ = joint_names;

    // Create mapping from joint name to index
    joint_name_to_index_.clear();
    for(unsigned int i = 0; i < joint_names_.size(); ++i)
        joint_name_to_index_[joint_names_[i]] = i;

    joint_info_.resize(joint_names_.size(), JointInfo());
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setJointState(const std::string& joint_name, double pos, double vel)
{
    int idx = this->joint_index(joint_name);
    if (idx < 0)
        return false;

    JointInfo& j = joint_info_[idx];
    j.interpolator.reset(pos, vel);
    j.is_idle = true;
    j.is_initialized = true;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& goal, std::stringstream& ss)
{
    JointGoal new_goal;
    new_goal.goal_msg = goal;
    new_goal.num_goal_joints = goal.trajectory.joint_names.size();
    new_goal.joint_index_mapping.resize(new_goal.num_goal_joints);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check feasibility of joint goals

    bool goal_ok = true;
    for (unsigned int i = 0; i < new_goal.num_goal_joints; ++i)
    {
        const std::string& joint_name = goal.trajectory.joint_names[i];

        int idx = joint_index(joint_name);

        if (idx < 0)
        {
            ss << "Unknown joint: '" << joint_name << "'.\n";
            goal_ok = false;
        }

        if (!joint_info_[idx].is_idle)
        {
            ss << "Joint '" << joint_name << "' is busy.\n";
            goal_ok = false;
        }

        if (!joint_info_[idx].is_initialized)
        {
            ss << "Joint '" << joint_name << "' is not initialized.\n";
            goal_ok = false;
        }

        new_goal.joint_index_mapping[i] = idx;
    }

    if (!goal_ok)
        return false;

    //    graph_viewer_.clear();

    if (new_goal.goal_msg.trajectory.points.empty())
    {
        for(unsigned int i = 0; i < new_goal.num_goal_joints; ++i)
            joint_info_[new_goal.joint_index_mapping[i]].is_idle = true;
        return true;
    }

    new_goal.sub_goal_idx = 0;
    new_goal.time_since_start = 0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Determine which points are smooth

    new_goal.is_smooth_point.reserve(goal.trajectory.points.size());
    new_goal.is_smooth_point[0] = false;
    for(unsigned int i = 1; i < new_goal.goal_msg.trajectory.points.size(); ++i)
    {
        const trajectory_msgs::JointTrajectoryPoint& sub_goal = new_goal.goal_msg.trajectory.points[i];

        new_goal.is_smooth_point[i] = (sub_goal.velocities.size() == new_goal.num_goal_joints
                                       && sub_goal.time_from_start.toSec() > 0);
    }

    goals_.push_back(new_goal);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::calculatePositionReferences(double dt, std::vector<double>& references)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int i = 0; i < joint_info_.size(); ++i)
    {
        JointInfo& j = joint_info_[i];
        if (j.is_idle)
            references[i] = j.interpolator.position();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(std::vector<JointGoal>::iterator it = goals_.begin(); it != goals_.end(); ++it)
    {
        JointGoal& goal = *it;

        goal.time_since_start += dt;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
        // Go to next unreached sub goal

        while(true)
        {
            bool sub_goal_reached = false;

            const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

            if (goal.is_smooth_point[goal.sub_goal_idx] && goal.is_smooth_point[goal.sub_goal_idx - 1])
            {
                if (goal.time_since_start >= sub_goal.time_from_start.toSec())
                    sub_goal_reached = true;
            }
            else
            {
                sub_goal_reached = true;
                for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                {
                    unsigned int joint_idx = goal.joint_index_mapping[i];

                    if (std::abs(sub_goal.positions[i] - joint_info_[joint_idx].interpolator.position()) > 0.01)
                    {
                        sub_goal_reached = false;
                        break;
                    }
                }

                if (sub_goal_reached)
                    goal.time_since_start = sub_goal.time_from_start.toSec();
            }

            if (sub_goal_reached)
            {
                // If it has been reached, go to the next one
                ++goal.sub_goal_idx;

                if (goal.sub_goal_idx >= goal.goal_msg.trajectory.points.size())
                {
                    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                        joint_info_[goal.joint_index_mapping[i]].is_idle = true;

                    return true;
                }

//                graph_viewer_.addPoint(0, 1, time_since_start_, sub_goal.positions[0], sub_goal.velocities[0]);

            }
            else
            {
                break;
            }
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Determine references using interpolation

        const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

        if (goal.is_smooth_point[goal.sub_goal_idx] && goal.is_smooth_point[goal.sub_goal_idx - 1])
        {
            //        std::cout << "CUBIC!" << std::endl;

            // Use cubic interpolation

            const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1];

            trajectory_msgs::JointTrajectoryPoint p_interpolated;
            interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, goal.time_since_start);

            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
            {
                unsigned int joint_idx = goal.joint_index_mapping[i];
                references[joint_idx] = p_interpolated.positions[i];
                joint_info_[joint_idx].interpolator.reset(p_interpolated.positions[i], p_interpolated.velocities[i]);
            }

            //        graph_viewer_.addPoint(0, 0, time_since_start_, p_interpolated.positions[0], p_interpolated.velocities[0]);
        }
        else
        {
            //        graph_viewer_.clear();

            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
            {
                double p_wanted = sub_goal.positions[i];

                unsigned int joint_idx = goal.joint_index_mapping[i];

                JointInfo& j = joint_info_[joint_idx];

                ReferencePoint ref = j.interpolator.generateReference(p_wanted, j.max_vel, j.max_pos, dt, false, 0.01);
                references[joint_idx] = ref.pos;
            }
        }
    }

    //    graph_viewer_.view();

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
