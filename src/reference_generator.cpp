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

ReferenceGenerator::ReferenceGenerator() : next_goal_id_(0)
{
    visualize_ = false;
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
    j.interpolator.setMaxVelocity(max_vel);
    j.interpolator.setMaxAcceleration(max_acc);
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
    j.interpolator.setState(pos, vel);
    j.is_idle = true;
    j.is_set = true;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& goal_msg, std::string& id, std::stringstream& ss)
{
//    std::cout << "ReferenceGenerator::setGoal" << std::endl;

//    for (unsigned int i = 0; i < goal_msg.trajectory.points.size(); ++i)
//    {
//        const trajectory_msgs::JointTrajectoryPoint& p = goal_msg.trajectory.points[i];
//        std::cout << "  " << i << ": t = " << p.time_from_start.toSec() << ", pos = " << p.positions << ", vel = " << p.velocities << std::endl;
//    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (id.empty())
    {
        std::stringstream s;
        s << "goal-" << (next_goal_id_++);
        id = s.str();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (goals_.find(id) != goals_.end())
    {
        ss << "Goal with id '" << id << " already exists.\n";
        return false;
    }

    JointGoal& goal = goals_[id];
    goal.goal_msg = goal_msg;
    goal.num_goal_joints = goal_msg.trajectory.joint_names.size();
    goal.joint_index_mapping.resize(goal.num_goal_joints);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check feasibility of joint goals

    bool goal_ok = true;
    for (unsigned int i = 0; i < goal.num_goal_joints; ++i)
    {
        const std::string& joint_name = goal_msg.trajectory.joint_names[i];

        int idx = joint_index(joint_name);        

        if (idx < 0)
        {
            ss << "Unknown joint: '" << joint_name << "'.\n";
            goal_ok = false;
            continue;
        }

        const JointInfo& js = joint_info_[idx];

        if (!js.is_idle)
        {
            ss << "Joint '" << joint_name << "' is busy.\n";
            goal_ok = false;
        }

        if (!js.is_set)
        {
            ss << "Joint '" << joint_name << "' initial position and velocity is not set.\n";
            goal_ok = false;
        }

        if (js.max_vel == 0 || js.max_acc == 0 || (js.min_pos == js.max_pos))
        {
            ss << "Joint '" << joint_name << "' limits not initialized.\n";
            goal_ok = false;
        }

        goal.joint_index_mapping[i] = idx;
    }

    if (!goal_ok)
    {
        goals_.erase(id);
        return false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check if joint goals go out of limits

    for (unsigned int i = 0; i < goal_msg.trajectory.points.size(); ++i)
    {
        const trajectory_msgs::JointTrajectoryPoint& p = goal_msg.trajectory.points[i];

        for(unsigned int j = 0; j < goal.num_goal_joints; ++j)
        {
            const JointInfo& js = joint_info_[goal.joint_index_mapping[j]];

            double pos = p.positions[j];
            if (pos < js.min_pos || pos > js.max_pos)
            {
                ss << "Joint '" << joint_name(j) << "' goes out of limits in point " << i << " "
                   << "(min = " << js.min_pos << ", max = " << js.max_pos << ", requested goal = " << pos << ").\n";
                goal_ok = false;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (!goal_ok)
    {
        goals_.erase(id);
        return false;
    }

    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        joint_info_[goal.joint_index_mapping[i]].is_idle = false;

    goal.sub_goal_idx = -1;
    goal.time_since_start = 0;
    goal.use_cubic_interpolation = false;

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
//    // Determine which points are smooth

//    std::cout << goal.goal_msg << std::endl;

//    for(unsigned int i = 1; i < goal.goal_msg.trajectory.points.size(); ++i)
//        std::cout << i << ": " << isSmoothPoint(goal, i) << std::endl;

    graph_viewer_.clear();
    time_ = 0;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const std::string& joint_name, double position)
{
    control_msgs::FollowJointTrajectoryGoal goal_msg;
    goal_msg.trajectory.joint_names.push_back(joint_name);

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(position);
    goal_msg.trajectory.points.push_back(p);

    std::string id;
    std::stringstream error;
    return setGoal(goal_msg, id, error);
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const std::vector<std::string>& joint_names, const std::vector<double>& positions)
{
    if (joint_names.empty() || joint_names.size() != positions.size())
        return false;

    control_msgs::FollowJointTrajectoryGoal goal_msg;
    goal_msg.trajectory.joint_names = joint_names;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions = positions;
    goal_msg.trajectory.points.push_back(p);

    std::string id;
    std::stringstream error;
    return setGoal(goal_msg, id, error);
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::cancelGoal(const std::string& id)
{
    std::map<std::string, JointGoal>::iterator it = goals_.find(id);
    if (it == goals_.end())
        return;

    JointGoal& goal = it->second;

    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        joint_info_[goal.joint_index_mapping[i]].is_idle = true;

    goal.is_done = true;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::calculatePositionReferences(JointGoal& goal, double dt, std::vector<double>& references)
{
    goal.time_since_start += dt;
    time_ += dt;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Go to next unreached sub goal

    for(bool sub_goal_reached = true; sub_goal_reached; )
    {
        if (goal.sub_goal_idx < 0)
        {
            sub_goal_reached = true;
        }
        else
        {
            const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

            if (goal.use_cubic_interpolation)
            {
                sub_goal_reached = (goal.time_since_start >= sub_goal.time_from_start.toSec());
            }
            else
            {
                sub_goal_reached = true;
                for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                {
                    unsigned int joint_idx = goal.joint_index_mapping[i];

                    if (!joint_info_[joint_idx].interpolator.done())
                    {
                        sub_goal_reached = false;
                        break;
                    }
                }
            }
        }

        if (sub_goal_reached)
        {
            // If it has been reached, go to the next one
            ++goal.sub_goal_idx;

            if (goal.sub_goal_idx >= goal.goal_msg.trajectory.points.size())
            {
                for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                    joint_info_[goal.joint_index_mapping[i]].is_idle = true;

                goal.is_done = true;
                return;
            }

            // If the velocities for the next and previous point are defined, use cubic interpolation
            if (goal.sub_goal_idx > 0
                    && goal.goal_msg.trajectory.points[goal.sub_goal_idx].velocities.size() == goal.num_goal_joints
                    && goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1].velocities.size() == goal.num_goal_joints)
            {
                goal.use_cubic_interpolation = true;
                goal.time_since_start = goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1].time_from_start.toSec();
            }
            else
            {
                goal.use_cubic_interpolation = false;

                for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                {
                    unsigned int joint_idx = goal.joint_index_mapping[i];
                    joint_info_[joint_idx].interpolator.setGoal(goal.goal_msg.trajectory.points[goal.sub_goal_idx].positions[i]);
                }
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

    if (goal.use_cubic_interpolation)
    {
        // Use cubic interpolation

//        std::cout << "cubic: " << goal.sub_goal_idx << ": " << goal.time_since_start << " / " << sub_goal.time_from_start.toSec() << std::endl;

        const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1];

        trajectory_msgs::JointTrajectoryPoint p_interpolated;
        interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, goal.time_since_start);

        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];
            references[joint_idx] = p_interpolated.positions[i];
            joint_info_[joint_idx].interpolator.setState(p_interpolated.positions[i], p_interpolated.velocities[i]);
        }
    }
    else
    {
//        std::cout << "normal: " << goal.sub_goal_idx << ": " << goal.time_since_start << " / " << sub_goal.time_from_start.toSec() << std::endl;

        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];

            JointInfo& j = joint_info_[joint_idx];

            j.interpolator.update(dt);
            references[joint_idx] = j.interpolator.position();
        }
    }

    if (visualize_)
    {
        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];
            graph_viewer_.addPoint(0, i, time_, joint_info_[joint_idx].interpolator.position());
        }
    }
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::calculatePositionReferences(double dt, std::vector<double>& references)
{
    if (references.size() != joint_info_.size())
        references.resize(joint_info_.size());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(std::map<std::string, JointGoal>::iterator it = goals_.begin(); it != goals_.end(); ++it)
    {
        JointGoal& goal = it->second;

        if (goal.is_done)
            continue;

        calculatePositionReferences(goal, dt, references);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int i = 0; i < joint_info_.size(); ++i)
    {
        JointInfo& j = joint_info_[i];
        if (j.is_idle)
        {
            ReferenceInterpolator& r = j.interpolator;
            if (std::abs(r.velocity()) > 0)
                r.brake(dt);

            references[i] = j.interpolator.position();
        }
    }

    if (visualize_)
        graph_viewer_.view();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
