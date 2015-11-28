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

    if (visualize_)
    {
        time_ = 0;
        graph_vis_pos_.setName("position");
        graph_vis_vel_.setName("velocity");
        graph_vis_acc_.setName("acceleration");
    }
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
    j.goal_id.clear();
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
    j.goal_id.clear();
    j.is_set = true;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& goal_msg, std::string& id, std::stringstream& ss)
{
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

    std::set<std::string> goals_to_cancel;

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

        if (!js.goal_id.empty())
            goals_to_cancel.insert(js.goal_id);

        if (!js.is_set)
        {
            ss << "Joint '" << joint_name << "' initial position and velocity is not set.\n";
            goal_ok = false;
        }

        if (js.max_vel == 0 || js.max_acc == 0 || (js.min_pos == js.max_pos))
        {
            ss << "Joint '" << joint_name << "' limits not initialized: "
               << "max vel = " << js.max_vel << ", max acc = " << js.max_acc
               << ", min pos = " << js.min_pos << ", max pos = " << js.max_pos << "\n";
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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Cancel overlapping goals

    for(std::set<std::string>::const_iterator it = goals_to_cancel.begin(); it != goals_to_cancel.end(); ++it)
        cancelGoal(*it);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        joint_info_[goal.joint_index_mapping[i]].goal_id = id;

    goal.sub_goal_idx = -1;
    goal.time_since_start = 0;
    goal.use_cubic_interpolation = false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const std::string& joint_name, double position, JointGoalInfo& info)
{
    control_msgs::FollowJointTrajectoryGoal goal_msg;
    goal_msg.trajectory.joint_names.push_back(joint_name);

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(position);
    goal_msg.trajectory.points.push_back(p);

    return setGoal(goal_msg, info.id, info.s_error);
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::setGoal(const std::vector<std::string>& joint_names, const std::vector<double>& positions,
                                 JointGoalInfo& info)
{
    if (joint_names.empty() || joint_names.size() != positions.size())
        return false;

    control_msgs::FollowJointTrajectoryGoal goal_msg;
    goal_msg.trajectory.joint_names = joint_names;

    trajectory_msgs::JointTrajectoryPoint p;
    p.positions = positions;
    goal_msg.trajectory.points.push_back(p);

    return setGoal(goal_msg, info.id, info.s_error);
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::cancelGoal(const std::string& id)
{
    std::map<std::string, JointGoal>::iterator it = goals_.find(id);
    if (it == goals_.end())
        return;

    JointGoal& goal = it->second;

    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        joint_info_[goal.joint_index_mapping[i]].goal_id.clear();

    goal.status = JOINT_GOAL_CANCELED;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::calculatePositionReferences(JointGoal& goal, double dt)
{
    time_ += dt;
    goal.time_since_start += dt;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Check if the sub goal is reached

    bool sub_goal_reached = false;
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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // If so, go to next unreached sub goal

    if (sub_goal_reached)
    {
        // If it has been reached, go to the next one
        ++goal.sub_goal_idx;

        // Check if this was the last trajectory point. If so, this goal is finished!
        if (goal.sub_goal_idx >= goal.goal_msg.trajectory.points.size())
        {
            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                joint_info_[goal.joint_index_mapping[i]].goal_id.clear();

            std::cout << "Goal reached in " << goal.time_since_start << " seconds" << std::endl;

            goal.status = JOINT_GOAL_SUCCEEDED;
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
            const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

            // Let's do some smoothing! We don't want to decelerate to 0 for each sub goal. However, we did not receive any
            // intermediate velocities or timestamps in the given goal, so we have to do some calculation of our own.

            // First determine for each joint the maximum velocity we are allowed to have when reaching the next sub goal,
            // such that we can still fully brake to 0 velocity in the goal after that.

            std::vector<double> sub_goal_velocities(goal.num_goal_joints, 0);
            if (goal.sub_goal_idx + 1 < goal.goal_msg.trajectory.points.size())
            {
                for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                {
                    const JointInfo& js = joint_info_[goal.joint_index_mapping[i]];

                    double v0 = js.velocity();          // Current velocity
                    double x0 = js.position();          // Current position
                    double x1 = sub_goal.positions[i];  // Next sub goal position
                    double x2 = goal.goal_msg.trajectory.points[goal.sub_goal_idx + 1].positions[i];  // Sub goal position after that

                    // Check if x0, x1 and x2 are going in the same direction. If not, the velocity has to be 0 in x1 because
                    // we have to change direction there.
                    if ((x0 < x1) == (x1 < x2))
                    {
                        // Calculate the maximum velocity we can reach from x0 to x1
                        double v_max_01 = sqrt(2 * js.max_acc * std::abs(x1 - x0) + v0 * v0);

                        // Calculate the maximum velocity we are allowed to have such that we can still reach x2 with 0 velocity
                        double v_max_12 = sqrt(2 * js.max_acc * std::abs(x2 - x1));

                        sub_goal_velocities[i] = std::min(js.max_vel, std::min(v_max_01, v_max_12));

                        if (x2 < x1)
                            sub_goal_velocities[i] = -sub_goal_velocities[i];
                    }
                }
            }

            // Now given our current joint positions and velocities, the sub goal positions and the sub goal velocities we
            // just calculated, calculate the time needed for each joint to reach the sub goal position and velocity, and
            // remember the longest time.

            double time = 0;
            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
            {
                JointInfo& js = joint_info_[goal.joint_index_mapping[i]];
                time = std::max<double>(time, js.interpolator.calculateTimeNeeded(sub_goal.positions[i], sub_goal_velocities[i]));
            }

            // Now just give each individual joint the calculated sub goal velocity and position and goal time calculated above.
            // There is one problem: it might be the case that the max time calculated is too long for the joint to reach the
            // given sub goal position and velocity. For example, it might have to brake to take more time, but then not have enough
            // position margin left to accerelate to the sub goal velocity (maybe a bit hard to grasp, but think about it for a
            // while...). Therefore we need to check for each joint if the calculated sub goal and time is still feasible. If not,
            // we do a dirty trick: we lower the sub goal velocity a bit, see if that alters the max time, and try again. We do this
            // in an iterative fashion until all joint goals are reachable.
            // Don't worry: in most cases repetition is not needed and if it is, the number of iterations will be quite low.
            while (true)
            {
                bool all_goals_ok = true;

                for(unsigned int i = 0; i < goal.num_goal_joints && all_goals_ok; ++i)
                {
                    JointInfo& js = joint_info_[goal.joint_index_mapping[i]];

                    while(!js.interpolator.setGoal(sub_goal.positions[i], sub_goal_velocities[i], time))
                    {
                        // Whoops, we can't reach the goal in the time given! Let's lower the sub goal velocity and try again
                        sub_goal_velocities[i] *= 0.9;

                        // Before trying again, we check if the time needed has changed.
                        double new_joint_time = js.interpolator.calculateTimeNeeded(sub_goal.positions[i], sub_goal_velocities[i]);

                        if (new_joint_time > time)
                        {
                            // If now the joint needs longer than the current max time, we have to recalculate the max time
                            // and repeat the process for all joints
                            time = std::max(new_joint_time, time);
                            all_goals_ok = false;
                            break;
                        }
                    }
                }

                if (all_goals_ok)
                    break;
            }
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

    if (goal.use_cubic_interpolation)
    {
        const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1];

        trajectory_msgs::JointTrajectoryPoint p_interpolated;
        interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, goal.time_since_start);

        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];
            joint_info_[joint_idx].interpolator.setState(p_interpolated.positions[i], p_interpolated.velocities[i]);
        }
    }
    else
    {
        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];
            JointInfo& j = joint_info_[joint_idx];
            j.interpolator.update(dt);
        }
    }

    if (visualize_)
    {
        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            unsigned int joint_idx = goal.joint_index_mapping[i];
            graph_vis_pos_.addPoint(0, joint_idx, time_, joint_info_[joint_idx].position());
            graph_vis_vel_.addPoint(0, joint_idx, time_, joint_info_[joint_idx].velocity());
            graph_vis_acc_.addPoint(0, joint_idx, time_, joint_info_[joint_idx].acceleration());
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

        if (goal.status != JOINT_GOAL_ACTIVE)
            continue;

        calculatePositionReferences(goal, dt);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int i = 0; i < joint_info_.size(); ++i)
    {
        JointInfo& j = joint_info_[i];
        if (j.goal_id.empty())
        {
            ReferenceInterpolator& r = j.interpolator;
            if (std::abs(r.velocity()) > 0)
                r.brake(dt);
        }

        references[i] = j.interpolator.position();
    }

    if (visualize_)
    {
        graph_vis_pos_.view();
        graph_vis_vel_.view();
        graph_vis_acc_.view();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
