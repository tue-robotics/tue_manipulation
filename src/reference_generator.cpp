#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

ReferenceGenerator::ReferenceGenerator() : next_goal_id_(0)
{
    visualize_ = true;
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

        if (js.interpolator.max_velocity() == 0 || js.interpolator.max_acceleration() == 0 || (js.min_pos == js.max_pos))
        {
            ss << "Joint '" << joint_name << "' limits not initialized: "
               << "max vel = " << js.interpolator.max_velocity() << ", max acc = " << js.interpolator.max_acceleration()
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

    graph_viewer_.clear();
    time_ = 0;

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

void ReferenceGenerator::calculatePositionReferences(JointGoal& goal, double dt, std::vector<double>& references)
{
    goal.time_since_start += dt;
    time_ += dt;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Determine if we reached the next sub goal

    bool sub_goal_reached = false;

    if (goal.sub_goal_idx < 0)
    {
        sub_goal_reached = true;
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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // If so, prepare for the next sub goal

    if (sub_goal_reached)
    {
        // If it has been reached, go to the next one
        ++goal.sub_goal_idx;

        if (goal.sub_goal_idx >= goal.goal_msg.trajectory.points.size())
        {
            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
                joint_info_[goal.joint_index_mapping[i]].goal_id.clear();

            goal.status = JOINT_GOAL_SUCCEEDED;
            return;
        }

        const trajectory_msgs::JointTrajectoryPoint& p_prev = goal.goal_msg.trajectory.points[goal.sub_goal_idx - 1];
        const trajectory_msgs::JointTrajectoryPoint& p_next = goal.goal_msg.trajectory.points[goal.sub_goal_idx];

        bool vels_defined = (p_next.velocities.size() == goal.num_goal_joints);

        double time = p_next.time_from_start.toSec() - p_prev.time_from_start.toSec();

        if (time <= 0)
        {
            for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
            {
                JointInfo& js = joint_info_[goal.joint_index_mapping[i]];
                ReferenceInterpolator& r = js.interpolator;

                double x_goal = p_next.positions[i];
                double v_goal = vels_defined ? p_next.velocities[i] : 0;

                time = std::max(time, r.calculateTime(x_goal, v_goal));
            }
        }

        for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
        {
            JointInfo& js = joint_info_[goal.joint_index_mapping[i]];
            ReferenceInterpolator& r = js.interpolator;

            double x_goal = p_next.positions[i];
            double v_goal = vels_defined ? p_next.velocities[i] : 0;

            r.setGoal(x_goal, v_goal, time);
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    for(unsigned int i = 0; i < goal.num_goal_joints; ++i)
    {
        unsigned int joint_idx = goal.joint_index_mapping[i];
        JointInfo& j = joint_info_[joint_idx];

        j.interpolator.update(dt);
        references[joint_idx] = j.interpolator.position();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Visualize

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

        if (goal.status != JOINT_GOAL_ACTIVE)
            continue;

        calculatePositionReferences(goal, dt, references);
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
