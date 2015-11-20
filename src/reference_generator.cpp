#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

int signum(double a)
{
    if (a < 0)
        return -1;
    else
        return 1;
}

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
void interpolateCubic(double x0, double v0, double x1, double v1, double t, double T, double& x, double &v)
{
    // Transform time to [0, 1]
    double f = t / T;

    // Pre-calculate some things
    double f2 = f * f;
    double f3 = f * f2;

    x = (2 * f3 - 3 * f2 + 1) * x0
            + (f3 - 2 * f2 + f) * (v0 * T)
            + (-2 * f3 + 3 * f2) * x1
            + (f3 - f2) * (v1 * T);

    v = (6 * f2 - 6 * f) * x0 / T
            + (3 * f2 - 4 * f + 1) * v0
            + (-6 * f2 + 6 * f) * x1 / T
            + (3 * f2 - 2 * f) * v1;

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

bool ReferenceGenerator::setJointState(const std::string& joint_name, double pos, double vel)
{
    int idx = this->joint_index(joint_name);
    if (idx < 0)
        return false;

    JointInfo& j = joint_info_[idx];
    j.pos = pos;
    j.vel = vel;
    j.is_idle = true;
    j.is_initialized = true;

    return true;
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

bool ReferenceGenerator::setGoal(const control_msgs::FollowJointTrajectoryGoal& msg, std::stringstream& ss)
{
//    JointGoal goal;
    goal.msg = msg;
    goal.joint_index_mapping.resize(msg.trajectory.joint_names.size());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check feasibility of joint goals

    bool goal_ok = true;
    for (unsigned int i = 0; i < goal.num_goal_joints(); ++i)
    {
        const std::string& joint_name = msg.trajectory.joint_names[i];

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

        goal.joint_index_mapping[i] = idx;
    }

    if (!goal_ok)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Set goal

    if (goal.msg.trajectory.points.empty())
    {
        for(unsigned int i = 0; i < goal.num_goal_joints(); ++i)
            joint_info_[goal.joint_index_mapping[i]].is_idle = true;
        return true;
    }

    goal.sub_goal_idx = -1;
    goal.t = 0;
    goal.t_end = 0;

    for(unsigned int i = 0; i < goal.num_goal_joints(); ++i)
        joint_info_[goal.joint_index_mapping[i]].is_idle = false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void generateSegment(double x0, double x1, const JointInfo& j, TrajectorySegment& seg)
{
    double v0 = seg.v0;
    double v1 = seg.v1;

    std::cout << "Generate: x0 = " << x0 << ", x1 = " << x1 << ", v0 = " << v0 << ", v1 = " << v1 << std::endl;

    bool swapped = false;
    if (v0 > v1)
    {
        double temp = v0;
        v0 = v1;
        v1 = temp;
        swapped = true;
    }

    std::cout << "          v0 = " << v0 << ", v1 = " << v1 << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - -

    double k = (v1 - v0) / j.max_acc;
    double l = seg.t_c - k;

    double X = x1 - x0;
    double U = l * v1 + k * (v0 + v1) / 2;
    double L = l * v0 + k * (v0 + v1) / 2;


    std::cout << "l = " << l << ", X = " << X << ", L = " << L << ", U = " << U << std::endl;

    if (X > U)
    {
        double Y = X - U;
        seg.vc = v1 + 0.5 * j.max_acc * (l - sqrt(std::max<double>(0, l * l - (4 * Y / j.max_acc))));
    }
    else
    {
        if (X > L)
        {
            seg.vc = v0 + (X - L) / l;
        }
        else
        {
            double Y = L - X;
            seg.vc = v0 - 0.5 * j.max_acc * (l - sqrt(std::max<double>(0, l * l - (4 * Y / j.max_acc))));
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - -

    seg.t_a = std::abs(seg.vc - v0) / j.max_acc;
    seg.t_b = seg.t_c - (std::abs(v1 - seg.vc) / j.max_acc);    

    if (swapped)
    {
        double temp = seg.t_a;
        seg.t_a = seg.t_c - seg.t_b;
        seg.t_b = seg.t_c - temp;
    }

    std::cout << "(0, " << seg.v0 << ")    (" << seg.t_a << ", " << seg.vc << ")    (" << seg.t_b << ", " << seg.vc << ")    (" << seg.t_c << ", " << seg.v1 << ")" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::prepareSubGoalTrajectory(JointGoal& goal)
{
    trajectory_msgs::JointTrajectoryPoint& sub_goal = goal.msg.trajectory.points[goal.sub_goal_idx];

    unsigned int num_goal_joints = goal.num_goal_joints();

    if (goal.segments.empty())
        goal.segments.resize(goal.num_goal_joints());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set initial velocities

    for(unsigned int i = 0; i < num_goal_joints; ++i)
        goal.segments[i].v0 = joint_info_[goal.joint_index_mapping[i]].vel;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine sub goal velocities, if not given

    if (sub_goal.velocities.size() == num_goal_joints)
    {
        for(unsigned int i = 0; i < num_goal_joints; ++i)
            goal.segments[i].v1 = sub_goal.velocities[i];
    }
    else
    {
        if (goal.sub_goal_idx + 1 < goal.msg.trajectory.points.size())
        {
            for(unsigned int i = 0; i < num_goal_joints; ++i)
            {
                const JointInfo& j = joint_info_[goal.joint_index_mapping[i]];

                double x0 = j.pos;
                double x1 = sub_goal.positions[i];
                double x2 = goal.msg.trajectory.points[goal.sub_goal_idx + 1].positions[i];

                if ((x0 < x1) == (x1 < x2))
                {
                    // Calculate the maximum velocity we can reach from x0 to x1
                    double v_max_01 = sqrt(2 * j.max_acc * std::abs(x1 - x0) + j.vel * j.vel);

                    // Calculate the maximum velocity we are allowed to have such that we can still reach x2 with 0 velocity
                    double v_max_12 = sqrt(2 * j.max_acc * std::abs(x2 - x1));

                    goal.segments[i].v1 = std::min(j.max_vel, std::min(v_max_01, v_max_12));

                    if (x2 < x1)
                        goal.segments[i].v1 = -goal.segments[i].v1;
                }
                else
                {
                    goal.segments[i].v1 = 0;
                }
            }
        }
        else
        {
            // All velocities to 0
            for(unsigned int i = 0; i < num_goal_joints; ++i)
                goal.segments[i].v1 = 0;
        }

        sub_goal.time_from_start = ros::Duration(0);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine time until sub goal, if not given

    if (sub_goal.time_from_start > ros::Duration(0) && goal.sub_goal_idx > 0)
    {
        goal.t_end = (sub_goal.time_from_start - goal.msg.trajectory.points[goal.sub_goal_idx - 1].time_from_start).toSec();
        for(unsigned int i = 0; i < num_goal_joints; ++i)
            goal.segments[i].t_c = goal.t_end;
    }
    else
    {
        goal.t_end = 0;

        for(unsigned int i = 0; i < num_goal_joints; ++i)
        {
            const JointInfo& j = joint_info_[goal.joint_index_mapping[i]];

            double v0 = goal.segments[i].v0;
            double v1 = goal.segments[i].v1;

            double x_diff = std::abs(sub_goal.positions[i] - j.pos);

            double v_middle = sqrt(j.max_acc * x_diff + (v0 * v0 + v1 * v1) / 2);

            double t_diff;
            if (v_middle > j.max_vel)
            {
                double x_acc = (j.max_vel * j.max_vel - (v0 * v0)) / (2 * j.max_acc);
                double x_dec = (j.max_vel * j.max_vel - (v1 * v1)) / (2 * j.max_acc);

                double x_rest = x_diff - x_acc - x_dec;

                double t_acc = std::abs(j.max_vel - v0) / j.max_acc;
                double t_dec = std::abs(j.max_vel - v1) / j.max_acc;
                double t_rest = x_rest / j.max_vel;

                t_diff = t_acc + t_rest + t_dec;
            }
            else
            {
                double t_acc = std::abs(v_middle - v0) / j.max_acc;
                double t_dec = std::abs(v_middle - v1) / j.max_acc;

                t_diff = t_acc + t_dec;
            }

            goal.t_end = std::max(t_diff, goal.t_end);
        }

        for(unsigned int i = 0; i < num_goal_joints; ++i)
            goal.segments[i].t_c = goal.t_end;
    }

    for(unsigned int i = 0; i < num_goal_joints; ++i)
    {
        const JointInfo& j = joint_info_[goal.joint_index_mapping[i]];
        std::cout << "  " << i << ": " << "x: " << j.pos << " -> " << sub_goal.positions[i] << " | "
                                       << "v: " << j.vel << " -> " << goal.segments[i].v1 << " | "
                                       << "t: " << goal.t << " / " << goal.t_end << std::endl;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Generate segments based on given velocities and times

    for(unsigned int i = 0; i < num_goal_joints; ++i)
    {
        const JointInfo& j = joint_info_[goal.joint_index_mapping[i]];

        double x0 = j.pos;
        double x1 = sub_goal.positions[i];

        generateSegment(x0, x1, j, goal.segments[i]);
    }

//    std::cout << "Velocities: " << sub_goal.velocities << ", duration: " << time_until_next_sub_goal_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::calculatePositionReferences(double dt, std::vector<double>& references)
{   
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Go to next unreached sub goal

//    std::cout << goal.t << " / " << goal.t_end << std::endl;

    if (goal.sub_goal_idx < 0 || goal.t > goal.t_end)
    {
        ++goal.sub_goal_idx;

        if (goal.sub_goal_idx >= goal.msg.trajectory.points.size())
        {
            // Reached final goal

            for(unsigned int i = 0; i < goal.num_goal_joints(); ++i)
                joint_info_[goal.joint_index_mapping[i]].is_idle = true;

            return true;
        }

        std::cout << "---- " << goal.sub_goal_idx << " ----------------------------------------------" << std::endl;

        goal.t -= goal.t_end;
        prepareSubGoalTrajectory(goal);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    for(unsigned int i = 0; i < goal.num_goal_joints(); ++i)
    {
        unsigned int joint_idx = goal.joint_index_mapping[i];

        JointInfo& j = joint_info_[joint_idx];
        j.vel = goal.segments[i].calculateVelocity(goal.t);
        j.pos += dt * j.vel;
        references[joint_idx] = j.pos;

//        const TrajectorySegment& seg = goal.segments[i];
//        std::cout << "(0, " << seg.v0 << ")    (" << seg.t_a << ", " << seg.vc << ")    (" << seg.t_b << ", " << seg.vc << ")    (" << seg.t_c << ", " << seg.v1 << ")" << std::endl;
//        std::cout << goal.t << ": " << j.pos << "    " << j.vel << std::endl;
    }

    goal.t += dt;


    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
