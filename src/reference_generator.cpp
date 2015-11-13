#include "tue/manipulation/reference_generator.h"

namespace tue
{
namespace manipulation
{

double ReferenceGenerator::NO_VALUE = -1000; // TODO: make this nicer

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

    std::cout << "(" << x0 << ", " << v0<< ") " << "(" << x << ", " << v << ") " << "(" << x1 << ", " << v1 << ") " << std::endl;
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
    sub_goal_idx_ = -1;

//    graph_viewer_.clear();

    if (goal.trajectory.points.empty())
    {
        is_idle_ = true;
        return true;
    }

    is_idle_ = false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceGenerator::calculateTimeAndVelocities()
{
    trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine sub goal velocities, if not given

    if (sub_goal.velocities.size() != num_goal_joints_)
    {
        sub_goal.velocities.resize(num_goal_joints_, 0);

        if (sub_goal_idx_ + 1 < goal_.trajectory.points.size())
        {
            time_until_next_sub_goal_ = 0;

            for(unsigned int i = 0; i < num_goal_joints_; ++i)
            {
                unsigned int joint_idx = joint_index_mapping_[i];

                double x0 = positions_[joint_idx];
                double x1 = sub_goal.positions[i];
                double x2 = goal_.trajectory.points[sub_goal_idx_ + 1].positions[i];

                if ((x0 < x1) == (x1 < x2))
                {
                    sub_goal.velocities[i] = std::min(max_velocities_[joint_idx], sqrt(2 * max_accelerations_[joint_idx] * std::abs(x2 - x1)));
                    if (x2 < x1)
                        sub_goal.velocities[i] = -sub_goal.velocities[i];
                }
            }
        }

        sub_goal.time_from_start = ros::Duration(0);
    }

    std::cout << "Velocities: " << sub_goal.velocities << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine time until sub goal, if not given

    if (sub_goal.time_from_start > ros::Duration(0) && sub_goal_idx_ > 0)
    {
        time_until_next_sub_goal_ = (sub_goal.time_from_start - goal_.trajectory.points[sub_goal_idx_ - 1].time_from_start).toSec();
    }
    else
    {
        time_until_next_sub_goal_ = 0;

        for(unsigned int i = 0; i < num_goal_joints_; ++i)
        {
            unsigned int joint_idx = joint_index_mapping_[i];

            double v_max = max_velocities_[joint_idx];
            double a_max = max_accelerations_[joint_idx];

            double v0 = std::abs(velocities_[joint_idx]);
            double v1 = std::abs(sub_goal.velocities[i]);

            double x_diff = std::abs(sub_goal.positions[i] - positions_[joint_idx]);

            double v_middle = sqrt(max_accelerations_[joint_idx] * x_diff + (v0 * v0 + v1 * v1) / 2);

            double t_diff;
            if (v_middle > max_velocities_[joint_idx])
            {
                double x_acc = (v_max * v_max - (v0 * v0)) / (2 * a_max);
                double x_dec = (v_max * v_max - (v1 * v1)) / (2 * a_max);

                double x_rest = x_diff - x_acc - x_dec;

                double t_acc = std::abs(v_max - v0) / a_max;
                double t_dec = std::abs(v_max - v1) / a_max;
                double t_rest = x_rest / v_max;

                t_diff = t_acc + t_rest + t_dec;
            }
            else
            {
                double t_acc = std::abs(v_middle - v0) / max_accelerations_[joint_idx];
                double t_dec = std::abs(v_middle - v1) / max_accelerations_[joint_idx];

                t_diff = t_acc + t_dec;
            }

            time_until_next_sub_goal_ = std::max(t_diff, time_until_next_sub_goal_);
        }
    }

    t_segment_ = time_until_next_sub_goal_;
    last_pos_ = positions_;
    last_vel_ = velocities_;

    std::cout << time_until_next_sub_goal_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceGenerator::calculatePositionReferences(const std::vector<double>& positions, double dt,
                                                     std::vector<double>& references)
{   
//    std::cout << is_idle_ << " " << time_until_next_sub_goal_ << " (" << sub_goal_idx_ << "): " << positions << std::endl;

    time_until_next_sub_goal_ -= dt;

    if (positions_.empty())
    {
        positions_ = positions;
        velocities_.resize(positions.size(), 0); // Assume we are initially in 0 velocity position
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -

    if (is_idle_ || sub_goal_idx_ >= (int)goal_.trajectory.points.size())
    {
        references = positions;
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -- - - - - - -
    // Go to next unreached sub goal

    if (sub_goal_idx_ < 0 || time_until_next_sub_goal_ <= 0)
    {
        ++sub_goal_idx_;

        if (sub_goal_idx_ >= goal_.trajectory.points.size())
        {
            // Reached final goal
            references = positions;
            is_idle_ = true;
            return true;
        }

        calculateTimeAndVelocities();
    }

    // TODO: extra check to see if the goal position corresonds to the actual position

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine references using interpolation

    references = positions;

    const trajectory_msgs::JointTrajectoryPoint& sub_goal = goal_.trajectory.points[sub_goal_idx_];

    if (false)
    {

        for(unsigned int i = 0; i < num_goal_joints_; ++i)
        {
            unsigned int joint_idx = joint_index_mapping_[i];

            double& x = positions_[joint_idx];
            double x_goal = sub_goal.positions[i];

            double& v = velocities_[joint_idx];
            double v_abs = std::abs(v);
            double v_goal = sub_goal.velocities[i];

            double v_diff_abs = std::abs(v_goal - v);

            double t_brake = v_diff_abs / max_accelerations_[joint_idx];

            if (i == 0)
                std::cout << "t_brake = " << t_brake << " " << time_until_next_sub_goal_ << std::endl;

            int v_sign = signum(x_goal - x);

            if (time_until_next_sub_goal_ <= t_brake)
            {
                v -= dt * max_accelerations_[joint_idx] * v_sign;
            }
            else
            {
                double s;
                if (v_abs < std::abs(v_goal))
                    s = v_abs * time_until_next_sub_goal_ + (v_diff_abs  * t_brake) / 2;
                else
                    s = v_abs * time_until_next_sub_goal_ - (v_diff_abs  * t_brake) / 2;

                if (i == 0)
                    std::cout << i << ": x = " << x << ", x_goal = " << x_goal << ", v = " << v << ", v_goal = " << v_goal << ", t_left = " << time_until_next_sub_goal_
                              << ", s = " << s << "v_sign = " << v_sign << "x_diff = " << std::abs(x_goal - x) << std::endl;



                if (s < std::abs(x_goal - x))
                    v += dt * max_accelerations_[joint_idx] * v_sign;
                else
                    v -= dt * max_accelerations_[joint_idx] * v_sign;
            }

            if (i == 0)
                std::cout << "    v_new = " << v << std::endl;

            x += dt * v;

            references[joint_idx] = x;
        }
    }
    else
    {
        for(unsigned int i = 0; i < num_goal_joints_; ++i)
        {
            unsigned int joint_idx = joint_index_mapping_[i];

            std::cout << i << ": ";

            interpolateCubic(last_pos_[joint_idx], last_vel_[joint_idx], sub_goal.positions[i], sub_goal.velocities[i],
                             t_segment_ - time_until_next_sub_goal_, t_segment_, positions_[joint_idx], velocities_[joint_idx]);

            references[i] = positions_[joint_idx];
        }
    }


//    if (is_smooth_sub_goal_)
//    {
////        std::cout << "CUBIC!" << std::endl;

//        // Use cubic interpolation

//        const trajectory_msgs::JointTrajectoryPoint& prev_sub_goal = goal_.trajectory.points[sub_goal_idx_ - 1];

//        trajectory_msgs::JointTrajectoryPoint p_interpolated;
//        interpolateCubic(p_interpolated, prev_sub_goal, sub_goal, time_since_start_);

//        for(unsigned int i = 0; i < num_goal_joints_; ++i)
//        {
//            unsigned int joint_idx = joint_index_mapping_[i];
//            references[joint_idx] = p_interpolated.positions[i];
//            interpolators_[joint_idx].reset(p_interpolated.positions[i],
//                                            p_interpolated.velocities[i]);
//        }

////        graph_viewer_.addPoint(0, 0, time_since_start_, p_interpolated.positions[0], p_interpolated.velocities[0]);
//    }
//    else
//    {
////        graph_viewer_.clear();

//        for(unsigned int i = 0; i < num_goal_joints_; ++i)
//        {
//            double p_wanted = sub_goal.positions[i];

//            unsigned int joint_idx = joint_index_mapping_[i];

//            ReferenceInterpolator& r = interpolators_[joint_idx];
//            ReferencePoint ref = r.generateReference(p_wanted, max_velocities_[joint_idx],
//                                                     max_accelerations_[joint_idx], dt, false, 0.01);
//            references[joint_idx] = ref.pos;
//        }
//    }

//    graph_viewer_.view();

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
