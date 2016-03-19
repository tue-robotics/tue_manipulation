#include "tue/manipulation/reference_interpolator.h"

#include <algorithm>
#include <cmath>

#include <iostream>

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------


namespace
{

int signum(double a)
{
    if (a < 0)
        return -1;
    if (a >= 0)
        return 1;
    else
        return 1;
}

void swap(double& a, double& b)
{
    double temp = a;
    a = b;
    b = temp;
}

}

// ----------------------------------------------------------------------------------------------------

ReferenceInterpolator::ReferenceInterpolator() : t_(0), t_goal_(-1)
{
}

// ----------------------------------------------------------------------------------------------------

ReferenceInterpolator::~ReferenceInterpolator()
{
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::setState(double pos, double vel, double acc)
{
    v_ = vel;
    x_ = pos;
    a_ = acc;

    if (!done())
        setGoal(x_goal_);
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::resetState(double pos, double vel, double acc)
{
	v_ = vel;
	x_ = pos;
	a_ = acc;
	
	t_goal_ = -1.0;
	t_ = 0.0;
	x_goal_ = x_;
	v_goal_ = v_;
    
}

// ----------------------------------------------------------------------------------------------------

bool ReferenceInterpolator::setGoal(double pos, double vel, double t)
{
    // Check velocity limits
    if (vel < -max_vel_ || vel > max_vel_)
        return false;

    if (t < 0)
    {
        t = calculateTimeNeeded(pos, vel);
        if (t < 0)
            return false;
    }

    double v0 = v_;
    double v1 = vel;

    double x0 = x_;
    double x1 = pos;

    bool mirrored = false;
    if (x1 < x0)
    {
        swap(x0, x1);
        swap(v0, v1);
        v0 = -v0;
        v1 = -v1;
        mirrored = true;
    }

    if (v0 > v1)
        swap(v0, v1);

    // - - - - - - - - - - - - - - - - - - - - - - - - -

    double k = (v1 - v0) / max_acc_;
    double l = t - k;

    if (l < 0)
    {
//        std::cout << "ReferenceInterpolator::setGoal: Cannot do this! (l < 0): l = " << l << std::endl;
        return false;
    }

    double X = x1 - x0;
    double U = l * v1 + k * (v0 + v1) / 2;
    double L = l * v0 + k * (v0 + v1) / 2;

    if (X > U)
    {
        double Y = X - U;
        double r = l * l - (4 * Y / max_acc_);

        if (r < -1e-9)
        {
//            std::cout << "ReferenceInterpolator::setGoal: Cannot do this! (X > U): " << r << std::endl;
            return false;
        }

        r = std::max<double>(0, r);
        vc_ = v1 + 0.5 * max_acc_ * (l - sqrt(r));

    }
    else if (X > L)
    {
        vc_ = v0 + (X - L) / l;
    }
    else
    {
        double Y = L - X;
        double r = l * l - (4 * Y / max_acc_);

        if (r < -1e-9)
        {
//            std::cout << "ReferenceInterpolator::setGoal: Cannot do this! (X < L): " << r << std::endl;
            return false;
        }

        r = std::max<double>(0, r);
        vc_ = v0 - 0.5 * max_acc_ * (l - sqrt(r));
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - -

    if (mirrored)
        vc_ = -vc_;

    t_goal_ = t;
    v0_ = v_;
    v_goal_ = vel;
    x_goal_ = pos;

    t1_ = std::abs(vc_ - v0_) / max_acc_;
    t2_ = t_goal_ - (std::abs(v_goal_ - vc_) / max_acc_);

    x0_ = x_;
    x1_ = x0_ + t1_ * (v0_ + vc_) / 2;
    x2_ = x1_ + (t2_ - t1_) * vc_;

    t_ = 0;

//    std::cout << "(0, " << v_ << "), (" << t1_ << ", " << vc_ << "), (" << t2_ << ", " << vc_ << "), (" << t_goal_ << ", " << vel << ")" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::update(double dt)
{
    t_ += dt;

    if (t_ >= t_goal_)
    {
		//std::cout << "(t_ >= t_goal_)" << std::endl;
        x_ = x_goal_;
        v_ = v_goal_;
        a_ = 0;
        return;
    }

    if (t_ < t1_)
    {
        double f = (t_ / t1_);
        v_ = (1 - f) * v0_ + f * vc_;
        x_ = x0_ + t_ * (v0_ + v_) / 2;
        a_ = max_acc_ * signum(vc_ - v0_);
    }
    else if (t_ <= t2_)
    {
        v_ = vc_;
        x_ = x1_ + (t_ - t1_) * vc_;
        a_ = 0;
    }
    else
    {
        double f = (t_ - t2_) / (t_goal_ - t2_);
        v_ = (1 - f) * vc_ + f * v_goal_;
        x_ = x2_ + (t_ - t2_) * (vc_ + v_) / 2;
        a_ = max_acc_ * signum(v_goal_ - vc_);
    }
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::brake(double dt)
{
    if (v_ == 0)
        return;

    double vel_abs = fabs(v_);
    if (vel_abs < dt * max_acc_)
    {
        v_ = 0;
        return;
    }

    int v_sign = signum(v_);
    v_ -= dt * max_acc_ * v_sign;
    x_ += dt * v_;
}

// ----------------------------------------------------------------------------------------------------

double ReferenceInterpolator::calculateTimeNeeded(double x0, double v0, double x1, double v1)
{
    // Check velocity limits
    if (v0 < -max_vel_ || v0 > max_vel_ || v1 < -max_vel_ || v1 > max_vel_)
        return -1;

    // Calculate distance to travel
    double X = x1 - x0;

    // Precalculate (we need it later)
    double v_diff_sq = v0 * v0 + v1 * v1;

    // Calculate the center velocity 'vc'. This is a peak velocity that makes sure we get
    // to the goal position as soons as possible. It is either larger than both v0 and v1,
    // or smaller, as illustrated in the following velocity profiles:

    //
    //        * vc                                * v1
    //       / \                                 /
    //      /   \                        v0     /
    //     /     \                       *     /
    //    /       * v1          or        \   /
    //   /                                 \ /
    //  * v0                                * vc
    //
    //  ----------------> t            ----------------> t

    // We now if it is higher or lower by comparing the distance we need to travel (X) by the
    // distance we traveled if we would directly change from velocity v0 to v1 (Y).
    // If X is lower than Y, it means we have to decelerate first, then accelerate to v1,
    // and vc is below v0 and v1. Otherwise 'vc' is above v0 and v1.

    // Calculate the distance traveled if we would directly change from velocity v0 to v1
    double Y = (std::abs(v1 - v0) / max_acc_) * (v0 + v1) / 2;

    double vc;
    if (X < Y)
        // vc is below v0 and v1
        vc = -sqrt(v_diff_sq / 2 - max_acc_ * X);
    else
        // vc is above v0 and v1
        vc =  sqrt(v_diff_sq / 2 + max_acc_ * X);

    // If vc is larger than max_vel, it means we would exceed our maximum velocity. Instead
    // we should accelerate to the maximum velocity as fast as possible, then maintain it for
    // some time, and decelerate.
    if (vc > max_vel_)
    {
        double time = (max_vel_ - v0) / max_acc_ + (max_vel_ - v1) / max_acc_;
        double X_covered = (2 * max_vel_ * max_vel_ - v_diff_sq) / (2 * max_acc_);
        double X_rest = X - X_covered;
        double t_rest = X_rest / max_vel_;
        return time + t_rest;
    }
    // A similar thing holds for when vc is negative and smaller than -max_vel
    else if (vc < -max_vel_)
    {
        double time = (v0 + max_vel_) / max_acc_ + (v1 + max_vel_) / max_acc_;
        double X_covered = (v_diff_sq - 2 * max_vel_ * max_vel_) / (2 * max_acc_);
        double X_rest = X_covered - X;
        double t_rest = X_rest / max_vel_;
        return time + t_rest;
    }
    // Otherwise we can just calculate how long it will take to change velocity from v0 to vc,
    // and from vc to v1.
    else
    {
        return std::abs(vc - v0) / max_acc_ + std::abs(vc - v1) / max_acc_;
    }
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation
