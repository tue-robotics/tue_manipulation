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

}

// ----------------------------------------------------------------------------------------------------

ReferenceInterpolator::ReferenceInterpolator() : state_(IDLE)
{
}

// ----------------------------------------------------------------------------------------------------

ReferenceInterpolator::~ReferenceInterpolator()
{
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::setState(double pos, double vel)
{
    vel_ = vel;
    x_ = pos;

    if (!done())
        setGoal(x_goal_);
}

// ----------------------------------------------------------------------------------------------------


void ReferenceInterpolator::setGoal(double pos)
{
    x_goal_ = pos;
    state_ = ACCELERATE;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::update(double dt)
{
    if (done())
        return;

    double v_step = dt * max_acc_;
    int x_dir = signum(x_goal_ - x_);

    if (state_ == ACCELERATE)
    {
        double x_brake = 0.5 * vel_ * vel_ / max_acc_;
        if (x_brake >= std::abs(x_goal_ - x_) && signum(vel_) == x_dir)
            state_ = DECELERATE;
        else if (state_ == ACCELERATE)
            vel_ += v_step * x_dir;
    }

    if (state_ == DECELERATE)
    {
        if (std::abs(vel_) < v_step || signum(vel_) != x_dir)
        {
            x_ = x_goal_;
            vel_ = 0;
            state_ = IDLE;
            return;
        }
        else
            vel_ -= v_step * x_dir;
    }


    if (vel_ < -max_vel_)
        vel_ = -max_vel_;
    else if (vel_ > max_vel_)
        vel_ = max_vel_;

    x_ += dt * vel_;
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::brake(double dt)
{
    if (vel_ == 0)
        return;

    double vel_abs = fabs(vel_);
    if (vel_abs < dt * max_acc_)
    {
        vel_ = 0;
        return;
    }

    int v_sign = signum(vel_);
    vel_ -= dt * max_acc_ * v_sign;
    x_ += dt * vel_;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

