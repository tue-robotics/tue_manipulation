#include "tue/manipulation/reference_interpolator.h"

#include <algorithm>
#include <cmath>

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

ReferenceInterpolator::ReferenceInterpolator()
{
}

// ----------------------------------------------------------------------------------------------------

ReferenceInterpolator::~ReferenceInterpolator()
{
}

// ----------------------------------------------------------------------------------------------------

void ReferenceInterpolator::reset(double pos, double vel)
{
    reset_ = true;
    dir_ = 1;
    vel_ = vel;
    x_ = pos;
    ready_ = false;
}

// ----------------------------------------------------------------------------------------------------

ReferencePoint ReferenceInterpolator::generateReference(double x_desired, double max_vel, double max_acc,
                                                   double dt, bool stopping, double eps_tune)
{
    eps_tune = std::max<double>(std::min<double>(eps_tune,2.0),1.0);
    double EPS = 0.5 * max_acc*dt;

    //initial state
    bool still = false;
    bool move = false;
    bool dec = false;
    bool con = false;
    bool acc = false;

    ///double v;
    double a = 0.0;
    double vel_mag = fabs(vel_);

    //compute deceleration distance
    double delta_t1=vel_mag/max_acc; //deceleration segment time
    double dec_dist = 0.5*max_acc * (delta_t1) * (delta_t1); //deceleration distance

    //determine magnitude and sign of error vector
    double delta_x = fabs(x_desired - x_);
    int sign_x = signum(vel_);

    //decide whether to move or stand still
    if (vel_mag!=0.0 || stopping){
        move = true;
        ///ROS_WARN("case 1");
    }
    else if (delta_x > EPS || stopping){
        move = true;
        ///ROS_WARN("case 2");
    }
    else {
        still = true;
        x_ = x_desired;
        ///ROS_WARN("case 3");
    }


    if (reset_)
    {
        dir_ = signum(x_desired - x_);
        reset_ = false;
    }

    ///ROS_INFO("dec_dist=%f, delta_x=%f,dir=%d, sign=%d",fabs(dec_dist),fabs(delta_x),dir,sign_x);

    //move: decide whether to stop, decelerate, constant speed or accelerate
    if (move){

        if (stopping){
            acc = false;
            con = false;
            still = false;
            dec = true;
            ///ROS_ERROR("stopping");
        }
        else if (fabs(dec_dist) >= fabs(delta_x)){
            dec = true;
            ///ROS_INFO("go to dec");
        }
        else if (sign_x * (x_desired - x_) < 0 && vel_mag != 0.0){
            dec = true;
            ///ROS_INFO("setpoint behind");
        }
        else if (fabs(dec_dist) < fabs(delta_x) && vel_mag >= max_vel){
            con = true;
            ///ROS_INFO("go to con");
        }
        else{
            acc = true;
            ///ROS_INFO("go to acc");
        }


        //move: reference value computations
        if (acc){
            vel_mag += max_acc * dt;
            vel_mag = std::min<double>(vel_mag, max_vel);
            x_+= dir_ * vel_mag * dt;
            a = dir_ * max_acc;
        }
        if (con){
            x_+= dir_ * vel_mag * dt;
            a = 0;
        }
        if (dec){
            vel_mag -= max_acc * dt;
            vel_mag = std::max<double>(vel_mag, 0.0);
            x_+= dir_ * vel_mag * dt;
            a = - dir_ * max_acc;
            if (vel_mag < (0.5 * max_acc * dt)){
                vel_mag = 0.0;
                reset_ = true;
                ///ROS_WARN("reset");
            }

        }

        ready_ = false;

    }

    //stand still: reset values
    else if (still){
        vel_ = 0;
        a = 0;
        sign_x = 0;
        reset_ = true;
        ready_ = true;
        /// ROS_INFO("still");

    }
    else {
        ///ROS_ERROR("uncovered!!");
    }
    //populate return values
    ///v = dir * vel_mag;
    vel_ = dir_ * vel_mag;
    ///a = (vel - vel_last)/dt;

    vel_last_ = vel_;

    //return values
    return ReferencePoint(x_, vel_, a);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

