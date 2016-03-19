#ifndef _REFERENCE_INTERPOLATOR_H_
#define _REFERENCE_INTERPOLATOR_H_

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

class ReferenceInterpolator
{

public:

    ReferenceInterpolator();

    ~ReferenceInterpolator();


    // State and goal specification

    void setState(double pos, double vel, double acc = 0);
    
    void resetState(double pos, double vel, double acc = 0); // Reset state for resetting to initial position upon startup

    bool setGoal(double pos, double vel = 0, double t = -1);

    void brake(double dt);


    // Calculation methods

    double calculateTimeNeeded(double x1, double v1)
    {
        return calculateTimeNeeded(x_, v_, x1, v1);
    }

    double calculateTimeNeeded(double x0, double v0, double x1, double v1);


    // Update

    void update(double dt);


    // Setters limits

    void setMaxVelocity(double max_vel) { max_vel_ = max_vel; }

    void setMaxAcceleration(double max_acc) { max_acc_ = max_acc; }


    // Query methods

    double max_velocity() const { return max_vel_; }

    double max_acceleration() const { return max_acc_; }

    double position() const { return x_; }

    double velocity() const { return v_; }

    double acceleration() const { return a_; }

    double goal_position() const { return x_goal_; }

    double goal_velocity() const { return v_goal_; }

    bool done() const { return t_ > t_goal_; }

private:

    // Current state

    double t_;
    double x_;
    double v_;
    double a_;

    // Trajectory

    double x0_;
    double x1_;
    double x2_;

    double v0_;
    double vc_;

    double t1_;
    double t2_;

    // Goal

    double x_goal_;
    double v_goal_;
    double t_goal_;

    // Limits

    double max_acc_;
    double max_vel_;

};

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

#endif
