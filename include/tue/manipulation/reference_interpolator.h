#ifndef _REFERENCE_INTERPOLATOR_H_
#define _REFERENCE_INTERPOLATOR_H_

namespace tue
{
namespace manipulation
{

enum RefState
{
    ACCELERATE,
    DECELERATE,
    IDLE
};

// ----------------------------------------------------------------------------------------------------

class ReferenceInterpolator
{

public:

    ReferenceInterpolator();

    ~ReferenceInterpolator();

    void setState(double pos, double vel);

    void setGoal(double pos);


    void setMaxVelocity(double max_vel) { max_vel_ = max_vel; }

    void setMaxAcceleration(double max_acc) { max_acc_ = max_acc; }


    void update(double dt);

    void brake(double dt);


    double position() const { return x_; }

    double velocity() const { return vel_; }

    bool done() const { return state_ == IDLE; }

private:

    double x_;
    double vel_;
    double x_goal_;

    RefState state_;

    double max_acc_;
    double max_vel_;

};

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

#endif
