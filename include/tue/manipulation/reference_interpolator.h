#ifndef _REFERENCE_INTERPOLATOR_H_
#define _REFERENCE_INTERPOLATOR_H_

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

struct ReferencePoint
{
    ReferencePoint() {}

    ReferencePoint(double pos_, double vel_, double acc_)
        : pos(pos_), vel(vel_), acc(acc_) {}

    double pos;
    double vel;
    double acc;
};

// ----------------------------------------------------------------------------------------------------

class ReferenceInterpolator
{

public:

    ReferenceInterpolator();

    ~ReferenceInterpolator();

    void reset(double pos, double vel);

    ReferencePoint generateReference(double x_desired, double max_vel, double max_acc, double dt, bool stop, double EPS);

private:

    int dir_;
    bool reset_;
    double x_;
    double vel_;
    double vel_last_;
    bool ready_;

};

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

#endif
