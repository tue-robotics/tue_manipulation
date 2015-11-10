#ifndef _REFERENCE_INTERPOLATOR_H_
#define _REFERENCE_INTERPOLATOR_H_

namespace tue
{
namespace manipulation
{

// ----------------------------------------------------------------------------------------------------

struct PosVelAcc
{
    PosVelAcc() {}

    PosVelAcc(double pos_, double vel_, double acc_) : pos(pos_), vel(vel_), acc(acc_) {}

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

    void setRefGen(double x_reset);

    PosVelAcc generateReference(double x_desired, double max_vel, double max_acc, double dt, bool stop, double EPS);

private:

    int dir;
    bool reset;
    double x;
    double vel;
    double vel_last;
    bool ready;

};

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace manipulation

#endif
