#ifndef TUE_MANIPULATION_IK_SOLVER_H_
#define TUE_MANIPULATION_IK_SOLVER_H_

#include <string>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <boost/shared_ptr.hpp>

namespace urdf
{
    class Model;
}

namespace KDL
{
    class ChainFkSolverPos;
    class ChainIkSolverVel;
    class ChainIkSolverPos;
}

namespace tue
{

class IKSolver
{

public:

    IKSolver();

    virtual ~IKSolver();

    bool initFromURDF(const std::string& urdf, const std::string root_name,
                      const std::string& tip_name, unsigned int max_iter, std::string& error,
                      bool use_constrained_solver);

    bool jointsToCartesian(const KDL::JntArray& q_in, KDL::Frame& f_out);

    bool cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out);

    bool cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out, const KDL::JntArray& q_seed);

    inline const KDL::JntArray& jointLowerLimits() const { return q_min_; }

    inline const KDL::JntArray& jointUpperLimits() const { return q_max_; }

    inline const std::vector<std::string>& jointNames() const { return joint_names_; }

    inline unsigned int numJoints() const { return joint_names_.size(); }

private:

    //!Object to describe the (serial) kinematic chain
    KDL::Chain chain_;

    KDL::JntArray q_min_, q_max_, q_seed_;

    std::vector<std::string> joint_names_;

    // Solvers
    boost::shared_ptr<KDL::ChainFkSolverPos> fksolver_;
    boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
    boost::shared_ptr<KDL::ChainIkSolverPos> ik_solver_;

};

}

#endif
