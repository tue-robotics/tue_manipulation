#include "tue/manipulation/ik_solver.h"

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace tue
{

// ----------------------------------------------------------------------------------------------------

IKSolver::IKSolver()
{
}

// ----------------------------------------------------------------------------------------------------

IKSolver::~IKSolver()
{
}

// ----------------------------------------------------------------------------------------------------

bool IKSolver::initFromURDF(const std::string& urdf, const std::string root_name,
                            const std::string& tip_name, std::string& error)
{
    urdf::Model robot_model;
    KDL::Tree tree;

    if (!robot_model.initString(urdf))
    {
        error += "Could not initialize robot model";
        return false;
    }

    if (!kdl_parser::treeFromString(urdf, tree))
    {
        error += "Could not initialize tree object";
        return false;
    }

    if (tree.getSegment(root_name) == tree.getSegments().end())
    {
        error += "Could not find root link '" + root_name + "'.";
        return false;
    }

    if (tree.getSegment(tip_name) == tree.getSegments().end())
    {
        error += "Could not find tip link '" + tip_name + "'.";
        return false;
    }

    if (!tree.getChain(root_name, tip_name, chain_))
    {
        error += "Could not initialize chain object";
        return false;
    }

    // Get the joint limits from the robot model

    q_min_.resize(chain_.getNrOfJoints());
    q_max_.resize(chain_.getNrOfJoints());

    q_seed_.resize(chain_.getNrOfJoints());

    for(unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(chain_.getSegment(i).getJoint().getName());
        if (joint && joint->limits)
        {
            q_min_(i) = joint->limits->lower;
            q_max_(i) = joint->limits->upper;
            q_seed_(i) = (q_min_(i) + q_max_(i)) / 2;

        }
        else
        {
            q_min_(i) = -1e9;
            q_max_(i) = 1e9;
            q_seed_(i) = 0;

        }
//        std::cout << q_min_(i) << " - " << q_max_(i) << std::endl;
    }


    // Construct the IK solver
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
    ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fksolver_, *ik_vel_solver_));

    return true;
}

// ----------------------------------------------------------------------------------------------------

int IKSolver::jointsToCartesian(const KDL::JntArray& q_in, KDL::Frame& f_out)
{
    return fksolver_->JntToCart(q_in, f_out);
}

// ----------------------------------------------------------------------------------------------------

int IKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out)
{
    return cartesianToJoints(f_in, q_out, q_seed_);
}

// ----------------------------------------------------------------------------------------------------

int IKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out, const KDL::JntArray& q_seed)
{
    return ik_solver_->CartToJnt(q_seed, f_in, q_out);
}



}
