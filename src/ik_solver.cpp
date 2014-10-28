#include "tue/manipulation/ik_solver.h"

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <tue/manipulation/constrained_chainiksolverpos_nr_jl.hpp>
#include <tue/manipulation/constrained_chainiksolvervel_pinv.h>

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
                            const std::string& tip_name, unsigned int max_iter, std::string& error,
                            bool use_constrained_solver)
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

    joint_names_.resize(chain_.getNrOfJoints());

    unsigned int j = 0;
    for(unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        const KDL::Joint& kdl_joint = chain_.getSegment(i).getJoint();
        if (kdl_joint.getType() != KDL::Joint::None)
        {
//            std::cout << chain_.getSegment(i).getName() << " -> " << kdl_joint.getName() << " -> " << chain_.getSegment(i + 1).getName() << std::endl;

            boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(kdl_joint.getName());
            if (joint && joint->limits)
            {
                q_min_(j) = joint->limits->lower;
                q_max_(j) = joint->limits->upper;
                q_seed_(j) = (q_min_(j) + q_max_(j)) / 2;
            }
            else
            {
                q_min_(j) = -1e9;
                q_max_(j) = 1e9;
                q_seed_(j) = 0;

            }

            joint_names_[j] = kdl_joint.getName();
//            std::cout << j << ": " << q_min_(j) << " - " << q_max_(j) << std::endl;

            ++j;
        }
    }

    // Construct the IK solver
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

    if (!use_constrained_solver) {
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
        ik_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fksolver_, *ik_vel_solver_, max_iter));
        std::cout << "Using normal solver" << std::endl;
    } else {
        ik_vel_solver_.reset(new KDL::ConstrainedChainIkSolverVel_pinv(chain_, 0.00001, 150, 1));
        ik_solver_.reset(new KDL::ConstrainedChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fksolver_, *ik_vel_solver_, max_iter));
        std::cout << "Using constrained IK solver" << std::endl;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool IKSolver::jointsToCartesian(const KDL::JntArray& q_in, KDL::Frame& f_out)
{
    int status = fksolver_->JntToCart(q_in, f_out);
    return (status == 0);
}

// ----------------------------------------------------------------------------------------------------

bool IKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out)
{
    return cartesianToJoints(f_in, q_out, q_seed_);
}

// ----------------------------------------------------------------------------------------------------

bool IKSolver::cartesianToJoints(const KDL::Frame& f_in, KDL::JntArray& q_out, const KDL::JntArray& q_seed)
{
    int status = ik_solver_->CartToJnt(q_seed, f_in, q_out);
    return (status == 0);
}

}
