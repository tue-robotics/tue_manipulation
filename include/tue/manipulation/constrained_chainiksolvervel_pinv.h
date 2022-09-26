// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

// Modified by Janno Lunenburg

#ifndef KDL_CONSTRAINED_CHAIN_IKSOLVERVEL_PINV_HPP
#define KDL_CONSTRAINED_CHAIN_IKSOLVERVEL_PINV_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/utilities/svd_HH.hpp>

namespace KDL
{
    /**
     * Implementation of a inverse velocity kinematics algorithm based
     * on the generalize pseudo inverse to calculate the velocity
     * transformation from Cartesian to joint space of a general
     * KDL::Chain. It uses a svd-calculation based on householders
     * rotations.
     *
     * A constrained is applied which relates the velocity of two joints
     *
     * @ingroup KinematicFamily
     */
    class ConstrainedChainIkSolverVel_pinv : public ChainIkSolverVel
    {
    public:
        /**
         * Constructor of the solver
         *
         * @param chain the chain to calculate the inverse velocity
         * kinematics for
         * @param eps if a singular value is below this value, its
         * inverse is set to zero, default: 0.00001
         * @param maxiter maximum iterations for the svd calculation,
         * default: 150
         * @param _n_constraints number of constraints that is applied
         * default: 0
         * Should be replaced by constraints itself in some format
         *
         */
        explicit ConstrainedChainIkSolverVel_pinv(const Chain& chain, double eps=0.00001, int maxiter=150, uint _n_constraints=0);
        ~ConstrainedChainIkSolverVel_pinv();

        virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);
        /**
         * not (yet) implemented.
         *
         */
        virtual int CartToJnt(const JntArray& /*q_init*/, const FrameVel& /*v_in*/, JntArrayVel& /*q_out*/){return -1;};

        virtual void updateInternalDataStructures();
    private:
        const Chain& chain;
        ChainJntToJacSolver jnt2jac;
        unsigned int nj, n_constraints;
        JntArray qdot_out_reduced;
        Jacobian jac, jac_reduced;
        SVD_HH svd;
        std::vector<JntArray> U;
        JntArray S;
        std::vector<JntArray> V;
        JntArray tmp;
        double eps;
        int maxiter;

    };
}
#endif

