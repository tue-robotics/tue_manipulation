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

#include "tue/manipulation/constrained_chainiksolvervel_pinv.h"

namespace KDL
{
    ConstrainedChainIkSolverVel_pinv::ConstrainedChainIkSolverVel_pinv(const Chain& _chain, double _eps, int _maxiter, uint _n_constraints):
        chain(_chain),
        jnt2jac(chain),
        nj(chain.getNrOfJoints()),
        n_constraints(_n_constraints),
        qdot_out_reduced(nj - n_constraints),
        jac(nj),
        jac_reduced(nj - n_constraints),
        svd(jac),
        U(6,JntArray(nj - n_constraints)),
        S(nj - n_constraints),
        V(nj - n_constraints, JntArray(nj - n_constraints)),
        tmp(nj - n_constraints),
        eps(_eps),
        maxiter(_maxiter)
    {
    }

    void ConstrainedChainIkSolverVel_pinv::updateInternalDataStructures()
    {
        jnt2jac.updateInternalDataStructures();
        nj = chain.getNrOfJoints();
        jac.resize(nj);
        jac_reduced.resize(nj - n_constraints);
        svd = SVD_HH(jac);
        for(unsigned int i = 0 ; i < U.size(); i++)
            U[i].resize(nj);
        S.resize(nj - n_constraints);
        V.resize(nj - n_constraints);
        for(unsigned int i = 0 ; i < V.size(); i++)
            V[i].resize(nj - n_constraints);
        tmp.resize(nj - n_constraints);
    }

    ConstrainedChainIkSolverVel_pinv::~ConstrainedChainIkSolverVel_pinv()
    {
    }


    int ConstrainedChainIkSolverVel_pinv::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
    {
        //Let the ChainJntToJacSolver calculate the jacobian "jac" for
        //the current joint positions "q_in" 
        jnt2jac.JntToJac(q_in, jac);

        // Apply constraints (would JntToJac be a better place to do this immediately?)
        // ToDo: don't hardcode
        // ToDo: use polynomials
        // Compare this with the function under 'apply constraints' in the iksolverpos
        double dcdq = 3*1.3365*q_in(0)*q_in(0) - 2*1.9862*q_in(0) + 2.6392;
        // Loop over columns
        for (unsigned int col=0; col<jac_reduced.columns(); col++) {
            for (unsigned int row=0; row<jac_reduced.rows(); row++) {
                // ToDo: don't hardcode
                if (col == 0) {
                    jac_reduced(row, col) = jac(row, col) + dcdq*jac(row, col+1);
                } else {
                    jac_reduced(row, col) = jac(row, col+1);
                }
            }
        }

        //Do a singular value decomposition of "jac" with maximum
        //iterations "maxiter", put the results in "U", "S" and "V"
        //jac = U*S*Vt
        int ret = svd.calculate(jac_reduced, U, S, V, maxiter);

        double sum;
        unsigned int i, j;

        // We have to calculate qdot_out = jac_pinv*v_in
        // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
        // qdot_out = V*S_pinv*Ut*v_in
        //first we calculate Ut*v_in
        for (i=0; i<jac_reduced.columns(); i++) {
            sum = 0.0;
            for (j=0; j<jac_reduced.rows(); j++) {
                sum+= U[j](i)*v_in(j);
            }
            //If the singular value is too small (<eps), don't invert it but
            //set the inverted singular value to zero (truncated svd)
            tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
        }
        //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
        //it with V to get qdot_out
        for (i=0; i<jac_reduced.columns(); i++) {
            sum = 0.0;
            for (j=0; j<jac_reduced.columns(); j++) {
                sum+=V[i](j)*tmp(j);
            }
            //Put the result in qdot_out
            qdot_out_reduced(i)=sum;
        }
        //return the return value of the svd decomposition
        // Expand solution
        qdot_out.data.setZero();
        for (unsigned int k = 0; k < qdot_out_reduced.rows(); k++) {
            // ToDo: don't hardcode bookkeeping
            if (k == 0) {
                qdot_out(k) = qdot_out_reduced(k);
            } else {
                qdot_out(k+1) = qdot_out_reduced(k);
            }
        }
        return ret;
    }

}
