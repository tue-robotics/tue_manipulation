// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

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

#include <iostream>

#include "tue/manipulation/constrained_chainiksolverpos_nr_jl.hpp"

namespace KDL
{
    ConstrainedChainIkSolverPos_NR_JL::ConstrainedChainIkSolverPos_NR_JL(const Chain& _chain, const JntArray& _q_min, const JntArray& _q_max, ChainFkSolverPos& _fksolver,ChainIkSolverVel& _iksolver,
                                             unsigned int _maxiter, double _eps):
        chain(_chain), q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints()), fksolver(_fksolver),iksolver(_iksolver),delta_q(_chain.getNrOfJoints()),
        maxiter(_maxiter),eps(_eps)
    {
        q_min = _q_min;
    	q_max = _q_max;
        //for (unsigned int i = 0; i < chain.getNrOfJoints(); i++) {
        //    std::cout << "q_min = " << q_min(i) << ", q_max = " << q_max(i) << std::endl;
        //}
    }

    void ConstrainedChainIkSolverPos_NR_JL::updateInternalDataStructures()
    {
        nj = chain.getNrOfJoints();
        q_min.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::min()));
        q_max.data.conservativeResizeLike(Eigen::VectorXd::Constant(nj, std::numeric_limits<double>::max()));
        iksolver.updateInternalDataStructures();
        fksolver.updateInternalDataStructures();
        delta_q.resize(nj);
    }


    int ConstrainedChainIkSolverPos_NR_JL::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
    {
            q_out = q_init;

            unsigned int i;
            for(i=0; i<maxiter; i++){
                fksolver.JntToCart(q_out, f);
                delta_twist = diff(f, p_in);

				if(Equal(delta_twist,Twist::Zero(),eps))
					break;

                // ToDo: modify iksolver
                iksolver.CartToJnt(q_out, delta_twist, delta_q);
                Add(q_out, delta_q, q_out);

                /// Apply constraints
                // ToDo: don't hardcode
                // ToDo: use polynomials
                q_out(1) = 1.3365*q_out(0)*q_out(0)*q_out(0) - 1.9862*q_out(0)*q_out(0) + 2.6392*q_out(0) + 0.001;

                for(unsigned int j=0; j<q_min.rows(); j++) {
                  if(q_out(j) < q_min(j))
                    q_out(j) = q_min(j);
                }

                for(unsigned int j=0; j<q_max.rows(); j++) {
                    if(q_out(j) > q_max(j))
                      q_out(j) = q_max(j);
                }
            }

            if(i!=maxiter)
                return 0;
            else
                return -3;
    }

    ConstrainedChainIkSolverPos_NR_JL::~ConstrainedChainIkSolverPos_NR_JL()
    {
    }

}

