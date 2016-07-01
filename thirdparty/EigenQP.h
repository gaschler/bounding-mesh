/*
 
 The quadprog_solve() function implements the algorithm of Goldfarb and Idnani 
 for the solution of a (convex) Quadratic Programming problem
 by means of an active-set dual method.
	 
The problem is in the form:

min 0.5 * x G x + g0 x
s.t.
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0
	 
 The matrix and vectors dimensions are as follows:
     G: n * n
		g0: n
				
		CE: n * p
	 ce0: p
				
	  CI: n * m
   ci0: m

     x: n
 
 The function will return the cost of the solution written in the x vector or
 std::numeric_limits::infinity() if the problem is infeasible. In the latter case
 the value of the x vector is not correct.
 
 References: D. Goldfarb, A. Idnani. A numerically stable dual method for solving
             strictly convex quadratic programs. Mathematical Programming 27 (1983) pp. 1-33.

 Notes:
  1. pay attention in setting up the vectors ce0 and ci0. 
	   If the constraints of your problem are specified in the form 
	   A^T x = b and C^T x >= d, then you should set ce0 = -b and ci0 = -d.  
  2. The matrix G is modified within the function since it is used to compute
     the G = L^T L cholesky factorization for further computations inside the function. 
     If you need the original matrix G you should make a copy of it and pass the copy
     to the function.
    
 Author: Luca Di Gaspero
  			 DIEGM - University of Udine, Italy
				 l.digaspero@uniud.it
				 http://www.diegm.uniud.it/digaspero/
 
 The author will be grateful if the researchers using this software will
 acknowledge the contribution of this function in their research papers.

LICENSE

This file is part of QuadProg++: a C++ library implementing
the algorithm of Goldfarb and Idnani for the solution of a (convex) 
Quadratic Programming problem by means of an active-set dual method.
Copyright (C) 2007-2009 Luca Di Gaspero.
Copyright (C) 2009 Eric Moyer.  

QuadProg++ is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

QuadProg++ is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with QuadProg++. If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef _EIGENQP
#define _EIGENQP

					   //#include <boost/numeric/ublas/vector.hpp>
					   //#include <boost/numeric/ublas/matrix.hpp>
#include <Eigen/Eigen>
namespace QP {

  //namespace ublas = boost::numeric::ublas;
  using namespace Eigen;
  double solve_quadprog(MatrixXd& G, VectorXd& g0, 
			const MatrixXd& CE, const VectorXd& ce0,  
			const MatrixXd& CI, const VectorXd& ci0, 
			VectorXd& x);
}

#endif // #define _UQUADPROGPP
