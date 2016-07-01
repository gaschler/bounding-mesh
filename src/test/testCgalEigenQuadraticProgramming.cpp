//	Copyright (c) 2016, Andre Gaschler
//	All rights reserved.

#include <chrono>
#include <iostream>
#include <sstream>
#include <vector>

#include <boundingmesh.h>
using namespace boundingmesh;

#include "../../thirdparty/EigenQP.h"

//NOTICE: CGAL Quadratic Programming has license GPL!
//http://doc.cgal.org/latest/Manual/packages.html

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#if 0
#include <CGAL/MP_Float.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Quotient.h>
typedef CGAL::MP_Float ET;
#endif
typedef double ET;
typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

#if (__GNUC__ > 4 || __GNUC__ == 4 && __GNUC_MINOR__ >= 7)
typedef ::std::chrono::steady_clock MyClock;
#else
typedef ::std::chrono::monotonic_clock MyClock;
#endif

class InternalTimer
{
public:
	InternalTimer()
{
	this->elapsedTime = MyClock::duration::zero();
}

	MyClock::duration getElapsedTime()
	{
		return this->elapsedTime;
	}

	double getElapsedSeconds()
	{
		return (::std::chrono::duration_cast< ::std::chrono::duration< double > >(this->elapsedTime)).count();
	}

	void start()
	{
		this->startTime = MyClock::now();
	}

	void stop()
	{
		MyClock::time_point stopTime = MyClock::now();
		this->elapsedTime += stopTime - this->startTime;
	}

private:

	MyClock::time_point startTime;

	MyClock::duration elapsedTime;
};

class ProfilerScopedTimer
{
public:
	ProfilerScopedTimer(const std::string& name) :
		name(name)
{
		this->timer.start();
}

	~ProfilerScopedTimer()
	{
		this->timer.stop();
		std::cout << "Time: " << this->name << " took " << this->timer.getElapsedSeconds() * 1000  << " ms." << std::endl;
	}

private:
	std::string name;

	InternalTimer timer;
};

int main(int argc, char** argv)
{
	int numRepetitions = 10;
	Matrix44 qem = Vector4(0.1, 1, 1, 0).asDiagonal();
	qem(0, 2) = qem(2, 0) = -0.02;
	qem(0, 3) = qem(3, 0) = 0.3;
	qem(3, 3) = 0.47;
	std::cout << "qem: " << qem << std::endl;
	std::vector<Plane> constraints;
	for (int s = 0; s < 5; ++s)
	{
		std::cout << "Number of constraints: " << constraints.size() << std::endl;
		std::cout << "Solving " << numRepetitions << " repetitions." << std::endl;
		{
			ProfilerScopedTimer p("CGAL quadratic programming");
			for (int rep = 0; rep < numRepetitions; ++rep)
			{
				Program qp(CGAL::SMALLER, false, 0, false, 0);
				for (int i = 0; i < 3; ++i)
				{
					for (int j = 0; j < 3; ++j)
					{
						qp.set_d(i, j, qem(i, j));
					}

					for (int j = 0; j < constraints.size(); ++j)
					{
						qp.set_a(i, j, constraints[j].normal(i));
						qp.set_b(j, constraints[j].d); //TODO move to outer loop
					}
					
					qp.set_c(i, qem(3, i)); //FIXME: does not solve the right values
				}
				qp.set_c0(qem(3, 3));

				Solution s = CGAL::solve_quadratic_program(qp, ET());
				if (rep == 0)
				{
					for (auto it = s.variable_values_begin(); it < s.variable_values_end(); ++it)
					{
						std::cout << "x: " << CGAL::to_double(*it) << std::endl;
					}
					std::cout << "y: " << CGAL::to_double(s.objective_value()) << std::endl;
				}
			}
		}

		{
			ProfilerScopedTimer p("CGAL quadratic programming with direct matrices");
			for (int rep = 0; rep < numRepetitions; ++rep)
			{
				typedef CGAL::Quadratic_program_from_iterators< Real**, Real*, 
						CGAL::Const_oneset_iterator<CGAL::Comparison_result>, bool*,
						Real*, bool*, Real*, Real**, Real* > ProgramIt;
				int numVariables = 3;
				int numConstraints = constraints.size();
				std::vector< Real > Ax(numConstraints), Ay(numConstraints), Az(numConstraints);
				std::vector< Real > b(numConstraints);
				//TODO: maybe Eigen::Map< Eigen::Matrix< Real, 3, 3> >
				for (int i = 0; i < constraints.size(); ++i)
				{
					Ax[i] = constraints[i].normal(0);
					Ay[i] = constraints[i].normal(1);
					Az[i] = constraints[i].normal(2);
					b[i] = constraints[i].d;
				}
				Real *A[] = {&Ax[0], &Ay[0], &Az[0]};
				Real Dx[] = {qem(0, 0)};//, qem(0, 1), qem(0, 2)};
				Real Dy[] = {qem(1, 0), qem(1, 1)};//, qem(1, 2)};
				Real Dz[] = {qem(2, 0), qem(2, 1), qem(2, 2)};
				Real *D[] = {Dx, Dy, Dz};
				CGAL::Const_oneset_iterator<CGAL::Comparison_result> r(CGAL::SMALLER);
				Real c[] = {qem(3, 0), qem(3, 1), qem(3, 2)}; //FIXME: does not solve the right values
				Real c0 = qem(3, 3);
				bool fl[] = {false, false};
				Real l[] = {0, 0};
				bool fu[] = {false, false};
				Real u[] = {0, 0};
				ProgramIt qp(numVariables, numConstraints, A, &b[0], r, fl, l, fu, u, D, c, c0);
				CGAL::Quadratic_program_solution<Real> s = CGAL::solve_quadratic_program(qp, ET());
				if (rep == 0)
				{
					for (auto it = s.variable_values_begin(); it < s.variable_values_end(); ++it)
					{
						std::cout << "x: " << CGAL::to_double(*it) << std::endl;
					}
					std::cout << "y: " << CGAL::to_double(s.objective_value()) << std::endl;
					//TODO: why is this value different?
				}
			}
		}
		
		{
			ProfilerScopedTimer p("Eigen fork of QuadProg++");
			Eigen::MatrixXd G = qem.topLeftCorner<3, 3>();
			Eigen::VectorXd g0 = qem.topRightCorner<3, 1>();
			Eigen::MatrixXd CE(3, 0);
			Eigen::MatrixXd ce0(0, 0);
			Eigen::MatrixXd CI(3, constraints.size());
			Eigen::MatrixXd ci0(1, constraints.size());
			for (int i = 0; i < constraints.size(); ++i)
			{
				CI.col(i) = constraints[i].normal;
				ci0(i) = constraints[i].d;
			}
			Eigen::VectorXd x(3);
			QP::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
			
			Real cost = (x.transpose() * G * x)(0, 0) + (2 * g0.transpose() * x)(0, 0) + qem(3, 3);
			
			std::cout << "x: " << x.transpose() << std::endl;
			std::cout << "y: " << cost << std::endl;
		}

		{
			ProfilerScopedTimer p("usual solver with combinatorial subspaces");
			for (int rep = 0; rep < numRepetitions; ++rep)
			{
				//Search for valid minimizer that complies with 0 - 3 constraints
				bool found = false;
				Vector3 new_point;
				Real minimal_cost = std::numeric_limits<Real>::max();
				int used_m = -1;
				for(int m = 3; m >= 0; --m)
				{
					unsigned int n_subsets = Decimator::nSubsets(m, constraints.size());
					std::vector<unsigned int> subset;
					subset.reserve(m);
					for(unsigned int i = 0; i < m; ++i)
						subset.push_back(i);
					for(unsigned int i = 0; i < n_subsets; ++i)
					{
						Vector3 result;
						bool found_valid = Decimator::solveConstrainedMinimization(qem, constraints, subset, DecimationDirection::Outward, result);
						Vector4 result_homogeneous;
						result_homogeneous << result, 1;
						if(found_valid)
						{
							found = true;
							Real new_cost = result_homogeneous.transpose() * qem * result_homogeneous;
							if(new_cost < minimal_cost)// + 1e-16)
							{
								minimal_cost = new_cost;
								new_point = result;
								used_m = m;
							}
						}

						Decimator::nextSubset(subset, constraints.size());
					}
				}
				if (rep == 0)
				{
					std::cout << "new_point: " << new_point.transpose() << std::endl;
					std::cout << "minimal_cost: " << minimal_cost << std::endl;
					std::cout << "found: " << (found ? "true" : "false") << std::endl;
					
					Vector4 result_homogeneous;
					result_homogeneous << new_point, 1;
					Real new_cost = result_homogeneous.transpose() * qem * result_homogeneous;
					std::cout << "new_cost: " << new_cost << std::endl;
				}
			}
		}

		constraints.push_back(Plane(Vector3(0, 0.01, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0.02, 1), Vector3(0, 0, 0.1001)));
		constraints.push_back(Plane(Vector3(0, 0.04, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0.01, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0.02, 1), Vector3(0, 0, 0.1001)));
		constraints.push_back(Plane(Vector3(0, 0.04, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0, 1), Vector3(0, 0, 0.1)));
		constraints.push_back(Plane(Vector3(0, 0, 1), Vector3(0, 0, 0.1)));	
	}
	return 0;
}

