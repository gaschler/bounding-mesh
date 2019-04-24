//  Copyright (c) 2019, the boundingmesh authors
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
// modification,
//  are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
// this
//    list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright
// notice, this
//    list of conditions and the following disclaimer in the documentation
// and/or
//    other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR
//  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON
//  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "boundingmesh/SimpleOptimizer.h"

#include "boundingmesh/Decimator.h"
#include "boundingmesh/MetricGenerator.h"

#if USE_EIGENQUADPROG
#include "../../thirdparty/EigenQP.h"
#endif  // USE_EIGENQUADPROG

namespace boundingmesh {
namespace {

unsigned int nSubsets(unsigned int subset_size, unsigned int total_size) {
  if (subset_size > total_size) return 0;
  // Compute binomial coefficient "total_size over subset_size".
  unsigned int result = 1;
  for (unsigned int i = 0; i < subset_size; ++i) result *= total_size - i;
  for (unsigned int i = 0; i < subset_size; ++i) result /= i + 1;
  return result;
}

void nextSubset(std::vector<unsigned int>& indices_subset,
                unsigned int total_size) {
  for (int i = (int)indices_subset.size() - 1; i >= 0; --i) {
    unsigned int max_next_index =
        indices_subset[i] + 1 + (indices_subset.size() - 1 - i);
    if (max_next_index < total_size) {
      indices_subset[i] = indices_subset[i] + 1;
      for (unsigned int j = i + 1; j < indices_subset.size(); ++j) {
        indices_subset[j] = indices_subset[j - 1] + 1;
      }
      break;
    }
  }
}

Vector3 minimizeSubspace(const Matrix44& quadratic_cost) {
  // Direct solution
  Eigen::Matrix<Real, 4, 3> E = Eigen::Matrix<Real, 4, 3>::Identity();
  Eigen::Matrix<Real, 4, 1> f(0, 0, 0, 1);
  Eigen::Matrix<Real, 3, 3> A = E.transpose() * quadratic_cost * E;
  Eigen::Matrix<Real, 3, 1> b_ = -(E.transpose() * quadratic_cost * f);
  Eigen::Matrix<Real, 3, 1> result = A.ldlt().solve(b_);
  return result;
}

Vector3 minimizeSubspace(const Matrix44& quadratic_cost, Plane plane) {
  // Numerically stable orthonormals
  Vector3 some_unit(1, 0, 0);
  Vector3 some_other_unit(0, 1, 0);
  Vector3 plane_direction_1, plane_direction_2;

  if ((plane.normal.transpose() * some_unit).norm() <
      (plane.normal.transpose() * some_other_unit).norm())
    plane_direction_1 = plane.normal.cross(some_unit);
  else
    plane_direction_1 = plane.normal.cross(some_other_unit);

  plane_direction_1.normalize();
  plane_direction_2 = plane.normal.cross(plane_direction_1);
  plane_direction_2.normalize();

  Eigen::Matrix<Real, 4, 2> E;
  E << plane_direction_1, plane_direction_2, 0, 0;
  Eigen::Matrix<Real, 4, 1> f;
  f << (-plane.d) * plane.normal, 1;
  // Direct solution
  Eigen::Matrix<Real, 2, 1> m = (E.transpose() * quadratic_cost * E).inverse() *
                                (-(E.transpose() * quadratic_cost * f));
  Eigen::Matrix<Real, 4, 1> minimizer = E * m + f;
  Vector3 result = minimizer.topRows(3);
  return result;
}

Vector3 minimizeSubspace(const Matrix44& quadratic_cost, Plane plane1,
                         Plane plane2) {
  Vector3 point_on_p = plane1.normal * (-plane1.d);
  Vector3 along_p_to_edge =
      plane1.normal.cross(plane1.normal.cross(plane2.normal));
  Real mu = ((-plane2.d) - plane2.normal.transpose() * point_on_p) /
            (plane2.normal.transpose() * along_p_to_edge);
  Vector3 point_on_edge = point_on_p + mu * along_p_to_edge;
  if (std::abs(point_on_edge.transpose() * plane1.normal + plane1.d) >
          epsilon ||
      std::abs(point_on_edge.transpose() * plane2.normal + plane2.d) >
          epsilon) {
    // std::cout << "Debug: m=2: point_on_edge is not on edge (p,q)." <<
    // std::endl;
    // That means the planes are close to parallel and a bad choice anyways
    return Vector3::Zero();
  } else {
    // std::cout << "Debug: point_on_edge is on edge (p,q)." << std::endl;
  }

  // Constraint subspace { E m + f | x }
  Eigen::Matrix<Real, 4, 1> E;
  E << plane1.normal.cross(plane2.normal), 0;
  Eigen::Matrix<Real, 4, 1> f;
  f << point_on_edge, 1;

  // Direct solution
  Real m = -static_cast<Real>(E.transpose() * quadratic_cost * f) /
           static_cast<Real>(E.transpose() * quadratic_cost * E);
  Eigen::Matrix<Real, 4, 1> minimizer = E * m + f;
  Vector3 result = minimizer.topRows(3);
  return result;
}

Vector3 minimizeSubspace(const Matrix44& quadratic_cost, Plane plane1,
                         Plane plane2, Plane plane3) {
  Eigen::Matrix<Real, 3, 3> m, m_inverse;
  m << plane1.normal.transpose(), plane2.normal.transpose(),
      plane3.normal.transpose();
  m_inverse = m.inverse();

  Vector3 b;
  b << (-plane1.d), (-plane2.d), (-plane3.d);
  if (m_inverse.norm() > 1e3) {
    // std::cout << "Warning, intersection is ill-conditioned." <<
    // m_inverse.norm() << std::endl;
    return Vector3::Zero();
  }
  Vector3 result = m_inverse * b;  // m.colPivHouseholderQr().solve(b);
  return result;
}

bool solveConstrainedMinimization(const Matrix44& qem,
                                  const std::vector<Plane>& constraints,
                                  const std::vector<unsigned int>& subset,
                                  DecimationDirection direction,
                                  Vector3& result) {
  // Guess solution for the minimization problem by solving only for a subset of
  // constraints.
  // Return a bool indicating if the guess is allowed by all constraints.(even
  // then it might be non-optimal)
  Vector3 result_position = Vector3::Zero();
  switch (subset.size()) {
    case 0:
      result_position = minimizeSubspace(qem);
      break;
    case 1:
      result_position = minimizeSubspace(qem, constraints[subset[0]]);
      break;
    case 2:
      result_position =
          minimizeSubspace(qem, constraints[subset[0]], constraints[subset[1]]);
      break;
    case 3:
      result_position =
          minimizeSubspace(qem, constraints[subset[0]], constraints[subset[1]],
                           constraints[subset[2]]);
      break;
    default:
      assert(false);
  }

  if (!result_position.array().allFinite()) return false;

  // Check if minimizer fulfills all constraints
  Vector4 result_homogeneous;
  result_homogeneous << result_position, 1;

  bool valid_solution = true;
  for (unsigned int i = 0; i < constraints.size(); ++i) {
    if (direction == Outward &&
        constraints[i].vector().transpose() * result_homogeneous + epsilon <
            0) {
      return false;
    } else if (direction == Inward &&
               constraints[i].vector().transpose() * result_homogeneous -
                       epsilon >
                   0) {
      return false;
    }
  }
  result = result_position;

  return valid_solution;
}

bool solveConstrainedMinimizationInequalities(
    const Matrix44& qem, const std::vector<Plane>& constraints,
    DecimationDirection direction, Vector3& result, Real& result_cost) {
#if USE_EIGENQUADPROG
  Eigen::MatrixXd G = 2 * qem.topLeftCorner<3, 3>();
  Eigen::VectorXd g0 = 2 * qem.topRightCorner<3, 1>();
  Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(3, 0);
  Eigen::VectorXd ce0 = Eigen::VectorXd::Zero(0);
  Eigen::MatrixXd CI(3, constraints.size());
  Eigen::VectorXd ci0(constraints.size());
  for (int i = 0; i < constraints.size(); ++i) {
    CI.col(i) = constraints[i].normal;
    ci0(i) = constraints[i].d;
  }
  Eigen::VectorXd x(3);
  Real cost;
  try {
    cost = QP::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
  } catch (const ::std::logic_error& e) {
    return false;
  }
  if (cost == std::numeric_limits<Real>::infinity()) {
    return false;
  }
  result_cost = cost + qem(3, 3);
  result = x;
  return true;
#else
  // Search for valid minimizer that complies with 0 - 3 constraints
  bool found = false;
  Real minimal_cost = std::numeric_limits<Real>::max();
  int used_m = -1;
  Vector3 new_point;
  for (int m = 3; m >= 0; --m) {
    unsigned int n_subsets = nSubsets(m, constraints.size());
    std::vector<unsigned int> subset;
    subset.reserve(m);
    for (unsigned int i = 0; i < m; ++i) subset.push_back(i);
    for (unsigned int i = 0; i < n_subsets; ++i) {
      Vector3 result;
      bool found_valid = solveConstrainedMinimization(qem, constraints, subset,
                                                      direction, result);
      Vector4 result_homogeneous;
      result_homogeneous << result, 1;
      if (found_valid) {
        Real new_cost =
            result_homogeneous.transpose() * qem * result_homogeneous;
        if (new_cost < minimal_cost)  // + 1e-16)
        {
          found = true;
          minimal_cost = new_cost;
          new_point = result;
          used_m = m;
        }
      }
      nextSubset(subset, constraints.size());
    }
    if (found) break;
  }

  if (minimal_cost < -epsilon)  // Catch really wrong cases
  {
    std::cerr << "Warning: Bad contraction, negative cost: " << minimal_cost
              << std::endl
              << "qem: " << qem << std::endl
              << "point" << new_point << std::endl;
    minimal_cost = std::numeric_limits<Real>::max();
  }

  result_cost = minimal_cost;
  result = new_point;

  return found;
#endif
}

}  // namespace

bool SimpleOptimizer::optimize(const Matrix44& qem,
                               const std::vector<Plane>& constraints,
                               bool constrain_outside_planes, Vector3* solution,
                               Real* cost) const {
  DecimationDirection direction = constrain_outside_planes
                                      ? DecimationDirection::Outward
                                      : DecimationDirection::Inward;
  return solveConstrainedMinimizationInequalities(qem, constraints, direction,
                                                  *solution, *cost);
}

}  // namespace boundingmesh
