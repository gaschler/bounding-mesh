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

#include <memory>

#include "boundingmesh/OptimizerInterface.h"
#include "boundingmesh/Primitives.h"
#include "boundingmesh/SimpleOptimizer.h"
#include "gtest/gtest.h"

namespace boundingmesh {
namespace {

constexpr double kEpsilon = 1e-8;

Matrix44 GetSquaredDistanceToPointMatrix(const Vector3& point) {
  Matrix44 point_to_origin = Matrix44::Identity();
  point_to_origin.col(3).head<3>() = -point;
  point_to_origin(3, 3) = 0;
  return point_to_origin.transpose() * point_to_origin;
}

TEST(OptimizerTest, GetSquaredDistanceToPointMatrix) {
  Vector3 p(0.1, 0.5, 0.8);
  const Matrix44 distance_to_p = GetSquaredDistanceToPointMatrix(p);
  const Vector4 p_h(p(0), p(1), p(2), 1);
  double p_cost = (p_h.transpose() * distance_to_p * p_h)(0, 0);
  EXPECT_NEAR(p_cost, 0, kEpsilon);
  const Vector4 p_h_plus_1(p(0), p(1) + 1, p(2), 1);
  double p_plus_1_cost =
      (p_h_plus_1.transpose() * distance_to_p * p_h_plus_1)(0, 0);
  EXPECT_NEAR(p_plus_1_cost, 1, kEpsilon);
}

TEST(OptimizerTest, SimpleOptimizerNoConstraints) {
  std::unique_ptr<OptimizerInterface> optimizer(new SimpleOptimizer);
  Vector3 solution;
  Real cost;
  EXPECT_TRUE(
      optimizer->optimize(Matrix44::Identity(), {}, true, &solution, &cost));
  EXPECT_LT(solution.norm(), kEpsilon);
  EXPECT_NEAR(cost, 1, kEpsilon);
  const Vector3 p(0.1, 0.5, 0.8);
  const Matrix44 distance_to_p = GetSquaredDistanceToPointMatrix(p);
  EXPECT_TRUE(optimizer->optimize(distance_to_p, {}, true, &solution, &cost));
  EXPECT_LT((solution - p).norm(), kEpsilon);
  EXPECT_LT(cost, kEpsilon);
}

TEST(OptimizerTest, SimpleOptimizerConstraints) {
  std::unique_ptr<OptimizerInterface> optimizer(new SimpleOptimizer);
  Vector3 p(0.1, 0.5, 0.8);
  const Matrix44 distance_to_p = GetSquaredDistanceToPointMatrix(p);
  const Plane plane_through_p(Vector3(0, 0, 1), p);
  Vector3 solution;
  Real cost;
  // Point is on constraint plane.
  EXPECT_TRUE(optimizer->optimize(distance_to_p, {plane_through_p}, true,
                                  &solution, &cost));
  EXPECT_LT((solution - p).norm(), kEpsilon);
  EXPECT_LT(cost, kEpsilon);
  // Point is below constraint plane, solution is on plane.
  const Plane plane_z_1(Vector3(0, 0, 1), Vector3(1, 0, 1), Vector3(0, 1, 1));
  const Vector3 expected_solution_z_1(p[0], p[1], 1);
  const double expected_cost_z_1 = (expected_solution_z_1 - p).squaredNorm();
  EXPECT_TRUE(
      optimizer->optimize(distance_to_p, {plane_z_1}, true, &solution, &cost));
  EXPECT_LT((solution - expected_solution_z_1).norm(), kEpsilon);
  EXPECT_NEAR(cost, expected_cost_z_1, kEpsilon);
  // Second plane for x > 1.
  const Plane plane_x_1(Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(1, 1, 1));
  const Vector3 expected_solution_xz_1(1, p[1], 1);
  const double expected_cost_xz_1 = (expected_solution_xz_1 - p).squaredNorm();
  EXPECT_TRUE(optimizer->optimize(distance_to_p, {plane_z_1, plane_x_1}, true,
                                  &solution, &cost));
  EXPECT_LT((solution - expected_solution_xz_1).norm(), kEpsilon);
  EXPECT_NEAR(cost, expected_cost_xz_1, kEpsilon);
  // Third plane for y > 1.
  const Plane plane_y_1(Vector3(0, 1, 0), Vector3(0, 1, 1), Vector3(1, 1, 1));
  const Vector3 expected_solution_xyz_1(1, 1, 1);
  const double expected_cost_xyz_1 =
      (expected_solution_xyz_1 - p).squaredNorm();
  EXPECT_TRUE(optimizer->optimize(distance_to_p,
                                  {plane_z_1, plane_x_1, plane_y_1}, true,
                                  &solution, &cost));
  EXPECT_LT((solution - expected_solution_xyz_1).norm(), kEpsilon);
  EXPECT_NEAR(cost, expected_cost_xyz_1, kEpsilon);
}

TEST(OptimizerTest, SimpleOptimizerContradictingConstraints) {
  std::unique_ptr<OptimizerInterface> optimizer(new SimpleOptimizer);
  Vector3 solution;
  Real cost;
  // Contradicting constraints z > 1 and z < -1.
  const Plane plane_z_1(Vector3(0, 0, 1), Vector3(1, 0, 1), Vector3(0, 1, 1));
  const Plane plane_z_minus_1(Vector3(0, 1, -1), Vector3(1, 0, -1),
                              Vector3(0, 0, -1));
  EXPECT_FALSE(optimizer->optimize(Matrix44::Identity(),
                                   {plane_z_1, plane_z_minus_1}, true,
                                   &solution, &cost));
  const Plane plane_x_1(Vector3(1, 0, 0), Vector3(1, 1, 0), Vector3(1, 1, 1));
  EXPECT_FALSE(optimizer->optimize(Matrix44::Identity(),
                                   {plane_z_1, plane_z_minus_1, plane_x_1},
                                   true, &solution, &cost));
}

}  // namespace
}  // namespace boundingmesh
