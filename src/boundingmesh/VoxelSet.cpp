//	Copyright (c) 2015, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without
// modification,
//	are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice,
// this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright
// notice, this
//	  list of conditions and the following disclaimer in the documentation
// and/or
//	  other materials provided with the distribution.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "boundingmesh/VoxelSet.h"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <tuple>

namespace boundingmesh {
// License of the following functions is public domain according to
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-Möller                              */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/
// Modified to use local types

#define X 0
#define Y 1
#define Z 2
#define FINDMINMAX(x0, x1, x2, min, max) \
  min = max = x0;                        \
  if (x1 < min) min = x1;                \
  if (x1 > max) max = x1;                \
  if (x2 < min) min = x2;                \
  if (x2 > max) max = x2;

#define AXISTEST_X01(a, b, fa, fb)                 \
  p0 = a * v0[Y] - b * v0[Z];                      \
  p2 = a * v2[Y] - b * v2[Z];                      \
  if (p0 < p2) {                                   \
    min = p0;                                      \
    max = p2;                                      \
  } else {                                         \
    min = p2;                                      \
    max = p0;                                      \
  }                                                \
  rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z]; \
  if (min > rad || max < -rad) return 0;

#define AXISTEST_X2(a, b, fa, fb)                  \
  p0 = a * v0[Y] - b * v0[Z];                      \
  p1 = a * v1[Y] - b * v1[Z];                      \
  if (p0 < p1) {                                   \
    min = p0;                                      \
    max = p1;                                      \
  } else {                                         \
    min = p1;                                      \
    max = p0;                                      \
  }                                                \
  rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z]; \
  if (min > rad || max < -rad) return 0;

#define AXISTEST_Y02(a, b, fa, fb)                 \
  p0 = -a * v0[X] + b * v0[Z];                     \
  p2 = -a * v2[X] + b * v2[Z];                     \
  if (p0 < p2) {                                   \
    min = p0;                                      \
    max = p2;                                      \
  } else {                                         \
    min = p2;                                      \
    max = p0;                                      \
  }                                                \
  rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z]; \
  if (min > rad || max < -rad) return 0;

#define AXISTEST_Y1(a, b, fa, fb)                  \
  p0 = -a * v0[X] + b * v0[Z];                     \
  p1 = -a * v1[X] + b * v1[Z];                     \
  if (p0 < p1) {                                   \
    min = p0;                                      \
    max = p1;                                      \
  } else {                                         \
    min = p1;                                      \
    max = p0;                                      \
  }                                                \
  rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z]; \
  if (min > rad || max < -rad) return 0;

#define AXISTEST_Z12(a, b, fa, fb)                 \
  p1 = a * v1[X] - b * v1[Y];                      \
  p2 = a * v2[X] - b * v2[Y];                      \
  if (p2 < p1) {                                   \
    min = p2;                                      \
    max = p1;                                      \
  } else {                                         \
    min = p1;                                      \
    max = p2;                                      \
  }                                                \
  rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y]; \
  if (min > rad || max < -rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)                  \
  p0 = a * v0[X] - b * v0[Y];                      \
  p1 = a * v1[X] - b * v1[Y];                      \
  if (p0 < p1) {                                   \
    min = p0;                                      \
    max = p1;                                      \
  } else {                                         \
    min = p1;                                      \
    max = p0;                                      \
  }                                                \
  rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y]; \
  if (min > rad || max < -rad) return 0;

bool PlaneBoxOverlap(const Vector3& normal, const Vector3& vert,
                     const Vector3& maxbox) {
  int q;
  Vector3 vmin, vmax;
  Real v;
  for (q = X; q <= Z; q++) {
    v = vert[q];
    if (normal[q] > 0.0) {
      vmin[q] = -maxbox[q] - v;
      vmax[q] = maxbox[q] - v;
    } else {
      vmin[q] = maxbox[q] - v;
      vmax[q] = -maxbox[q] - v;
    }
  }
  if (normal.dot(vmin) > 0.0) return false;
  if (normal.dot(vmax) >= 0.0) return true;
  return false;
}

bool TriBoxOverlap(const Vector3& boxcenter, const Vector3& boxhalfsize,
                   const Vector3& triver0, const Vector3& triver1,
                   const Vector3& triver2) {
  /*    use separating axis theorem to test overlap between triangle and box */
  /*    need to test for overlap in these directions: */
  /*    1) the {x,y,z}-directions (actually, since we use the AABB of the
   * triangle */
  /*       we do not even need to test these) */
  /*    2) normal of the triangle */
  /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
  /*       this gives 3x3=9 more tests */

  Vector3 v0, v1, v2;
  double min, max, p0, p1, p2, rad, fex, fey,
      fez;  // -NJMP- "d" local variable removed
  Vector3 normal, e0, e1, e2;

  /* This is the fastest branch on Sun */
  /* move everything so that the boxcenter is in (0,0,0) */

  v0 = triver0 - boxcenter;
  v1 = triver1 - boxcenter;
  v2 = triver2 - boxcenter;

  /* compute triangle edges */
  e0 = v1 - v0; /* tri edge 0 */
  e1 = v2 - v1; /* tri edge 1 */
  e2 = v0 - v2; /* tri edge 2 */

  /* Bullet 3:  */
  /*  test the 9 tests first (this was faster) */
  fex = fabs(e0[X]);
  fey = fabs(e0[Y]);
  fez = fabs(e0[Z]);

  AXISTEST_X01(e0[Z], e0[Y], fez, fey);
  AXISTEST_Y02(e0[Z], e0[X], fez, fex);
  AXISTEST_Z12(e0[Y], e0[X], fey, fex);

  fex = fabs(e1[X]);
  fey = fabs(e1[Y]);
  fez = fabs(e1[Z]);

  AXISTEST_X01(e1[Z], e1[Y], fez, fey);
  AXISTEST_Y02(e1[Z], e1[X], fez, fex);
  AXISTEST_Z0(e1[Y], e1[X], fey, fex);

  fex = fabs(e2[X]);
  fey = fabs(e2[Y]);
  fez = fabs(e2[Z]);

  AXISTEST_X2(e2[Z], e2[Y], fez, fey);
  AXISTEST_Y1(e2[Z], e2[X], fez, fex);
  AXISTEST_Z12(e2[Y], e2[X], fey, fex);

  /* Bullet 1: */
  /*  first test overlap in the {x,y,z}-directions */
  /*  find min, max of the triangle each direction, and test for overlap in */
  /*  that direction -- this is equivalent to testing a minimal AABB around */
  /*  the triangle against the AABB */

  /* test in X-direction */
  FINDMINMAX(v0[X], v1[X], v2[X], min, max);
  if (min > boxhalfsize[X] || max < -boxhalfsize[X]) return false;

  /* test in Y-direction */
  FINDMINMAX(v0[Y], v1[Y], v2[Y], min, max);
  if (min > boxhalfsize[Y] || max < -boxhalfsize[Y]) return false;

  /* test in Z-direction */
  FINDMINMAX(v0[Z], v1[Z], v2[Z], min, max);
  if (min > boxhalfsize[Z] || max < -boxhalfsize[Z]) return false;

  /* Bullet 2: */
  /*  test if the box intersects the plane of the triangle */
  /*  compute plane equation of triangle: normal*x+d=0 */
  normal = e0.cross(e1);

  if (!PlaneBoxOverlap(normal, v0, boxhalfsize)) return false;
  return true; /* box and triangle overlaps */
}

Voxel::Voxel() {}

Voxel::Voxel(Index x, Index y, Index z, VoxelType type)
    : x_(x), y_(y), z_(z), type_(type), triangles_() {}

Voxel::Voxel(const Voxel& voxel)
    : x_(voxel.x_),
      y_(voxel.y_),
      z_(voxel.z_),
      type_(voxel.type_),
      triangles_(voxel.triangles_) {}

Voxel::~Voxel() {}

Voxel& Voxel::operator=(Voxel other) {
  swap(*this, other);
  return *this;
}

void swap(Voxel& first, Voxel& second) {
  std::swap(first.x_, second.x_);
  std::swap(first.y_, second.y_);
  std::swap(first.z_, second.z_);
  std::swap(first.type_, second.type_);
  first.triangles_.swap(second.triangles_);
}

Index Voxel::x() const { return x_; }

Index Voxel::y() const { return y_; }

Index Voxel::z() const { return z_; }

Index Voxel::coordinate(int dimension) const {
  assert(dimension >= 0 && dimension < 3);
  switch (dimension) {
    case 0:
      return x_;
    case 1:
      return y_;
    case 2:
      return z_;
    default:
      return 0;
  };
}

VoxelType Voxel::type() const { return type_; }

unsigned int Voxel::nTriangles() const { return triangles_.size(); }

Index Voxel::triangle(unsigned int i) const { return triangles_[i]; }

void Voxel::addTriangle(Index triangle) { triangles_.push_back(triangle); }

VoxelSet::VoxelSet() : origin_(0, 0, 0), voxel_size_(0), mesh_(NULL) {
  resolution_[0] = 0;
  resolution_[1] = 0;
  resolution_[2] = 0;
}

VoxelSet::VoxelSet(const VoxelSet& voxel_set) {
  origin_ = voxel_set.origin_;
  voxel_size_ = voxel_set.voxel_size_;
  resolution_[0] = voxel_set.resolution_[0];
  resolution_[1] = voxel_set.resolution_[1];
  resolution_[2] = voxel_set.resolution_[2];
  mesh_ = voxel_set.mesh_;
  voxels_ = voxel_set.voxels_;
  grid_ = voxel_set.grid_;
}

VoxelSet::VoxelSet(std::shared_ptr<Mesh> triangle_mesh, Real voxel_size,
                   bool newVersion) {
  //"Rasterize" triangle mesh to generate a voxel set
  mesh_ = triangle_mesh;
  voxel_size_ = voxel_size;

  // Set up the grid dimensions
  // First compute bounding box of the mesh
  Vector3 bounding_box_min = Vector3(std::numeric_limits<Real>::max(),
                                     std::numeric_limits<Real>::max(),
                                     std::numeric_limits<Real>::max());
  Vector3 bounding_box_max = Vector3(std::numeric_limits<Real>::min(),
                                     std::numeric_limits<Real>::min(),
                                     std::numeric_limits<Real>::min());
  for (unsigned int i = 0; i < triangle_mesh->nVertices(); ++i) {
    Vector3 position = triangle_mesh->vertex(i).position();
    bounding_box_min(0) = std::min(bounding_box_min(0), position(0));
    bounding_box_min(1) = std::min(bounding_box_min(1), position(1));
    bounding_box_min(2) = std::min(bounding_box_min(2), position(2));

    bounding_box_max(0) = std::max(bounding_box_max(0), position(0));
    bounding_box_max(1) = std::max(bounding_box_max(1), position(1));
    bounding_box_max(2) = std::max(bounding_box_max(2), position(2));
  }

  // Compute voxel resolution along the axes
  // All voxels are cubes with side size voxel_size
  //-> Round up, extend in all directions by 1
  // Adjust origin so the mesh is centered
  Vector3 size = bounding_box_max - bounding_box_min;
  for (int i = 0; i < 3; ++i) {
    resolution_[i] = (Index)(size(i) / voxel_size_);
    if (resolution_[i] * voxel_size_ < size(i)) resolution_[i] += 1;
    resolution_[i] += 2;
    origin_(i) =
        bounding_box_min(i) - ((resolution_[i] * voxel_size_) - size(i)) / 2;
  }

  Vector3 check_origin = origin_;

  if (resolution_[0] * resolution_[1] * resolution_[2] > 256 * 256 * 256)
    std::cout << "Very high resolution detected, might not fit into memory: "
              << resolution_[0] << ", " << resolution_[1] << ", "
              << resolution_[2] << std::endl;
  grid_ = std::vector<int>(resolution_[0] * resolution_[1] * resolution_[2]);
  for (unsigned int i = 0; i < grid_.size(); ++i) grid_[i] = -1;

  // Rasterize triangles to find voxels intersecting the surface of the mesh
  for (unsigned int i = 0; i < triangle_mesh->nTriangles(); ++i) {
    const Triangle& triangle = triangle_mesh->triangle(i);

    // Calculate bounding box of triangle
    Vector3 triangle_box_min = Vector3(std::numeric_limits<Real>::max(),
                                       std::numeric_limits<Real>::max(),
                                       std::numeric_limits<Real>::max());
    Vector3 triangle_box_max = Vector3(std::numeric_limits<Real>::lowest(),
                                       std::numeric_limits<Real>::lowest(),
                                       std::numeric_limits<Real>::lowest());
    Vector3 vertices[3];
    for (unsigned int j = 0; j < 3; ++j) {
      Vector3 position = triangle_mesh->vertex(triangle.vertex(j)).position();
      vertices[j] = position;
      triangle_box_min(0) = std::min(triangle_box_min(0), position(0));
      triangle_box_min(1) = std::min(triangle_box_min(1), position(1));
      triangle_box_min(2) = std::min(triangle_box_min(2), position(2));

      triangle_box_max(0) = std::max(triangle_box_max(0), position(0));
      triangle_box_max(1) = std::max(triangle_box_max(1), position(1));
      triangle_box_max(2) = std::max(triangle_box_max(2), position(2));
    }

    // Calculate range of voxels intersecting with the bounding box
    Index min_indices[3];
    Index max_indices[3];

    for (unsigned int j = 0; j < 3; ++j) {
      min_indices[j] = (int)((triangle_box_min(j) - origin_(j)) / voxel_size_);
      if (origin_(j) + (min_indices[j] * voxel_size_) + voxel_size_ / 2 <
          triangle_box_min(j))
        min_indices[j]++;
      max_indices[j] = (int)((triangle_box_max(j) - origin_(j)) / voxel_size_);
      if (origin_(j) + (max_indices[j] * voxel_size_) + voxel_size_ / 2 <
          triangle_box_max(j))
        max_indices[j]++;
    }

    // Test every voxel in range for intersection with the triangle
    for (int vox_x = min_indices[0]; vox_x <= max_indices[0]; vox_x++) {
      for (int vox_y = min_indices[1]; vox_y <= max_indices[1]; vox_y++) {
        for (int vox_z = min_indices[2]; vox_z <= max_indices[2]; vox_z++) {
          Vector3 voxel_position = origin_ +
                                   Vector3(voxel_size_, 0, 0) * vox_x +
                                   Vector3(0, voxel_size_, 0) * vox_y +
                                   Vector3(0, 0, voxel_size_) * vox_z;
          Vector3 voxel_halfdiagonal =
              Vector3(voxel_size_ / 2, voxel_size_ / 2, voxel_size_ / 2);

          if (TriBoxOverlap(voxel_position, voxel_halfdiagonal, vertices[0],
                            vertices[1], vertices[2])) {
            unsigned int grid_index = vox_x + vox_y * resolution_[0] +
                                      vox_z * resolution_[0] * resolution_[1];
            if (grid_[grid_index] == -1) {
              Voxel new_voxel(vox_x, vox_y, vox_z, SURFACE);
              new_voxel.addTriangle(i);
              grid_[grid_index] = voxels_.size();
              voxels_.push_back(new_voxel);
            } else {
              voxels_[grid_[grid_index]].addTriangle(i);
            }
          }
        }
      }
    }
  }
  // The new version uses an alternative approach to fill the interior, as the
  // ray casting approach
  // sometimes currently seems to leave holes within the interior
  if (!newVersion) {
    // Fill interior volume of shape
    // Cast rays through X-Y plane in direction -Z
    // fill empty areas after entering and before leaving the surface (check ray
    // against triangle plane)
    Vector3 ray_direction(0, 0, -1);
    for (int vox_x = 0; vox_x < resolution_[0]; vox_x++) {
      for (int vox_y = 0; vox_y < resolution_[1]; vox_y++) {
        int last_found_voxel = -1;
        for (int vox_z = 0; vox_z < resolution_[2]; vox_z++) {
          unsigned int grid_index = vox_x + vox_y * resolution_[0] +
                                    vox_z * resolution_[0] * resolution_[1];
          if (grid_[grid_index] == -1 && last_found_voxel != -1) {
            const Voxel& vox = voxel(last_found_voxel);
            int fill = 0;
            for (unsigned int triangle_i = 0; triangle_i < vox.nTriangles();
                 ++triangle_i) {
              const Triangle& triangle =
                  triangle_mesh->triangle(vox.triangle(triangle_i));
              Real side = triangle.plane().normal.dot(ray_direction);
              if (side > 0 && fill == 0) {
                fill = 1;
              } else if (side <= 0) {
                fill = -1;
                break;
              }
            }
            if (fill == 1) {
              Voxel new_voxel(vox_x, vox_y, vox_z, INNER);
              grid_[grid_index] = voxels_.size();
              voxels_.push_back(new_voxel);
            }
          } else if (grid_[grid_index] != -1) {
            last_found_voxel = grid_[grid_index];
          }
        }
      }
    }

  } else {
    // Mark all outside Voxels (the ones reachable from a border) with -2
    int neighbours[6][3] = {{0, 0, 1},  {0, 0, -1}, {0, 1, 0},
                            {0, -1, 0}, {1, 0, 0},  {-1, 0, 0}};
    for (int vox_x = 0; vox_x < resolution_[0]; vox_x++) {
      for (int vox_y = 0; vox_y < resolution_[1]; vox_y++) {
        for (int vox_z = 0; vox_z < resolution_[2]; vox_z++) {
          unsigned int grid_index = vox_x + vox_y * resolution_[0] +
                                    vox_z * resolution_[0] * resolution_[1];
          if (grid_[grid_index] == -1 &&
              (vox_x == 0 || vox_y == 0 || vox_z == 0 ||
               vox_x == resolution_[0] || vox_y == resolution_[1] ||
               vox_z == resolution_[2])) {
            std::queue<std::tuple<int, int, int>> q;
            q.push(std::make_tuple(vox_x, vox_y, vox_z));
            grid_[grid_index] = -2;
            while (!q.empty()) {
              std::tuple<int, int, int> current = q.front();
              q.pop();
              int x = std::get<0>(current);
              int y = std::get<1>(current);
              int z = std::get<2>(current);
              for (int i = 0; i < 6; i++) {
                int ax = x + neighbours[i][0];
                int ay = y + neighbours[i][1];
                int az = z + neighbours[i][2];
                unsigned int ni = ax + ay * resolution_[0] +
                                  az * resolution_[0] * resolution_[1];
                if (ni < 0 || ni > grid_.size() || ax < 0 || ay < 0 || az < 0 ||
                    ax >= resolution_[0] || ay >= resolution_[1] ||
                    az >= resolution_[2]) {
                  continue;
                }
                if (grid_[ni] == -1) {
                  grid_[ni] = -2;
                  q.push(std::make_tuple(ax, ay, az));
                }
              }
            }
          }
        }
      }
    }
    // Mark all voxels not marked as outside, that are not part of the surface
    // as inner voxel
    for (int vox_x = 0; vox_x < resolution_[0]; vox_x++) {
      for (int vox_y = 0; vox_y < resolution_[1]; vox_y++) {
        for (int vox_z = 0; vox_z < resolution_[2]; vox_z++) {
          unsigned int grid_index = vox_x + vox_y * resolution_[0] +
                                    vox_z * resolution_[0] * resolution_[1];
          if (grid_[grid_index] == -1) {
            Voxel new_voxel(vox_x, vox_y, vox_z, INNER);
            grid_[grid_index] = voxels_.size();
            voxels_.push_back(new_voxel);
          }
        }
      }
    }
    // Revert all -2 marks to -1
    for (int vox_x = 0; vox_x < resolution_[0]; vox_x++) {
      for (int vox_y = 0; vox_y < resolution_[1]; vox_y++) {
        for (int vox_z = 0; vox_z < resolution_[2]; vox_z++) {
          unsigned int grid_index = vox_x + vox_y * resolution_[0] +
                                    vox_z * resolution_[0] * resolution_[1];
          if (grid_[grid_index] == -2) {
            grid_[grid_index] = -1;
          }
        }
      }
    }

    // Use BFS to fix empty cells in the inside (convert them to inner cells)
    // Needed to fix bugs from previous procedure
    /*for(int vox_x = 0; vox_x < resolution_[0]; vox_x++)
     {
     for(int vox_y = 0; vox_y < resolution_[1]; vox_y++)
     {
     for(int vox_z = 0; vox_z < resolution_[2]; vox_z++)
     {
     unsigned int grid_index = vox_x + vox_y*resolution_[0] +
     vox_z*resolution_[0]*resolution_[1];
     if(grid_[grid_index] == -1) {
     continue;
     }
     const Voxel& vox = voxel(grid_[grid_index]);
     if (vox.type() == INNER) {
     for (int i = 0; i < 6; i++) {
     int nx = vox_x + neighbours[i][0];
     int ny = vox_y + neighbours[i][1];
     int nz = vox_z + neighbours[i][2];
     unsigned int neighbour_index = nx + ny*resolution_[0] +
     nz*resolution_[0]*resolution_[1];
     if (neighbour_index < 0 || neighbour_index > grid_.size()) {
     continue;
     }
     if (grid_[neighbour_index] == -1) {
     std::queue<std::tuple<int,int,int>> q;
     q.push(std::make_tuple(nx, ny, nz));
     Voxel new_voxel(nx, ny, nz, INNER);
     grid_[grid_index] = voxels_.size();
     voxels_.push_back(new_voxel);
     while (!q.empty()) {
     std::tuple<int,int,int> current = q.front();
     q.pop();
     int x = std::get<0>(current);
     int y = std::get<1>(current);
     int z = std::get<2>(current);
     for (int i = 0; i < 6; i++) {
     int ax = x + neighbours[i][0];
     int ay = y + neighbours[i][1];
     int az = z + neighbours[i][2];
     unsigned int ni = ax + ay*resolution_[0] +
     az*resolution_[0]*resolution_[1];
     if (ni < 0 || ni > grid_.size() || ax < 0 || ay < 0 || az < 0 || ax >=
     resolution_[0] || ay >= resolution_[1] || az >= resolution_[2]) {
     continue;
     }
     if (grid_[ni] == -1) {
     Voxel new_voxel(ax, ay, az, INNER);
     grid_[ni] = voxels_.size();
     voxels_.push_back(new_voxel);
     q.push(std::make_tuple(ax, ay, az));
     }
     }
     }
     }
     }
     }
     }
     }
     }*/
  }

  std::cout << "Finished voxeling: Dimensions " << resolution_[0] << " x "
            << resolution_[1] << " x " << resolution_[2] << ";  generated "
            << voxels_.size() << " voxels" << std::endl;
}

VoxelSet::~VoxelSet() {}

VoxelSet& VoxelSet::operator=(VoxelSet other) {
  swap(*this, other);
  return *this;
}

void swap(VoxelSet& first, VoxelSet& second) {
  std::swap(first.origin_, second.origin_);
  std::swap(first.voxel_size_, second.voxel_size_);
  std::swap(first.resolution_[0], second.resolution_[0]);
  std::swap(first.resolution_[1], second.resolution_[1]);
  std::swap(first.resolution_[2], second.resolution_[2]);
  std::swap(first.mesh_, second.mesh_);
  first.voxels_.swap(second.voxels_);
  first.grid_.swap(second.grid_);
}

unsigned int VoxelSet::nVoxels() const { return voxels_.size(); }

const Voxel& VoxelSet::voxel(Index i) const { return voxels_[i]; }

const Vector3& VoxelSet::origin() const { return origin_; }

Real VoxelSet::voxelSize() const { return voxel_size_; }

unsigned int VoxelSet::resolution(int dimension) const {
  assert(0 <= dimension && dimension < 3);
  return resolution_[dimension];
}

std::shared_ptr<Mesh> VoxelSet::mesh() const { return mesh_; }

int VoxelSet::voxelAt(unsigned int x, unsigned int y, unsigned int z) const {
  assert(x >= 0 && x < resolution_[0]);
  assert(y >= 0 && y < resolution_[1]);
  assert(z >= 0 && z < resolution_[2]);
  return grid_[x + y * resolution_[0] + z * resolution_[0] * resolution_[1]];
}

void VoxelSet::addVoxel(const Voxel& voxel) {
  assert(voxel.x() >= 0 && voxel.x() < resolution_[0]);
  assert(voxel.y() >= 0 && voxel.y() < resolution_[1]);
  assert(voxel.z() >= 0 && voxel.z() < resolution_[2]);
  grid_[voxel.x() + voxel.y() * resolution_[0] +
        voxel.z() * resolution_[0] * resolution_[1]] = voxels_.size();

  voxels_.push_back(voxel);
}

Real VoxelSet::volume() {
  return voxels_.size() * voxel_size_ * voxel_size_ * voxel_size_;
}

Vector3 VoxelSet::computePosition(const Voxel& voxel) const {
  return origin_ + Vector3(voxel_size_, 0, 0) * voxel.x() +
         Vector3(voxel_size_, 0, 0) * voxel.y() +
         Vector3(voxel_size_, 0, 0) * voxel.z();
}

void VoxelSet::writeWRL(std::string filename) {
  std::ofstream file(filename.c_str());
  file << "#VRML V2.0 utf8" << std::endl << std::endl;
  file << "#Created by boundingmesh" << std::endl << std::endl;
  file << "Transform {" << std::endl;
  file << "\t translation " << origin_(0) << " " << origin_(1) << " "
       << origin_(2) << std::endl;
  file << "\t scale " << voxel_size_ << " " << voxel_size_ << " " << voxel_size_
       << std::endl;
  file << "\t children [ " << std::endl;
  for (unsigned int i = 0; i < nVoxels(); ++i) {
    file << "\t\tTransform {" << std::endl;
    file << "\t\t\t translation " << voxel(i).x() << " " << voxel(i).y() << " "
         << voxel(i).z() << std::endl;
    file << "\t\t\t children [ " << std::endl;

    file << "\t\t\t\tShape {" << std::endl;
    file << "\t\t\t\t\tappearance Appearance {" << std::endl;
    file << "\t\t\t\t\t\tmaterial Material {" << std::endl;
    if (voxel(i).type() == SURFACE)
      file << "\t\t\t\t\t\t\tdiffuseColor 1 1 1" << std::endl;
    else if (voxel(i).type() == INNER)
      file << "\t\t\t\t\t\t\tdiffuseColor 0 0 1" << std::endl;
    file << "\t\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t\tgeometry Sphere {" << std::endl;
    file << "\t\t\t\t\t\tradius " << 0.25 << std::endl;
    file << "\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t}" << std::endl;
    file << "\t\t\t ]" << std::endl;
    file << "\t\t}";
  }
  file << "\t ]" << std::endl;
  file << "}";
}
}
