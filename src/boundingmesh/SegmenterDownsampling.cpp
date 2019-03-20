//	Copyright (c) 2016, Philipp Gergen, Andre Gaschler
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

#include "SegmenterDownsampling.h"
#include "Segmenter.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <Eigen/SVD>

#ifdef CGAL_AVAILABLE
#include "CGAL/convex_hull_2.h"
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
#endif

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX_Real (1.79769e+308)

namespace boundingmesh {

// Returns the unit vector in the direction of the current split
Vector3 Split::directionVector() {
  if (dimension == 0) {
    return Vector3(1, 0, 0);
  } else if (dimension == 1) {
    return Vector3(0, 1, 0);
  } else {
    return Vector3(0, 0, 1);
  }
}

void VoxelSubset::mergeSplit(AppliedSplit split) {
  // Filter redundant splits:
  // Each dimension can have at most 2 splits     ....|->....<-|...
  // AppliedSplit::mergeSplits determines which plane to keep/discard
  bool add = true;
  for (std::vector<AppliedSplit>::iterator it = splits_.begin();
       it != splits_.end();) {
    int result = AppliedSplit::mergeSplitsMerge(*it, split);
    switch (result) {
      case 0:
        ++it;
        break;
      case 1:
        add = false;
        ++it;
        break;
      case 2:
        it = splits_.erase(it);
        break;
    }
  }

  if (add) splits_.push_back(split);
}

int AppliedSplit::mergeSplitsMerge(const AppliedSplit& a,
                                   const AppliedSplit& b) {
  /*
   Determines the merge action for 2 splits
   0 -> keep both
   1 -> keep only a
   2 -> keep only b
   */
  if (a.split.dimension != b.split.dimension || a.direction != b.direction)
    return 0;
  else if (a.direction && b.direction) {
    // both splits are above, take higher
    if (a.split.index > b.split.index)
      return 2;
    else
      return 1;
  } else {
    // both splits are below, take lower
    if (a.split.index > b.split.index)
      return 1;
    else
      return 2;
  }
}

// Merges two voxel subsets together by adding the voxels form both sets to one
// set and recalculating the convex hull
VoxelSubset VoxelSubset::merge(VoxelSubset other) {
  VoxelSubset ret(*this, true);
  for (int i = 0; i < indices_.size(); ++i) {
    ret.addVoxel(indices_[i]);
  }

  for (int i = 0; i < other.indices_.size(); i++) {
    ret.addVoxel(other.indices_[i]);
  }

  for (int i = 0; i < 3; i++) {
    bool hasDirection = false;
    bool hasNotDirection = false;
    for (int j = 0; j < ret.splits_.size(); j++) {
      if (ret.splits_[j].split.dimension == i) {
        if (ret.splits_[j].direction) {
          hasDirection = true;
        } else {
          hasNotDirection = true;
        }
      }
    }
    if (!hasDirection) {
      Split split(i, -10);
      ret.splits_.push_back(AppliedSplit(split, true));
    }
    if (!hasNotDirection) {
      Split split(i, voxels_->resolution(i) + 10);
      ret.splits_.push_back(AppliedSplit(split, false));
    }
  }

  for (int i = 0; i < other.splits_.size(); i++) {
    // splits_.push_back(other.splits_[i]);
    ret.mergeSplit(other.splits_[i]);
  }

  ret.calculateConvexHullVoxel();
  return ret;
}

// Partitions the given voxel subset into two parts according the the provided
// split
// This also calculates the convex hulls for both new parts
std::vector<VoxelSubset> VoxelSubset::partitionVoxel(Split split) {
  return partitionVoxel(split, true);
}

std::vector<VoxelSubset> VoxelSubset::partitionVoxel(Split split,
                                                     bool calculateHulls) {
  // Partition voxels of a set according to a plane
  std::vector<VoxelSubset> subsets;

  subsets.push_back(VoxelSubset(*this, true));
  subsets.push_back(VoxelSubset(*this, true));

  AppliedSplit above(split, true);
  AppliedSplit below(split, false);

  subsets[0].addSplit(above);
  subsets[1].addSplit(below);

  for (int i = 0; i < indices_.size(); ++i) {
    const Voxel& voxel = voxels_->voxel(indices_[i]);
    if (split.test(voxel)) {
      subsets[0].addVoxel(indices_[i]);
    } else {
      subsets[1].addVoxel(indices_[i]);
    }
  }
  // std::cout<<"split "<<split.dimension<<" "<<split.index<<" Values:
  // "<<subsets[0].evaluate()<<" "<<subsets[1].evaluate()<<std::endl;
  if (calculateHulls) {
    subsets[0].calculateConvexHullVoxel();
    subsets[1].calculateConvexHullVoxel();
  }
  return subsets;
}
// Returns a subset of all voxels from the current voxel subset which
// only contains all voxels from the surface of the original mesh
VoxelSubset VoxelSubset::getSurfaceSubset() const {
  VoxelSubset ret = VoxelSubset(*this, true);
  for (int i = 0; i < indices_.size(); ++i) {
    if (voxels_->voxel(indices_[i]).type() == SURFACE) {
      ret.addVoxel(indices_[i]);
    }
  }
  return ret;
}

int VoxelSubset::voxelCount() const { return indices_.size(); }

// Returns the number of voxels which are located at the surface
// of the original mesh in the current voxel subset
int VoxelSubset::surfaceVoxelCount() const {
  int ret = 0;
  for (int i = 0; i < indices_.size(); i++) {
    if (voxels_->voxel(indices_[i]).type() == SURFACE) {
      ret++;
    }
  }
  return ret;
}

// Function calculating the area of the convex hull using
// http://alienryderflex.com/polygon_area/
Real convexHull2DArea(std::vector<Point_2> points) {
  Real area = 0.0;
  int j = points.size() - 1;
  for (int i = 0; i < points.size(); i++) {
    area += (points[j].x() + points[i].x()) * (points[j].y() - points[i].y());
    j = i;
  }

  return area;
}

// Calculates the area of the 2D convex hull of a voxel set not looking
// at the specified direction (first return value)
// Additionally the area of the inner pixels is calculated
void write2DHullWRL(std::vector<Point_2> points, std::vector<Point_2> hull,
                    string filename) {
  std::ofstream file(filename.c_str());
  file << "#VRML V2.0 utf8" << std::endl << std::endl;
  file << "#Created by boundingmesh" << std::endl << std::endl;

  file << "Transform {" << std::endl;
  file << "\t translation 0 0 0" << std::endl;
  // file << "\t scale " << subset.volume() << " " << voxels_->voxelSize()
  //		<< " " << voxels_->voxelSize() << std::endl;
  file << "\t children [ " << std::endl;
  file << "\t\tShape {" << std::endl;
  file << "\t\t\tgeometry PointSet { " << std::endl;
  file << "\t\t\t\tcoord Coordinate { " << std::endl;

  file << "\t\t\t\t\tpoint [" << std::endl;
  for (unsigned int i = 0; i < points.size(); ++i) {
    file << "\t\t\t\t\t\t" << points[i].x() << " " << points[i].y() << " 0,"
         << std::endl;
  }
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t\t}" << std::endl;
  file << "\t\t\t\tcolor Color {" << std::endl;
  file << "\t\t\t\t\tcolor [" << std::endl;
  for (unsigned int i = 0; i < points.size(); ++i) {
    file << "\t\t\t\t\t\t0 0 1," << std::endl;
  }
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t\t}" << std::endl;
  file << "\t\t\t}" << std::endl;
  file << "\t\t}" << std::endl;

  file << "\t\tShape {" << std::endl;
  file << "\t\t\tgeometry IndexedLineSet { " << std::endl;
  file << "\t\t\t\tcoord Coordinate { " << std::endl;

  file << "\t\t\t\t\tpoint [" << std::endl;
  for (unsigned int i = 0; i < hull.size(); ++i) {
    file << "\t\t\t\t\t\t" << hull[i].x() << " " << hull[i].y() << " 0,"
         << std::endl;
  }
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t\t}" << std::endl;
  file << "\t\t\t\tcolor Color {" << std::endl;
  file << "\t\t\t\t\tcolor [" << std::endl;
  for (unsigned int i = 0; i < hull.size(); ++i) {
    file << "\t\t\t\t\t\t1 0 0," << std::endl;
  }
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t\t}" << std::endl;
  file << "\t\t\t\t\tcoordIndex [" << std::endl;
  for (unsigned int i = 0; i < hull.size() - 1; ++i) {
    file << "\t\t\t\t\t\t" << i << ", " << (i + 1) << ", -1," << std::endl;
  }
  file << "\t\t\t\t\t\t" << (hull.size() - 1) << ", 0, -1" << std::endl;
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t\t\tcolorIndex [" << std::endl;
  for (unsigned int i = 0; i < hull.size(); ++i) {
    file << "\t\t\t\t\t\t" << i << ", " << std::endl;
  }
  file << "\t\t\t\t\t]" << std::endl;
  file << "\t\t\t}" << std::endl;
  file << "\t\t}" << std::endl;

  file << "\t]" << std::endl;
  file << "}";
}

std::pair<Real, Real> VoxelSubset::calculate2DConvexHullVolume(int direction) {
  // Determine directions of the convex hull
  int dir1 = (direction + 1) % 3;
  int dir2 = (direction + 2) % 3;

  ComputeBB();
  int len1 = bounding_box_max_[dir1] - bounding_box_min_[dir1] + 1;
  int len2 = bounding_box_max_[dir2] - bounding_box_min_[dir2] + 1;

  // Create and initialize new 2D array to store point positions
  int** points = new int*[len1];
  for (int i = 0; i < len1; i++) {
    points[i] = new int[len2];
    for (int j = 0; j < len2; j++) {
      points[i][j] = 0;
    }
  }

  // Fill the point position array
  for (int i = 0; i < indices_.size(); i++) {
    const Voxel& voxel = voxels_->voxel(indices_[i]);
    int x = voxel.coordinate(dir1) - bounding_box_min_[dir1];
    int y = voxel.coordinate(dir2) - bounding_box_min_[dir2];
    points[x][y] = 1;
  }

  // Calculate the real positions from the point array
  std::vector<Point_2> pointCoordinates;
  Vector3 origin = voxels_->origin();
  for (int i = 0; i < len1; i++) {
    for (int j = 0; j < len2; j++) {
      if (points[i][j] == 1) {
        double d1 = i * voxels_->voxelSize() + origin(dir1);
        double d2 = j * voxels_->voxelSize() + origin(dir1);

        pointCoordinates.push_back(Point_2(d1, d2));
      }
    }
  }
  // Delete unneeded points to prevent memory leaks
  for (int i = 0; i < len1; i++) {
    delete[] points[i];
  }
  delete[] points;

  // Calculate the convex hull and the area of it
  std::vector<Point_2> pointsHull;
  CGAL::convex_hull_2(pointCoordinates.begin(), pointCoordinates.end(),
                      std::back_inserter(pointsHull));
  // write2DHullWRL(pointCoordinates, pointsHull, "hull2d_" +
  // std::to_string(splits_.size()) + "_" + std::to_string(len1) + "_" +
  // std::to_string(len2) + ".wrl");
  return std::make_pair(
      fabs(convexHull2DArea(pointsHull)),
      pointCoordinates.size() * voxels_->voxelSize() * voxels_->voxelSize());
}

// Calculates the convex hull from voxel subset
// This uses one point for each of the 8 corner points of a voxel and
// then simply calls the convex hull algorithm
void VoxelSubset::calculateConvexHullVoxel() {
  std::vector<Vector3> points;
  bool useCenter = false;

  // Generate additional points within the volume
  // Required at intersection of 3 planes, within the volume
  for (int i = 0; i < splits_.size(); ++i)
    for (int j = i + 1; j < splits_.size(); ++j)
      for (int k = j + 1; k < splits_.size(); ++k) {
        if (splits_[i].split.dimension != splits_[j].split.dimension &&
            splits_[i].split.dimension != splits_[k].split.dimension &&
            splits_[j].split.dimension != splits_[k].split.dimension) {
          Vector3 voxel_pos;
          voxel_pos(splits_[i].split.dimension) = splits_[i].split.index;
          voxel_pos(splits_[j].split.dimension) = splits_[j].split.index;
          voxel_pos(splits_[k].split.dimension) = splits_[k].split.index;

          if (voxel_pos(0) < 0 || voxel_pos(0) > voxels_->resolution(0) ||
              voxel_pos(1) < 0 || voxel_pos(1) > voxels_->resolution(1) ||
              voxel_pos(2) < 0 || voxel_pos(2) > voxels_->resolution(2)) {
            continue;
          }

          bool neighbour_filled =
              voxels_->voxelAt(voxel_pos(0), voxel_pos(1), voxel_pos(2)) >= 0;
          if (voxel_pos(2) + 1 < voxels_->resolution(2))
            neighbour_filled = neighbour_filled ||
                               voxels_->voxelAt(voxel_pos(0), voxel_pos(1),
                                                voxel_pos(2) + 1) >= 0;
          if (voxel_pos(1) + 1 < voxels_->resolution(1))
            neighbour_filled = neighbour_filled ||
                               voxels_->voxelAt(voxel_pos(0), voxel_pos(1) + 1,
                                                voxel_pos(2)) >= 0;

          if (voxel_pos(1) + 1 < voxels_->resolution(1) &&
              voxel_pos(2) + 1 < voxels_->resolution(2))
            neighbour_filled = neighbour_filled ||
                               voxels_->voxelAt(voxel_pos(0), voxel_pos(1) + 1,
                                                voxel_pos(2) + 1) >= 0;
          if (voxel_pos(0) + 1 < voxels_->resolution(0))
            neighbour_filled = neighbour_filled ||
                               voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1),
                                                voxel_pos(2)) >= 0;
          if (voxel_pos(0) + 1 < voxels_->resolution(0) &&
              voxel_pos(2) + 1 < voxels_->resolution(2))
            neighbour_filled = neighbour_filled ||
                               voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1),
                                                voxel_pos(2) + 1) >= 0;
          if (voxel_pos(0) + 1 < voxels_->resolution(0) &&
              voxel_pos(1) + 1 < voxels_->resolution(1))
            neighbour_filled =
                neighbour_filled ||
                voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1) + 1,
                                 voxel_pos(2)) >= 0;
          if (voxel_pos(0) + 1 < voxels_->resolution(0) &&
              voxel_pos(1) + 1 < voxels_->resolution(1) &&
              voxel_pos(2) + 1 < voxels_->resolution(2))
            neighbour_filled =
                neighbour_filled ||
                voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1) + 1,
                                 voxel_pos(2) + 1) >= 0;
          // Consider interior if at least one neighbouring voxel is filled
          if (!neighbour_filled) {
            continue;
          }
          Vector3 offset;
          offset(splits_[i].split.dimension) =
              (splits_[i].split.index + 0.5) * voxels_->voxelSize();
          offset(splits_[j].split.dimension) =
              (splits_[j].split.index + 0.5) * voxels_->voxelSize();
          offset(splits_[k].split.dimension) =
              (splits_[k].split.index + 0.5) * voxels_->voxelSize();
          Vector3 new_point = voxels_->origin() + offset;
          points.push_back(new_point);
        }
      }

  Vector3 origin = voxels_->origin();

  if (useCenter) {
    for (int i = 0; i < indices_.size(); i++) {
      const Voxel& voxel = voxels_->voxel(indices_[i]);
      if (voxels_->voxel(indices_[i]).type() == SURFACE) {
        double xp = voxel.x() * voxels_->voxelSize() + origin(0);
        double yp = voxel.y() * voxels_->voxelSize() + origin(1);
        double zp = voxel.z() * voxels_->voxelSize() + origin(2);
        points.push_back(Vector3(xp, yp, zp));
      }
    }
  } else {
    for (int i = 0; i < indices_.size(); i++) {
      const Voxel& voxel = voxels_->voxel(indices_[i]);
      if (voxels_->voxel(indices_[i]).type() == SURFACE) {
        double xp = (voxel.x() + 0.5) * voxels_->voxelSize() + origin(0);
        double yp = (voxel.y() + 0.5) * voxels_->voxelSize() + origin(1);
        double zp = (voxel.z() + 0.5) * voxels_->voxelSize() + origin(2);
        double xm = (voxel.x() - 0.5) * voxels_->voxelSize() + origin(0);
        double ym = (voxel.y() - 0.5) * voxels_->voxelSize() + origin(1);
        double zm = (voxel.z() - 0.5) * voxels_->voxelSize() + origin(2);
        points.push_back(Vector3(xp, yp, zp));
        points.push_back(Vector3(xp, yp, zm));
        points.push_back(Vector3(xp, ym, zp));
        points.push_back(Vector3(xp, ym, zm));
        points.push_back(Vector3(xm, yp, zp));
        points.push_back(Vector3(xm, yp, zm));
        points.push_back(Vector3(xm, ym, zp));
        points.push_back(Vector3(xm, ym, zm));
      }
    }
  }
  // std::cout << "Calculating convex hull with number of points: "
  //		<< points.size() << std::endl;
  if (points.size() > 3)
    convex_hull_ = Convex(points);
  else
    convex_hull_ = Convex();

  // convex_hull_.volume = convex_hull_.ComputeVolume();
}

// Simply method for debugging purposes which outputs a given voxel subset
// to a file in WRL file format
void VoxelSubset::writeWRL(string filename, bool debugTriangles) {
  std::ofstream file(filename.c_str());
  file << "#VRML V2.0 utf8" << std::endl << std::endl;
  file << "#Created by boundingmesh" << std::endl << std::endl;
  file << "Transform {" << std::endl;
  file << "\t translation " << voxels_->origin()(0) << " "
       << voxels_->origin()(1) << " " << voxels_->origin()(2) << std::endl;
  file << "\t scale " << voxels_->voxelSize() << " " << voxels_->voxelSize()
       << " " << voxels_->voxelSize() << std::endl;
  file << "\t children [ " << std::endl;
  for (unsigned int i = 0; i < indices_.size(); ++i) {
    const Voxel& voxel = voxels_->voxel(indices_[i]);
    file << "\t\tTransform {" << std::endl;
    file << "\t\t\t translation " << voxel.x() << " " << voxel.y() << " "
         << voxel.z() << std::endl;
    file << "\t\t\t children [ " << std::endl;

    file << "\t\t\t\tShape {" << std::endl;
    file << "\t\t\t\t\tappearance Appearance {" << std::endl;
    file << "\t\t\t\t\t\tmaterial Material {" << std::endl;
    file << "\t\t\t\t\t\t\ttransparency 0.5" << std::endl;
    if (debugTriangles) {
      if (voxel.nTriangles() == 0)
        file << "\t\t\t\t\t\t\tdiffuseColor 1 0 0" << std::endl;
      else if (voxel.nTriangles() == 1)
        file << "\t\t\t\t\t\t\tdiffuseColor 0 1 0" << std::endl;
      else if (voxel.nTriangles() == 2)
        file << "\t\t\t\t\t\t\tdiffuseColor 0 0 1" << std::endl;
      else if (voxel.nTriangles() == 3)
        file << "\t\t\t\t\t\t\tdiffuseColor 1 1 1" << std::endl;
      else
        file << "\t\t\t\t\t\t\tdiffuseColor 0 0 0" << std::endl;

    } else {
      if (voxel.type() == SURFACE)
        file << "\t\t\t\t\t\t\tdiffuseColor 1 0 0" << std::endl;
      else if (voxel.type() == INNER)
        file << "\t\t\t\t\t\t\tdiffuseColor 0 0 1" << std::endl;
    }
    file << "\t\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t\tgeometry Box {" << std::endl;
    file << "\t\t\t\t\t\tsize 1 1 1" << std::endl;
    file << "\t\t\t\t\t}" << std::endl;
    file << "\t\t\t\t}" << std::endl;
    file << "\t\t\t ]" << std::endl;
    file << "\t\t}";
  }
  file << "\t ]" << std::endl;
  file << "}";
}

SegmenterDownsampling::SegmenterDownsampling()
    : passes_(32),
      maximum_concavity_(0.001),
      initial_volume_(1),
      initial_size_(1),
      voxel_size_(1),
      alpha_(0.05),
      beta_(0.05),
      gamma_(0.0005),
      delta_(0.05),
      convexhull_downsampling_(1),
      plane_downsampling_(4),
      heuristic_(1),
      debug_(false),
      superDebug_(false) {}

void SegmenterDownsampling::setMaximumConcavity(Real maximum_concavity) {
  maximum_concavity_ = maximum_concavity;
}

void SegmenterDownsampling::setMaxPasses(int passes) { passes_ = passes; }

void SegmenterDownsampling::setMesh(std::shared_ptr<Mesh> mesh,
                                    int min_voxel_count) {
  int count = 1;
  double length = 100;
  VoxelSet set;
  while (count < min_voxel_count * 0.9 || count > min_voxel_count * 1.1) {
    set = VoxelSet(mesh, length, true);
    count = set.nVoxels();
    // std::cout << "count: " << count << " length: " << length << std::endl;
    if (count < min_voxel_count * 0.9 || count > min_voxel_count * 1.1) {
      double a = pow((double)(min_voxel_count) / count, 1.0 / 3.0);
      length = length / a;
      // std::cout << "a: " << a << " length: " << length << std::endl;
    }
  }

  voxels_ = std::make_shared<VoxelSet>(mesh, length, true);
}

std::shared_ptr<VoxelSet> SegmenterDownsampling::getVoxels() { return voxels_; }

// Computes and stores the bounding box
void VoxelSubset::ComputeBB() {
  const size_t nVoxels = indices_.size();
  if (nVoxels == 0) return;
  bounding_box_min_[0] = voxels_->voxel(indices_[0]).x();
  bounding_box_max_[0] = voxels_->voxel(indices_[0]).x();
  bounding_box_min_[1] = voxels_->voxel(indices_[0]).y();
  bounding_box_max_[1] = voxels_->voxel(indices_[0]).y();
  bounding_box_min_[2] = voxels_->voxel(indices_[0]).z();
  bounding_box_max_[2] = voxels_->voxel(indices_[0]).z();
  Vector3 bary = Vector3::Zero();
  for (size_t p = 0; p < nVoxels; ++p) {
    const Voxel& voxel = voxels_->voxel(indices_[p]);
    bary[0] += voxel.x();
    bary[1] += voxel.y();
    bary[2] += voxel.z();

    if (bounding_box_min_[0] > voxel.x()) {
      bounding_box_min_[0] = voxel.x();
    }
    if (bounding_box_max_[0] < voxel.x()) {
      bounding_box_max_[0] = voxel.x();
    }
    if (bounding_box_min_[1] > voxel.y()) {
      bounding_box_min_[1] = voxel.y();
    }
    if (bounding_box_max_[1] < voxel.y()) {
      bounding_box_max_[1] = voxel.y();
    }
    if (bounding_box_min_[2] > voxel.z()) {
      bounding_box_min_[2] = voxel.z();
    }
    if (bounding_box_max_[2] < voxel.z()) {
      bounding_box_max_[2] = voxel.z();
    }
  }

  bary /= (double)nVoxels;
  for (int h = 0; h < 3; ++h) {
    barycenter_[h] = (short)(bary[h] + 0.5);
  }
}

void VoxelSubset::ComputePrincipalAxes() {
  const size_t nVoxels = indices_.size();
  if (nVoxels == 0) return;
  barycenterPCA_[0] = barycenterPCA_[1] = barycenterPCA_[2] = 0.0;
  for (size_t v = 0; v < nVoxels; ++v) {
    const Voxel& voxel = voxels_->voxel(indices_[v]);
    barycenterPCA_[0] += voxel.x();
    barycenterPCA_[1] += voxel.y();
    barycenterPCA_[2] += voxel.z();
  }
  barycenterPCA_ /= (double)nVoxels;

  double covMat[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double x, y, z;
  for (size_t v = 0; v < nVoxels; ++v) {
    const Voxel& voxel = voxels_->voxel(indices_[v]);
    x = voxel.x() - barycenter_[0];
    y = voxel.y() - barycenter_[1];
    z = voxel.z() - barycenter_[2];
    covMat[0][0] += x * x;
    covMat[1][1] += y * y;
    covMat[2][2] += z * z;
    covMat[0][1] += x * y;
    covMat[0][2] += x * z;
    covMat[1][2] += y * z;
  }
  covMat[0][0] /= nVoxels;
  covMat[1][1] /= nVoxels;
  covMat[2][2] /= nVoxels;
  covMat[0][1] /= nVoxels;
  covMat[0][2] /= nVoxels;
  covMat[1][2] /= nVoxels;
  covMat[1][0] = covMat[0][1];
  covMat[2][0] = covMat[0][2];
  covMat[2][1] = covMat[1][2];
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covMatEigen(
      &covMat[0][0]);
  Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(covMatEigen);
  // covMatEigen = Q*D*QT
  D_.setZero();
  D_.diagonal().head(svd.nonzeroSingularValues()) = svd.singularValues();
}

Real VoxelSubset::GetEigenValue(int axis) const {
  // std::cout << "Eigenvalue: " << d_[axis][axis] << " " << D_(axis, axis);
  return D_(axis, axis);
}

Vector3 VoxelSubset::GetBoundingBoxMin() const { return bounding_box_min_; }
Vector3 VoxelSubset::GetBoundingBoxMax() const { return bounding_box_max_; }

Real VoxelSubset::GetBoundingBoxSize() const {
  Real a = bounding_box_max_[0] - bounding_box_min_[0];
  Real b = bounding_box_max_[1] - bounding_box_min_[1];
  Real c = bounding_box_max_[2] - bounding_box_min_[2];
  return sqrt(a * a + b * b + c * c);
}

Real SegmenterDownsampling::computePreferredCuttingDirection(VoxelSubset& tset,
                                                             Vector3& dir) {
#if 0
    //FIXME: how should eigenvalues indicate a direction? For that, we would need eigenvectors
	Real ex = tset.GetEigenValue(0);
	Real ey = tset.GetEigenValue(1);
	Real ez = tset.GetEigenValue(2);
	Real vx = (ey - ez) * (ey - ez);
	Real vy = (ex - ez) * (ex - ez);
	Real vz = (ex - ey) * (ex - ey);
	if (vx < vy && vx < vz) {
		Real e = ey * ey + ez * ez;
		dir[0] = 1.0;
		dir[1] = 0.0;
		dir[2] = 0.0;
		return (e == 0.0) ? 0.0 : 1.0 - vx / e;
	} else if (vy < vx && vy < vz) {
		Real e = ex * ex + ez * ez;
		dir[0] = 0.0;
		dir[1] = 1.0;
		dir[2] = 0.0;
		return (e == 0.0) ? 0.0 : 1.0 - vy / e;
	} else {
		Real e = ex * ex + ey * ey;
		dir[0] = 0.0;
		dir[1] = 0.0;
		dir[2] = 1.0;
		return (e == 0.0) ? 0.0 : 1.0 - vz / e;
	}
#else
  dir[0] = 1.0;
  dir[1] = 0.0;
  dir[2] = 0.0;
  return 0;
#endif
}

inline Real computeLocalConcavity(const Real volume,
                                  const Real volumeConvexHull) {
  return fabs(volumeConvexHull - volume) / volumeConvexHull;
}
inline Real computeConcavity(Real volume, Real volumeConvexHull, Real volume0) {
  return fabs(volumeConvexHull - volume) / volume0;
}

Real SegmenterDownsampling::rateSplit(VoxelSubset& inputSet, Split split,
                                      const Vector3& preferredCuttingDirection,
                                      const Real w, const Real alpha,
                                      const Real beta, const Real delta) {
  VoxelSubset onSurfacePSet = inputSet.getSurfaceSubset();

  VoxelSubset left(0);
  VoxelSubset right(0);
  // compute convex-hulls

  std::vector<VoxelSubset> surfaceParts = onSurfacePSet.partitionVoxel(split);
  left = surfaceParts[0];
  right = surfaceParts[1];
  if (superDebug_) {
    surfaceParts[0].writeWRL("a0_voxel.wrl");
    surfaceParts[1].writeWRL("a1_voxel.wrl");
    left.getConvexHull()->writeWrl("b0_voxel.wrl");
    right.getConvexHull()->writeWrl("b1_voxel.wrl");
  }

  Real volumeLeftCH = left.convexVolume();
  Real volumeRightCH = right.convexVolume();

  // compute clipped volumes
  std::vector<VoxelSubset> parts = inputSet.partitionVoxel(split, false);
  Real volumeLeft = parts[0].volume();
  Real volumeRight = parts[1].volume();
  Real concavityLeft =
      computeConcavity(volumeLeft, volumeLeftCH, initial_volume_);
  Real concavityRight =
      computeConcavity(volumeRight, volumeRightCH, initial_volume_);
  Real localConcaviotyLeft = computeLocalConcavity(volumeLeft, volumeLeftCH);
  Real localConcavityRight = computeLocalConcavity(volumeRight, volumeRightCH);
  Real concavity = (concavityLeft + concavityRight);
  Real localConcavity = delta * (localConcaviotyLeft + localConcavityRight);

  // compute cost
  Real balance =
      alpha * pow(pow(volumeLeft - volumeRight, 2.0), 0.5) / initial_volume_;
  Real symmetry =
      beta * w * preferredCuttingDirection.dot(split.directionVector());
  Real total = concavity + balance + symmetry + localConcavity;
  if (debug_) {
    std::cout << split.dimension << " " << split.index << " " << volumeLeft
              << " " << volumeRight << " " << total << " " << concavity << " "
              << balance << " " << symmetry << " " << localConcavity
              << std::endl;
  }

  return total;
}

Real SegmenterDownsampling::rateSplit3(VoxelSubset& inputSet, Split split,
                                       const Vector3& preferredCuttingDirection,
                                       const Real w, const Real alpha,
                                       const Real beta, const Real delta) {
  int direction1 = (split.dimension + 1) % 3;
  int direction2 = (split.dimension + 2) % 3;

  VoxelSubset onSurfacePSet = inputSet.getSurfaceSubset();
  std::vector<VoxelSubset> parts = onSurfacePSet.partitionVoxel(split, false);
  std::pair<Real, Real> part00 =
      parts[0].calculate2DConvexHullVolume(direction1);
  std::pair<Real, Real> part01 =
      parts[1].calculate2DConvexHullVolume(direction1);
  std::pair<Real, Real> part10 =
      parts[0].calculate2DConvexHullVolume(direction2);
  std::pair<Real, Real> part11 =
      parts[1].calculate2DConvexHullVolume(direction2);

  Real volumeLeftCH0 = part00.first;
  Real volumeRightCH0 = part01.first;
  Real volumeLeft0 = part00.second;
  Real volumeRight0 = part01.second;
  Real volumeLeftCH1 = part10.first;
  Real volumeRightCH1 = part11.first;
  Real volumeLeft1 = part10.second;
  Real volumeRight1 = part11.second;

  Real area0 = initial_area_[direction1];
  Real concavityLeft0 = computeConcavity(volumeLeft0, volumeLeftCH0, area0);
  Real concavityRight0 = computeConcavity(volumeRight0, volumeRightCH0, area0);
  Real localConcavityLeft0 = computeLocalConcavity(volumeLeft0, volumeLeftCH0);
  Real localConcavityRight0 =
      computeLocalConcavity(volumeRight0, volumeRightCH0);

  Real area1 = initial_area_[direction2];
  Real concavityLeft1 = computeConcavity(volumeLeft1, volumeLeftCH1, area1);
  Real concavityRight1 = computeConcavity(volumeRight1, volumeRightCH1, area1);
  Real localConcavityLeft1 = computeLocalConcavity(volumeLeft1, volumeLeftCH1);
  Real localConcavityRight1 =
      computeLocalConcavity(volumeRight1, volumeRightCH1);

  Real concavity =
      MAX(concavityLeft0 + concavityRight0, concavityLeft1 + concavityRight1);
  // Real concavity = MAX(concavityLeft0 + concavityRight0, concavityLeft1 +
  // concavityRight1);
  Real localConcavity = delta * MAX(localConcavityLeft0 + localConcavityRight0,
                                    localConcavityLeft1 + localConcavityRight1);

  // compute cost
  Real balance = alpha *
                 pow(pow(parts[0].volume() - parts[1].volume(), 2.0), 0.5) /
                 initial_volume_;
  Real symmetry =
      beta * w * preferredCuttingDirection.dot(split.directionVector());
  Real total = concavity + balance + symmetry + localConcavity;
  if (debug_) {
    std::cout << split.dimension << " " << split.index << " " << total << " "
              << concavity << " " << balance << " " << symmetry << " "
              << localConcavity << std::endl;
    std::cout << "\t" << volumeLeft0 << "\t" << volumeLeftCH0 << "\t\t"
              << volumeRight0 << "\t" << volumeRightCH0 << std::endl;
    std::cout << "\t" << volumeLeft1 << "\t" << volumeLeftCH1 << "\t\t"
              << volumeRight1 << "\t" << volumeRightCH1 << std::endl;
  }

  return total;
}

void SegmenterDownsampling::computeBestClippingPlane(
    const VoxelSubset& inputPSet2, const Real volume,
    const std::vector<Split>& splits, const Vector3& preferredCuttingDirection,
    const Real w, const Real alpha, const Real beta, const Real delta,
    const int convexhullDownsampling, Split& bestSplit) {
  VoxelSubset inputPSet = inputPSet2;

  int iBest = -1;
  Real minTotal = MAX_Real;

  for (int x = 0; x < splits.size(); ++x) {
    Split split = splits[x];

    Real total = 0;
    if (heuristic_ == 1) {
      total = rateSplit(inputPSet, split, preferredCuttingDirection, w, alpha,
                        beta, delta);
    } else if (heuristic_ == 2) {
      std::cout << "Heuristic2 not compiled." << std::endl;
      throw std::invalid_argument("Heuristic2 not compiled");
    } else if (heuristic_ == 3) {
      total = rateSplit3(inputPSet, split, preferredCuttingDirection, w, alpha,
                         beta, delta);
    } else {
      std::cout << "Unknown heuristc: " << heuristic_ << std::endl;
    }

    if (total < minTotal || (total == minTotal && x < iBest)) {
      bestSplit = split;
      minTotal = total;
      iBest = x;
    }
  }
}

void SegmenterDownsampling::printDebugSlices(VoxelSubset whole_mesh) {
  for (int i = 0; i < voxels_->resolution(0); i++) {
    std::vector<VoxelSubset> parts = whole_mesh.partitionVoxel(Split(0, i));
    parts[0].writeWRL("part" + std::to_string(static_cast<long long>(i)) +
                      "_voxel.wrl");
  }
}

void SegmenterDownsampling::compute() {
  std::vector<VoxelSubset> parts;
  std::vector<VoxelSubset> inputParts;
  std::vector<VoxelSubset> temp;

  VoxelSubset whole_mesh(voxels_);
  for (int i = 0; i < voxels_->nVoxels(); ++i) {
    whole_mesh.addVoxel(i);
  }
  whole_mesh.calculateConvexHullVoxel();
  inputParts.push_back(whole_mesh);
  // printDebugSlices(whole_mesh);

  for (int pass = 0; pass < passes_ && inputParts.size() > 0; pass++) {
    std::cout << "Pass " << pass << " Number of parts: " << inputParts.size()
              << std::endl;
    if (debug_) {
      std::vector<std::shared_ptr<Mesh>> meshes;
      for (unsigned int i = 0; i < inputParts.size(); ++i) {
        meshes.push_back(inputParts[i].getConvexHull());
      }
      for (unsigned int i = 0; i < parts.size(); ++i) {
        meshes.push_back(parts[i].getConvexHull());
      }
      boundingmesh::Mesh::writeMultimeshWrl(
          meshes,
          "step" + std::to_string(static_cast<long long>(pass)) + ".wrl", true);
    }
    for (int partIndex = 0; partIndex < inputParts.size(); partIndex++) {
      VoxelSubset currentPart = inputParts[partIndex];
      Real volume = currentPart.volume();
      currentPart.ComputeBB();
      currentPart.ComputePrincipalAxes();
      if (pass == 1 && partIndex == 0) {
        // superDebug_ = true;
      }
      if (debug_) {
        currentPart.getConvexHull()->writeWrl(
            "step" + std::to_string(static_cast<long long>(pass)) + "_" +
            std::to_string(static_cast<long long>(partIndex)) + ".wrl");
        inputParts[partIndex].writeWRL(
            "step" + std::to_string(static_cast<long long>(pass)) + "_" +
                std::to_string(static_cast<long long>(partIndex)) +
                "_voxel.wrl",
            true);
      }
      Real volumeConvexHull = fabs(currentPart.convexVolume());

      if (pass == 0) {
        initial_volume_ = volumeConvexHull;
        initial_size_ = currentPart.GetBoundingBoxSize();
        initial_surface_voxels_ = currentPart.getSurfaceSubset().voxelCount();
        for (int i = 0; i < 3; i++) {
          initial_area_[i] = currentPart.calculate2DConvexHullVolume(i).first;
        }
      }

      Real concavity =
          computeConcavity(volume, volumeConvexHull, initial_volume_);
      Real error =
          (1.01 * currentPart.surfaceVoxelCount() * voxels_->voxelSize() *
           voxels_->voxelSize() * voxels_->voxelSize()) /
          initial_volume_;
      Real localConcavity =
          (volumeConvexHull > 0.0)
              ? computeLocalConcavity(volume, volumeConvexHull)
              : 0.0;
      if (debug_) {
        std::cout << "volume: " << volume
                  << " convexVolume: " << volumeConvexHull
                  << " concavity: " << concavity << " error: " << error
                  << " localConcavity: " << localConcavity << std::endl;
      }

      if (concavity > maximum_concavity_ && concavity > error) {
        Vector3 preferredCuttingDirection;
        Real w = computePreferredCuttingDirection(currentPart,
                                                  preferredCuttingDirection);

        std::vector<Split> possibleSplits =
            generateSplits(currentPart, plane_downsampling_);

        Split bestSplit;

        computeBestClippingPlane(
            currentPart, volume, possibleSplits, preferredCuttingDirection, w,
            concavity * alpha_, concavity * beta_, concavity * delta_,
            convexhull_downsampling_, bestSplit);

        if (plane_downsampling_ > 1) {
          possibleSplits =
              refineSplits(currentPart, bestSplit, plane_downsampling_);

          computeBestClippingPlane(currentPart, volume, possibleSplits,
                                   preferredCuttingDirection, w,
                                   concavity * alpha_, concavity * beta_,
                                   concavity * delta_, 1, bestSplit);
        }

        std::vector<VoxelSubset> parts = currentPart.partitionVoxel(bestSplit);
        temp.insert(temp.end(), parts.begin(), parts.end());
      } else {
        parts.push_back(currentPart);
      }
    }
    inputParts = temp;
    temp.clear();
  }
  parts.insert(parts.end(), inputParts.begin(), inputParts.end());

  std::vector<std::shared_ptr<Mesh>> meshes;
  std::vector<std::shared_ptr<Mesh>> meshes2;
  for (unsigned int i = 0; i < parts.size(); ++i) {
    if (debug_) {
      meshes2.push_back(parts[i].getConvexHull());
    }
  }
  if (debug_) {
    boundingmesh::Mesh::writeMultimeshWrl(
        meshes2, "stepfinal_beforemerge_voxelhull.wrl", true);
  }
  std::cout << "Merging convex hulls..." << std::endl;
  // parts = mergeConvexHulls(parts);
  for (long long i = 0; i < parts.size(); ++i) {
    if (debug_) {
      meshes.push_back(parts[i].getConvexHull());
    }
    parts[i].calculateConvexHull();
    if (debug_) {
      parts[i].getConvexHull()->writeWrl(
          "stepfinal_part" + std::to_string(static_cast<long long>(i)) +
          ".wrl");
    }
  }
  if (debug_) {
    boundingmesh::Mesh::writeMultimeshWrl(
        meshes, "stepfinal_aftermerge_voxelhull.wrl", true);
  }
  subsets_ = parts;
  for (unsigned int i = 0; i < subsets_.size(); ++i) {
    segmentation_.push_back(subsets_[i].getConvexHull());
  }
}

// Determines whether the bounding boxes of two voxel subsets are adjacent
// to each other or not
bool SegmenterDownsampling::arePartsAdjacent(VoxelSubset& p1, VoxelSubset& p2) {
  double maxDelta = 1;
  Vector3 p1min = p1.GetBoundingBoxMin();
  Vector3 p1max = p1.GetBoundingBoxMax();
  Vector3 p2min = p2.GetBoundingBoxMin();
  Vector3 p2max = p2.GetBoundingBoxMax();

  for (int i = 0; i < 3; i++) {
    if (p1max[i] + maxDelta < p2min[i] || p1min[i] - maxDelta > p2max[i]) {
      return false;
    }
  }

  return true;
}

// Merges adjacent convex hulls so that the error does not increase above the
// specified threshhold and
// the total number of parts decreases at the same time
std::vector<VoxelSubset> SegmenterDownsampling::mergeConvexHulls(
    std::vector<VoxelSubset> parts) {
  long long iteration = 0;
  if (parts.size() > 1) {
    bool iterate = true;

    for (int i = 0; i < parts.size(); i++) {
      parts[i].ComputeBB();
    }

    while (iterate) {
      size_t bestp1;
      size_t bestp2;
      double bestCost = MAX_Real;  // initial_volume_;
      for (size_t p1 = 0; p1 < parts.size() - 1; ++p1) {
        double volume1 = parts[p1].convexVolume();
        for (size_t p2 = p1 + 1; p2 < parts.size() - 1; ++p2) {
          double volume2 = parts[p2].convexVolume();

          if (!arePartsAdjacent(parts[p1], parts[p2])) {
            continue;
          }

          VoxelSubset merged = parts[p1].merge(parts[p2]);

          double combinedVolumeCH = merged.convexVolume();
          double combinedVolume = volume1 + volume2;
          double cost = computeConcavity(combinedVolume, combinedVolumeCH,
                                         initial_volume_);

          if (cost < bestCost) {
            bestCost = cost;
            bestp1 = p1;
            bestp2 = p2;
          }
        }
      }
      if (debug_) {
        std::cout << " best cost: " << bestCost << " gamma: " << gamma_
                  << std::endl;
      }
      if (bestCost < gamma_) {
        if (debug_) {
          parts[bestp1].getConvexHull()->writeWrl(
              "mergedpart1_" + std::to_string(iteration) + ".wrl");
          parts[bestp2].getConvexHull()->writeWrl(
              "mergedpart2_" + std::to_string(iteration) + ".wrl");
        }
        VoxelSubset merged = parts[bestp1].merge(parts[bestp2]);
        parts.erase(parts.begin() + bestp1);
        parts.erase(parts.begin() + bestp2 - 1);
        parts.push_back(merged);
        std::cout << "merged " << bestp1 << " with " << bestp2
                  << "part1 volume: " << parts[bestp1].convexVolume()
                  << " part2 volume: " << parts[bestp2].convexVolume()
                  << " combined volume: " << merged.convexVolume() << std::endl;
        iterate = true;
        if (debug_) {
          merged.getConvexHull()->writeWrl("mergedpart" +
                                           std::to_string(iteration) + ".wrl");
          std::vector<std::shared_ptr<Mesh>> meshes;
          for (unsigned int i = 0; i < parts.size(); ++i) {
            if (debug_) {
              meshes.push_back(parts[i].getConvexHull());
            }
          }
          boundingmesh::Mesh::writeMultimeshWrl(
              meshes, "merge" + std::to_string(iteration) + ".wrl", true);
        }
      } else {
        iterate = false;
      }
      iteration++;
    }
  }

  return parts;
}

std::vector<std::shared_ptr<Mesh>> SegmenterDownsampling::getSegmentation() {
  return segmentation_;
}

// Generates a lit of all possible splits considering only every nth split
// where n is the downsampling rate
std::vector<Split> SegmenterDownsampling::generateSplits(
    const VoxelSubset& subset, const int downsampling) {
  std::vector<Split> possible_splits;
  Vector3 min = subset.GetBoundingBoxMin();
  Vector3 max = subset.GetBoundingBoxMax();
  for (int dimension = 0; dimension < 3; ++dimension)
    for (int index = min(dimension) + 1; index < max(dimension);
         index += downsampling) {
      possible_splits.push_back(Split(dimension, index));
    }

  return possible_splits;
}

// Generates all splits around the split that has previously been declared as
// best split with downsampling
std::vector<Split> SegmenterDownsampling::refineSplits(
    const VoxelSubset& subset, const Split& bestSplit, const int downsampling) {
  std::vector<Split> possible_splits;
  int dimension = bestSplit.dimension;
  int start = MAX(subset.GetBoundingBoxMin()(dimension),
                  bestSplit.index - downsampling);
  int end = MIN(subset.GetBoundingBoxMax()(dimension) - 1,
                bestSplit.index + downsampling);
  for (int index = start; index <= end; index += 1) {
    possible_splits.push_back(Split(dimension, index));
  }

  return possible_splits;
}
}
