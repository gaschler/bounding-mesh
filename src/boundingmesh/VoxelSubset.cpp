//	Copyright (c) 2015, Andre Gaschler, Quirin Fischer
//	All rights reserved.
//
//	Redistribution and use in source and binary forms, with or without modification,
//	are permitted provided that the following conditions are met:
//
//	* Redistributions of source code must retain the above copyright notice, this
//	  list of conditions and the following disclaimer.
//
//	* Redistributions in binary form must reproduce the above copyright notice, this
//	  list of conditions and the following disclaimer in the documentation and/or
//	  other materials provided with the distribution.
//
//	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//	ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "VoxelSubset.h"

namespace boundingmesh
{
    VoxelSubset::VoxelSubset(std::shared_ptr<VoxelSet> voxels) : voxels_(voxels), final_(false)
    {
    }

    VoxelSubset::VoxelSubset(const VoxelSubset &subset, bool empty) : voxels_(subset.voxels_), splits_(subset.splits_),
                                                                      convex_hull_(subset.convex_hull_),
                                                                      final_(subset.final_)
    {
        if (!empty)
            indices_ = std::vector<Index>(subset.indices_);
        else
            indices_ = std::vector<Index>();
    }

    VoxelSubset::~VoxelSubset()
    {
    }

    VoxelSubset &VoxelSubset::operator=(VoxelSubset other)
    {
        swap(*this, other);
        return *this;
    }

    void swap(VoxelSubset &first, VoxelSubset &second)
    {
        first.voxels_.swap(second.voxels_);
        first.indices_.swap(second.indices_);
        first.splits_.swap(second.splits_);
        std::swap(first.convex_hull_, second.convex_hull_);
        std::swap(first.final_, second.final_);
    }

    void VoxelSubset::addVoxel(Index index)
    {
        indices_.push_back(index);
    }


    void VoxelSubset::addSplit(AppliedSplit split)
    {
        //Filter redundant splits:
        //Each dimension can have at most 2 splits     ....|->....<-|...
        //AppliedSplit::mergeSplits determines which plane to keep/discard
        bool add = true;
        for (std::vector<AppliedSplit>::iterator it = splits_.begin(); it != splits_.end();)
        {
            int result = AppliedSplit::mergeSplits(*it, split);
            switch (result)
            {
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

        if (add)
            splits_.push_back(split);
    }

    std::vector<VoxelSubset> VoxelSubset::partition(Split split)
    {
        //Partition voxels of a set according to a plane
        std::vector<VoxelSubset> subsets;

        subsets.push_back(VoxelSubset(*this, true));
        subsets.push_back(VoxelSubset(*this, true));

        AppliedSplit above(split, true);
        AppliedSplit below(split, false);

        subsets[0].addSplit(above);
        subsets[1].addSplit(below);

        for (int i = 0; i < indices_.size(); ++i)
        {
            const Voxel &voxel = voxels_->voxel(indices_[i]);
            if (split.test(voxel))
            {
                subsets[0].addVoxel(indices_[i]);
            }
            else
            {
                subsets[1].addVoxel(indices_[i]);
            }
        }
        //std::cout<<"split "<<split.dimension<<" "<<split.index<<" Values: "<<subsets[0].evaluate()<<" "<<subsets[1].evaluate()<<std::endl;
        subsets[0].calculateConvexHull();
        subsets[1].calculateConvexHull();
        return subsets;
    }

    Real VoxelSubset::convexVolume() const
    {
        return convex_hull_.volume;
    }

    Real VoxelSubset::volume() const
    {
        return indices_.size() * voxels_->voxelSize() * voxels_->voxelSize() * voxels_->voxelSize();
    }

    std::vector<Index> const &VoxelSubset::getIndices() const
    {
        return indices_;
    }

    std::shared_ptr<VoxelSet> const &VoxelSubset::getVoxels() const
    {
        return voxels_;
    }

    template<typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }


    void VoxelSubset::calculateConvexHull()
    {
        std::vector<Vector3> points;

        //Generate additional points within the volume
        //Required at intersection of 3 planes, within the volume
        for (int i = 0; i < splits_.size(); ++i)
            for (int j = i + 1; j < splits_.size(); ++j)
                for (int k = j + 1; k < splits_.size(); ++k)
                {
                    if (splits_[i].split.dimension != splits_[j].split.dimension &&
                        splits_[i].split.dimension != splits_[k].split.dimension &&
                        splits_[j].split.dimension != splits_[k].split.dimension)
                    {
                        Vector3 voxel_pos;
                        voxel_pos(splits_[i].split.dimension) = splits_[i].split.index;
                        voxel_pos(splits_[j].split.dimension) = splits_[j].split.index;
                        voxel_pos(splits_[k].split.dimension) = splits_[k].split.index;
                        if (voxel_pos(0) < 0 || voxel_pos(0) > voxels_->resolution(0) ||
                            voxel_pos(1) < 0 || voxel_pos(1) > voxels_->resolution(1) ||
                            voxel_pos(2) < 0 || voxel_pos(2) > voxels_->resolution(2))
                        {
                            continue;
                        }

                        bool neighbour_filled = voxels_->voxelAt(voxel_pos(0), voxel_pos(1), voxel_pos(2)) >= 0;
                        if (voxel_pos(2) + 1 < voxels_->resolution(2))
                            neighbour_filled =
                                neighbour_filled || voxels_->voxelAt(voxel_pos(0), voxel_pos(1), voxel_pos(2) + 1) >= 0;
                        if (voxel_pos(1) + 1 < voxels_->resolution(1))
                            neighbour_filled =
                                neighbour_filled || voxels_->voxelAt(voxel_pos(0), voxel_pos(1) + 1, voxel_pos(2)) >= 0;

                        if (voxel_pos(1) + 1 < voxels_->resolution(1) && voxel_pos(2) + 1 < voxels_->resolution(2))
                            neighbour_filled = neighbour_filled ||
                                               voxels_->voxelAt(voxel_pos(0), voxel_pos(1) + 1, voxel_pos(2) + 1) >= 0;
                        if (voxel_pos(0) + 1 < voxels_->resolution(0))
                            neighbour_filled =
                                neighbour_filled || voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1), voxel_pos(2)) >= 0;
                        if (voxel_pos(0) + 1 < voxels_->resolution(0) && voxel_pos(2) + 1 < voxels_->resolution(2))
                            neighbour_filled = neighbour_filled ||
                                               voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1), voxel_pos(2) + 1) >= 0;
                        if (voxel_pos(0) + 1 < voxels_->resolution(0) && voxel_pos(1) + 1 < voxels_->resolution(1))
                            neighbour_filled = neighbour_filled ||
                                               voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1) + 1, voxel_pos(2)) >= 0;
                        if (voxel_pos(0) + 1 < voxels_->resolution(0) && voxel_pos(1) + 1 < voxels_->resolution(1) &&
                            voxel_pos(2) + 1 < voxels_->resolution(2))
                            neighbour_filled = neighbour_filled ||
                                               voxels_->voxelAt(voxel_pos(0) + 1, voxel_pos(1) + 1, voxel_pos(2) + 1) >=
                                               0;
                        //Consider interior if at least one neighbouring voxel is filled
                        if (!neighbour_filled)
                        {
                            continue;
                        }
                        Vector3 offset;
                        offset(splits_[i].split.dimension) = (splits_[i].split.index + 0.5) * voxels_->voxelSize();
                        offset(splits_[j].split.dimension) = (splits_[j].split.index + 0.5) * voxels_->voxelSize();
                        offset(splits_[k].split.dimension) = (splits_[k].split.index + 0.5) * voxels_->voxelSize();
                        Vector3 new_point = voxels_->origin() + offset;
                        points.push_back(new_point);
                    }
                }
        std::set<Index> vertices;
        Real epsilon = 0.00001;
        //Collect points to generate convex hull
        //Vertices of triangles on the surface
        //Clip triangles against the split planes to remove overlap
        //Points generated by clipping are directly added, vertices are added collectively by index to remove repetition
        for (unsigned int i = 0; i < indices_.size(); ++i)
        {
            const Voxel &voxel = voxels_->voxel(indices_[i]);
            for (unsigned int j = 0; j < voxel.nTriangles(); ++j)
            {
                const Triangle &triangle = voxels_->mesh()->triangle(voxel.triangle(j));
                std::vector<Vector3> clipped_points;
                std::vector<int> vertex_counter;
                for (unsigned int k = 0; k < 3; ++k)
                {
                    clipped_points.push_back(voxels_->mesh()->vertex(triangle.vertex(k)).position());
                    vertex_counter.push_back(triangle.vertex(k));
                }
                for (unsigned int k = 0; k < splits_.size(); ++k)
                {
                    Plane plane = splits_[k].getPlane(*voxels_);
                    for (unsigned int l = 0; l < clipped_points.size(); ++l)
                    {
                        Vector3 current = clipped_points[l];
                        Real distance = plane.distance(current);
                        if (distance < -epsilon)
                        {
                            Vector3 direction_forward = clipped_points[(l + 1) % clipped_points.size()] - current;
                            Vector3 direction_backward =
                                clipped_points[(l + clipped_points.size() - 1) % clipped_points.size()] - current;
                            clipped_points.erase(clipped_points.begin() + l);
                            vertex_counter.erase(vertex_counter.begin() + l);
                            int inserted = 0;
                            if (direction_backward.dot(plane.normal) > epsilon)
                            {
                                Real lambda_backward =
                                    (-current.dot(plane.normal) - plane.d) / direction_backward.dot(plane.normal);
                                if (lambda_backward < 1 + epsilon && lambda_backward > 0)
                                {
                                    Vector3 backward_pos = current + lambda_backward * direction_backward;

                                    clipped_points.insert(clipped_points.begin() + l, backward_pos);
                                    vertex_counter.insert(vertex_counter.begin() + l, -1);
                                    inserted++;
                                }
                            }

                            if (direction_forward.dot(plane.normal) > epsilon)
                            {
                                Real lambda_forward =
                                    (-current.dot(plane.normal) - plane.d) / direction_forward.dot(plane.normal);
                                if (lambda_forward < 1 + epsilon && lambda_forward > 0)
                                {
                                    Vector3 forward_pos = current + lambda_forward * direction_forward;

                                    clipped_points.insert(clipped_points.begin() + l + inserted, forward_pos);
                                    vertex_counter.insert(vertex_counter.begin() + l + inserted, -1);
                                    inserted++;
                                }
                            }

                            l += inserted - 1;
                        }
                    }
                }
                for (unsigned int k = 0; k < vertex_counter.size(); ++k)
                {
                    if (vertex_counter[k] < 0)
                    {
                        points.push_back(clipped_points[k]);
                    }
                    else
                    {
                        vertices.insert(vertex_counter[k]);
                    }
                }
            }
        }
        for (std::set<Index>::iterator it = vertices.begin(); it != vertices.end(); ++it)
        {
            points.push_back(voxels_->mesh()->vertex(*it).position());
        }

        if (points.size() > 3)
            convex_hull_ = Convex(points);
        else
            convex_hull_ = Convex();

        //Add split planes to hull for debugging
#if 0
		for(unsigned int k = 0; k < splits_.size(); ++k)
		{
			Vector3 normal = Vector3::Zero();
			normal(splits_[k].split.dimension) = 1;
			Vector3 A = voxels_->origin() + ( ((Real)splits_[k].split.index+0.5) * voxels_->voxelSize()) * normal;
			Vector3 direction1 = Vector3::Zero();
			direction1((splits_[k].split.dimension+1)%3) = voxels_->voxelSize()*voxels_->resolution((splits_[k].split.dimension+1)%3);
			Vector3 direction2 = Vector3::Zero();
			direction2((splits_[k].split.dimension+2)%3) = voxels_->voxelSize()*voxels_->resolution((splits_[k].split.dimension+2)%3);
			Index iA = result->addVertex(A);
			Index iB = result->addVertex(A+direction1);
			Index iC = result->addVertex(A+direction2);
			Index iD = result->addVertex(A+direction1+direction2);
			if(splits_[k].direction)
			{
				result->addTriangle(iA, iB, iD);
				result->addTriangle(iA, iD, iC);
			}
			else
			{
				result->addTriangle(iA, iD, iB);
				result->addTriangle(iA, iC, iD);
			}
		}
#endif
    }


    std::shared_ptr<Mesh> VoxelSubset::getConvexHull()
    {
        if (!convex_hull_.mesh)
        {
            return std::make_shared<Mesh>();
        }
        else
        {
            return convex_hull_.mesh;
        }
    }

    bool VoxelSubset::isFinal()
    {
        return final_;
    }

    void VoxelSubset::setFinal()
    {
        final_ = true;
    }
}