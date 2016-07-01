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


#include "Split.h"

#include <iostream>
#include <math.h>

namespace boundingmesh
{
    Split::Split() : dimension(0), index(0)
    {
        //Splits along given dimension between index and index+1
    }

    Split::Split(int dimension, int index) : dimension(dimension), index(index)
    {
    }

    Split::~Split()
    {
    }

    bool Split::test(const Voxel &voxel)
    {
        return voxel.coordinate(dimension) > index;
    }

    AppliedSplit::AppliedSplit(Split &split, bool direction) : split(split), direction(direction)
    {
        /*
        direction:
            true -> all voxels above the split plane
            false -> all voxels below the split plane
        */
    }

    Plane AppliedSplit::getPlane(const VoxelSet &voxel_set)
    {
        //Return plane of the split
        //Is oriented so all included voxels lie on the front (positive distance)
        Vector3 normal = Vector3::Zero();
        normal(split.dimension) = 1;
        Vector3 point = voxel_set.origin() + (((Real) split.index + 0.5) * voxel_set.voxelSize()) * normal;
        if (!direction)
            normal *= -1;
        return Plane(normal, point);
    }

    int AppliedSplit::mergeSplits(const AppliedSplit &a, const AppliedSplit &b)
    {
        /*
            Determines the merge action for 2 splits
            0 -> keep both
            1 -> keep only a
            2 -> keep only b
        */
        if (a.split.dimension != b.split.dimension || a.direction != b.direction)
            return 0;
        else if (a.direction && b.direction)
        {
            //both splits are above, take higher
            if (a.split.index > b.split.index)
                return 1;
            else
                return 2;
        }
        else
        {
            //both splits are below, take lower
            if (a.split.index > b.split.index)
                return 2;
            else
                return 1;
        }
    }
}
