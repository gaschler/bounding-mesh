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

#ifndef BOUNDINGMESH_SEGMENTER_SIMPLE_H
#define BOUNDINGMESH_SEGMENTER_SIMPLE_H

#include "Mesh.h"
#include "Segmenter.h"
#include "Split.h"
#include "VoxelSet.h"
#include "VoxelSubset.h"

namespace boundingmesh
{
	class SegmenterSimple : public Segmenter
	{
	public:
		SegmenterSimple();

		void setMaxPasses(int passes);
		void setMinVolumeGain(Real min_gain);

		void setMeshWithVoxelSize(std::shared_ptr<Mesh> mesh, Real voxel_size);
		void setMesh(std::shared_ptr<Mesh> mesh, int min_voxel_count);
		void compute();

		std::vector< std::shared_ptr<Mesh> > getSegmentation();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
		int passes_;
		Real min_gain_;

		Real evaluation_blend_;
		Real voxel_size_;

		std::shared_ptr<VoxelSet> voxels_;
		std::vector< VoxelSubset >  subsets_;
		std::vector< std::shared_ptr<Mesh> >  segmentation_;

		std::vector<Split> generateSplits(const VoxelSubset& subset);

		Real evaluatePartition(const VoxelSubset& previous_set, const std::vector<VoxelSubset>& partition);
		Real evaluateSubset(const VoxelSubset& subset);
	};

}

#endif //BOUNDINGMESH_SEGMENTER_SIMPLE_H
