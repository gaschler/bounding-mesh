//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
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

#ifndef BOUNDINGMESH_DECIMATOR_H
#define BOUNDINGMESH_DECIMATOR_H

#include "Mesh.h"
#include "MetricGenerator.h"
#include "ContractionUtils.h"
#include <Eigen/StdVector>

#include <deque>
#include <memory>

namespace boundingmesh
{	
	enum DecimationDirection{
		Outward,
		Inward,
		Any
	};

	const unsigned int default_target_vertices = 1000;
	const Real default_maximum_error = 1.0;

	const Real pi = 3.141592653589793238462;
	const Real phi = (2 * pi) * (30 / 360);


	typedef void (*ComputeCallback)(unsigned int, Real);  

	class Decimator
	{
	public:
		Decimator(DecimationDirection direction = Outward);
		~Decimator();

		Real currentError();
		Real nextError();

		void setDirection(DecimationDirection direction);
		void setMetric(Metric metric);
		void setInitialization(Initialization metric);

		void setTargetVertices(int target_vertices);
		void unsetTargetVertices();

		void setMaximumError(Real maximum_error);
		void unsetMaximumError();

		void setMesh(const Mesh& mesh);
		std::shared_ptr< Mesh > getMesh();

		std::shared_ptr< Mesh > doContractions(unsigned int n = 1);
		std::shared_ptr< Mesh > compute(ComputeCallback callback = NULL);

		static unsigned int nSubsets(unsigned int subset_size, unsigned int total_size);
		static void nextSubset(std::vector<unsigned int>& indices_subset,  unsigned int total_size);
		static bool solveConstrainedMinimization(const Matrix44& qem, const std::vector<Plane>& constraints, const std::vector<unsigned int>& subset, DecimationDirection direction, Vector3& result);
		static bool solveConstrainedMinimizationInequalities(const Matrix44& qem, const std::vector<Plane>& constraints, DecimationDirection direction, Vector3& result, Real& result_cost);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
	private:
		void recomputeQueue();
		void cleanAndRenumber();

		unsigned int target_vertices_;
		bool target_vertices_used_;
		Real maximum_error_;
		bool maximum_error_used_;
		DecimationDirection direction_;
		
		Real current_error_;		
		std::shared_ptr< Mesh > result_mesh_;
		
		ContractionQueue queue_;

		MetricGenerator metric_generator_;
		
		void executeEdgeContraction(const EdgeContraction& contraction);
		void collectRemovalData(Index vertex_index, Index other_index,
					std::set<Index>& edges_to_remove, std::vector<Index*>& hole_border);

		EdgeContraction computeEdgeContraction(Index edge_index);

		static Vector3 minimizeSubspace(const Matrix44& qadratic_cost);
		static Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane);
		static Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane1, Plane plane2);
		static Vector3 minimizeSubspace(const Matrix44& qadratic_cost, Plane plane1, Plane plane2, Plane plane3);
		Vector3 minimizeLagrange(const Matrix44& qadratic_cost);
		Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane);
		Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane1, Plane plane2);
		Vector3 minimizeLagrange(const Matrix44& qadratic_cost, Plane plane1, Plane plane2, Plane plane3);
	};
} 
#endif //BOUNDINGMESH_DECIMATOR_H
