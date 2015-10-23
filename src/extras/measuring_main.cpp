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

#include <iostream>
#include <sstream>
#include <vector>
#include <boundingmesh.h>
#include "utils.h"
#include "measuring.h"


int main(int argc, char** argv)
{
	if(argc != 3)
	{
		std::cerr << "Usage: boundingmesh-measure file-decimated file-original" << std::endl;
		return 0;
	}
	
	std::shared_ptr<boundingmesh::Mesh> mesh_decimated = loadMesh(argv[1]);
	std::shared_ptr<boundingmesh::Mesh> mesh_original = loadMesh(argv[2]);
	
	if(mesh_decimated || mesh_original)
	{
		std::cerr << "Failed loading a mesh" << std::endl;
		return 0;
	}

	initSolid();

	{
		SolidObject original_object(createSolidMesh(*mesh_original));
		double distance = getMeshDistance(original_object, *mesh_decimated);

		std::cerr << distance << " (max distance over vertices of decimated mesh to original)" << std::endl;
		deleteSolidObject(original_object);
	}

	{
		SolidObject original_object(createSolidMesh(*mesh_original));
		double distance = getMeshDistance2(original_object, *mesh_decimated);

		std::cerr << distance << " (max distance over non-internal vertices of decimated mesh to original)" << std::endl;
		deleteSolidObject(original_object);
	}
	
	{
		std::vector< float > distanceFromMesh;
		std::vector< float > distanceToMesh;
	
		SolidObject object_original(createSolidMesh(*mesh_original));
		SolidObject object_decimated(createSolidMesh(*mesh_decimated));
	
		for(unsigned int i = 0; i < mesh_decimated->nVertices(); ++i)
		{
			double d = getDistance(object_original, mesh_decimated->vertex(i).position());
			distanceToMesh.push_back(d);
		}
	
		for(unsigned int i = 0; i < mesh_original->nVertices(); ++i)
		{
			double d = getDistance(object_decimated, mesh_original->vertex(i).position());
			distanceFromMesh.push_back(d);
		}

		Eigen::Map< Eigen::ArrayXf > distanceFromMeshArray(distanceFromMesh.data(), distanceFromMesh.size());
		Eigen::Map< Eigen::ArrayXf > distanceToMeshArray(distanceToMesh.data(), distanceToMesh.size());
	
		std::cout << mesh_decimated->nVertices() << " & ";
		std::cout << distanceToMeshArray.minCoeff() << " & " << distanceToMeshArray.mean() << " & " << distanceToMeshArray.maxCoeff() << " & ";
		std::cout << distanceFromMeshArray.minCoeff() << " & " << distanceFromMeshArray.mean() << " & " << distanceFromMeshArray.maxCoeff();
		std::cout << std::endl;
	}

	return 0;
}
