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

#include "measuring.h"
#include <iostream>

#include <Inventor/SoDB.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/SbLinear.h>
#include <Inventor/SoPath.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>


void initSolid()
{
	DT_SetTolerance(0.0000001);
	DT_SetAccuracy(0.0000001);
}

PointVectorVector
loadConvexBodies(const std::string filename)
{
	std::vector< std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > > convexBodies;
	
	SoDB::init();

	SoGroup* root;
	SoInput input;
	if (!input.openFile(filename.c_str(), true))
	{
		std::cerr << "Error: File not found." << std::endl;
		exit(-1);
	}
	else
	{
		root = SoDB::readAll(&input);
	}
	input.closeFile();
	if (NULL == root)
	{
		std::cerr << "Error: Could not read file." << std::endl;
		exit(-1);
	}

	root->ref();

	SbViewportRegion viewportRegion;
	SoSearchAction searchAction;
	searchAction.setInterest(SoSearchAction::ALL);
	searchAction.setType(SoVRMLIndexedFaceSet::getClassTypeId());
	searchAction.apply(root);
	std::cerr << "Debug: VRML contains " << searchAction.getPaths().getLength() << " SoVRMLIndexedFaceSet." << std::endl;
	
	for (int i = 0; i < searchAction.getPaths().getLength(); ++i)
	{
		SoNode* node = searchAction.getPaths()[i]->getTail();
		//std::cout << "Debug: node type: " << node->getTypeId().getName() << std::endl;
		
		SoVRMLIndexedFaceSet* indexedFaceSet = static_cast< SoVRMLIndexedFaceSet* >(static_cast< SoVRMLShape* >(node)->geometry.getValue());
		//TODO: check if points are in global coordinates
		SoMFVec3f point = static_cast< SoVRMLCoordinate* >(indexedFaceSet->coord.getValue())->point;
		//std::cout << "Debug: point.getNum() " <<  point.getNum() << std::endl;

		std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > convexBody;
		for (int p = 0; p < point.getNum(); ++p)
		{
			convexBody.push_back(Eigen::Vector3f(point[p][0], point[p][1], point[p][2]) + 1e-4 * Eigen::Vector3f::Random());
		}
		if(point.getNum() < 6)
		{
			std::cerr << "Warning: convex no " << i << " has less than 6 points" << std::endl;
		}
		/*if(i==1)
		{
			std::cout << "Debug: convex(1): " << std::endl;
			for(auto point : convexBody)
			{
				std::cout << ", " << point.transpose() << std::endl;
			}
		}*/
		convexBodies.push_back(convexBody);
	}
	
	return convexBodies;
}


SolidObject createSolidMesh(const boundingmesh::Mesh& mesh){
	DT_ShapeHandle shape = DT_NewComplexShape(NULL);
	DT_Vector3 solid_vertex;
	
	for (unsigned int i = 0; i < mesh.nTriangles(); ++i)
	{
		const boundingmesh::Triangle& triangle = mesh.triangle(i);
		const boundingmesh::Vertex& vertex1 = mesh.vertex(triangle.vertex(0));
		DT_Begin();
		solid_vertex[0] = vertex1.position()(0);
		solid_vertex[1] = vertex1.position()(1);
		solid_vertex[2] = vertex1.position()(2);
		DT_Vertex(solid_vertex);
		
		const boundingmesh::Vertex& vertex2 = mesh.vertex(triangle.vertex(1));
		solid_vertex[0] = vertex2.position()(0);
		solid_vertex[1] = vertex2.position()(1);
		solid_vertex[2] = vertex2.position()(2);
		DT_Vertex(solid_vertex);

		const boundingmesh::Vertex& vertex3 = mesh.vertex(triangle.vertex(2));
		solid_vertex[0] = vertex3.position()(0);
		solid_vertex[1] = vertex3.position()(1);
		solid_vertex[2] = vertex3.position()(2);
		DT_Vertex(solid_vertex);

		DT_End();
	}
	
	DT_EndComplexShape();
	
	return DT_CreateObject(NULL, shape);
}

void deleteSolidObject(SolidObject object)
{
	DT_DestroyObject(object);
	//FIXME: DT_DeleteShape to prevent leak
}


double getDistance(SolidObject object, boundingmesh::Vector3 point)
{
	DT_Vector3 position = {
		static_cast<DT_Scalar>(point(0)),
		static_cast<DT_Scalar>(point(1)),
		static_cast<DT_Scalar>(point(2))
	};
	
	DT_ShapeHandle shapeHandle = DT_NewPoint(position);
	DT_ObjectHandle objectHandle = DT_CreateObject(NULL, shapeHandle);
	
	DT_Vector3 vector1;
	DT_Vector3 vector2;
	
	DT_Scalar distance = DT_GetClosestPair(
			object,
			objectHandle,
			vector1,
			vector2
	);

	//TODO: PenDepth doesn't seen to work for non-convex shapes
	DT_Bool inward = DT_GetPenDepth(
		object,
		objectHandle,
		vector1,
		vector2
	);

	if(inward)
	{
		Eigen::Vector3f p1(vector1[0], vector1[1], vector1[2]);
		Eigen::Vector3f p2(vector2[0], vector2[1], vector2[2]);
		double inner_distance = (p2 - p1).norm();
		distance = -inner_distance;
		//std::cout << "Debug: Found inward point " << distance << std::endl;
	}
	
	DT_DestroyObject(objectHandle);
	DT_DeleteShape(shapeHandle);
	return distance;
}

double getPenetrationDepth(SolidObject object, boundingmesh::Vector3 point)
{
	DT_Vector3 position = {
		static_cast<DT_Scalar>(point(0)),
		static_cast<DT_Scalar>(point(1)),
		static_cast<DT_Scalar>(point(2))
	};
	
	DT_ShapeHandle shapeHandle = DT_NewPoint(position);
	DT_ObjectHandle objectHandle = DT_CreateObject(NULL, shapeHandle);
	
	DT_Vector3 vector1;
	DT_Vector3 vector2;
	
	DT_Scalar distance;

	DT_Bool inward = DT_GetPenDepth(
		object,
		objectHandle,
		vector1,
		vector2
	);
	DT_DestroyObject(objectHandle);
	DT_DeleteShape(shapeHandle);

	if(inward)
	{
		Eigen::Vector3f p1(vector1[0], vector1[1], vector1[2]);
		Eigen::Vector3f p2(vector2[0], vector2[1], vector2[2]);
		return (p2 - p1).norm();
	}
	else
	{
		return -1;
	}	
}

double getMeshDistance(SolidObject object, boundingmesh::Mesh& mesh)
{
	double distance = 0;
	for(unsigned int i = 0; i < mesh.nVertices();++i)
	{
		double next_distance = getDistance(object, mesh.vertex(i).position());
		if(std::abs(next_distance) > std::abs(distance))
		{
			distance = next_distance;
		}
	}
	return distance;
}

/**
maximum distance over non-internal vertices of first mesh to second
*/
double getMeshDistance2(SolidObject object, boundingmesh::Mesh& mesh)
{
	SolidObject mesh_object(createSolidMesh(mesh));
	
	double distance = 0;
	int vertices_inside = 0;
	for(unsigned int i = 0; i < mesh.nVertices();++i)
	{
		boundingmesh::Vector3 x = mesh.vertex(i).position();
		if(getDistance(mesh_object, x) < 0)
		{
			vertices_inside++;
			continue;
		}
		
		double next_distance = getDistance(object, x);
		if(std::abs(next_distance) > std::abs(distance))
		{
			distance = next_distance;
		}
	}
	std::cerr << "Debug: vertices_inside: " << vertices_inside << std::endl;
	return distance;
}
