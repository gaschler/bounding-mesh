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

#include <boundingmesh.h>
#include <iostream>
#include "utils.h"
#include "measuring.h"

const double error = 0.001;

int main(int argc, char** argv)
{
	if(argc != 2)
		std::cout << "Usage: " << argv[0] << " [Mesh File]" << std::endl;

	::std::shared_ptr< boundingmesh::Mesh > mesh = loadMesh(std::string(argv[1]));
	boundingmesh::Decimator decimator;
	decimator.setMaximumError(error);
	decimator.setMesh(*mesh);
	::std::shared_ptr< boundingmesh::Mesh > result_mesh = decimator.compute();
	SolidObject solid_object = createSolidMesh(*mesh);
	double distance = getMeshDistance(solid_object, *result_mesh);

	std::cout << "Simplified " << argv[0] << " to error " << error << ", got distance " << distance << std::endl;
	deleteSolidObject(solid_object);

	if(distance > error)
		return 1;
	else
		return 0;
}
