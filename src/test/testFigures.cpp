//	Copyright (c) 2013, Andre Gaschler, Quirin Fischer
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

#include <boundingmesh.h>
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
  if (argc != 2) return 1;
  std::cout << "Testfig " << argv[1] << std::endl;

  boundingmesh::Mesh bunny;
  bunny.loadOff(std::string(argv[1]) + "/bunny/bunny.off");

  boundingmesh::Mesh teapot;
  teapot.loadOff(std::string(argv[1]) + "/teapot/teapot.off");

  boundingmesh::Decimator decimator;
  std::stringstream sstream;
  ::std::shared_ptr<boundingmesh::Mesh> decimated;
  double error = 0.00000001;
  decimator.setMesh(bunny);
  for (unsigned int i = 0; i < 8; ++i) {
    decimator.setMaximumError(error);
    decimated = decimator.compute();
    sstream.str("");
    sstream << "bunny_e-" << (8 - i) << ".wrl";
    decimated->writeWrl(sstream.str());
    error *= 10;
  }

  error = 0.00000001;
  decimator.setMesh(teapot);
  for (unsigned int i = 0; i < 8; ++i) {
    decimator.setMaximumError(error);
    decimated = decimator.compute();
    sstream.str("");
    sstream << "teapot_e-" << (8 - i) << ".wrl";
    decimated->writeWrl(sstream.str());
    error *= 10;
  }

  return 0;
}
