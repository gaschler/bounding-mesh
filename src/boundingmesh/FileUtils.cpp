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

#include "boundingmesh/FileUtils.h"
#include <iostream>

namespace boundingmesh {

FileFormat getFileFormat(std::string filename) {
  std::string file_extension = filename.substr(filename.rfind(".") + 1);

  if (file_extension == "off")
    return Off;
  else if (file_extension == "obj")
    return Obj;
  else if (file_extension == "stl")
    return Stl;
  else if (file_extension == "wrl")
    return Wrl;
  else {
    std::cout << "Detected unsupported file format: " << file_extension
              << std::endl;
    return Invalid;
  }
}

std::shared_ptr<boundingmesh::Mesh> loadMesh(std::string filename,
                                             bool debugOutput) {
  FileFormat file_format = Invalid;

  std::shared_ptr<boundingmesh::Mesh> mesh =
      std::make_shared<boundingmesh::Mesh>();

  bool loadSuccess = mesh->loadFile(filename);
  if (!loadSuccess) {
    return NULL;
  }
  return mesh;
}
}  // namespace boundingmesh
