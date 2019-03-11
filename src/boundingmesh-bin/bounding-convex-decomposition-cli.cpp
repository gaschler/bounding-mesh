//	Copyright (c) 2016, Andre Gaschler
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
#include "../../thirdparty/optionparser.h"

#include <iostream>
#include <sstream>

option::ArgStatus checkInt(const option::Option& option, bool msg) {
  char* endptr = 0;
  if (option.arg != 0 && strtol(option.arg, &endptr, 10)) {
  };
  if (endptr != option.arg && *endptr == 0) return option::ARG_OK;

  if (msg)
    std::cout << "Expected integer, option ignored: " << option.arg
              << std::endl;
  return option::ARG_IGNORE;
}

option::ArgStatus checkFloat(const option::Option& option, bool msg) {
  char* endptr = 0;
  if (option.arg != 0 && strtod(option.arg, &endptr)) {
  };
  if (endptr != option.arg && *endptr == 0) return option::ARG_OK;

  if (msg)
    std::cout << "Expected float, option ignored. " << option.arg << std::endl;
  return option::ARG_IGNORE;
}

enum optionIndex { DUMMY, HELP, TARGET_ERROR, VOXELS, ALPHA };
const option::Descriptor usage[] = {
    {DUMMY, 0, "", "", option::Arg::None,
     "\nUsage: bounding-convex-decomposition [options] filename "
     "[filename_out.wrl]\nSupported input file formats are .off, .obj, .stl "
     "and .wrl\n"},
    {HELP, 0, "h", "help", option::Arg::None, " --help: Print usage"},
    {TARGET_ERROR, 0, "e", "error", checkFloat,
     " --error, -e: Allow bounding mesh error in post-processing, measured in "
     "bounding box lengths, default is 0.02"},
    {VOXELS, 0, "x", "voxels", checkInt,
     " --voxels, -x: Use number of voxels, default is 50000"},
    {ALPHA, 0, "a", "alpha", checkFloat,
     " --alpha, -a: Alpha parameter, increases the number of decompositions, "
     "default is 1"},
    {0, 0, 0, 0, 0, 0}};

int main(int argc, char** argv) {
  argc -= (argc > 0);
  argv += (argc > 0);  // skip program name argv[0] if present
  option::Stats stats(usage, argc, argv);
  option::Option* options = new option::Option[stats.options_max];
  option::Option* buffer = new option::Option[stats.buffer_max];
  option::Parser parse(usage, argc, argv, options, buffer);

  if (parse.error()) return 1;

  if (options[HELP]) {
    option::printUsage(std::cout, usage);
  }

  std::string filename_in;
  std::string filename_out;

  if (parse.nonOptionsCount() == 2) {
    filename_in = parse.nonOption(0);
    filename_out = parse.nonOption(1);
  } else if (parse.nonOptionsCount() == 1) {
    filename_in = parse.nonOption(0);
    filename_out = filename_in;
    filename_out.insert(filename_out.rfind("/") + 1,
                        "bounding_convex_decomposition_");
    filename_out.append(".wrl");
  } else {
    std::cout << "Expected filename(s)." << std::endl;
    option::printUsage(std::cout, usage);
    return 1;
  }

  boundingmesh::Real alpha = 1;
  if (options[ALPHA]) {
    alpha = atof(options[ALPHA].arg);
  }
  int voxels = 50000;
  if (options[VOXELS]) {
    voxels = atoi(options[VOXELS].arg);
  }
  boundingmesh::Real target_error = 0.02;
  if (options[TARGET_ERROR]) {
    target_error = atof(options[TARGET_ERROR].arg);
  }

  std::cout << "Loading mesh..." << std::endl;
  std::shared_ptr<boundingmesh::Mesh> mesh =
      boundingmesh::loadMesh(filename_in);
  if (mesh == NULL) {
    std::cerr << "Error: could not load mesh" << std::endl;
    return 1;
  }
  if (mesh->nVertices() < 4) {
    std::cerr << "Error: mesh could not be read" << std::endl;
    return 2;
  }

  boundingmesh::Real bounding_box_diagonal = mesh->getBoundingBoxDiagonal();
  std::cout << "Filling holes in geometry..." << std::endl;
  mesh->closeHoles();

  boundingmesh::SegmenterDownsampling segmenter;
  segmenter.setMaxPasses(10);
  segmenter.setAlpha(alpha);
  segmenter.setDelta(alpha / 2);

  std::cout << "Voxelizing mesh..." << std::endl;
  segmenter.setMesh(mesh, voxels);
  std::cout << "Calculating bounding convex decomposition..." << std::endl;
  segmenter.compute();
  std::vector<std::shared_ptr<boundingmesh::Mesh> > decomposition =
      segmenter.getSegmentation();

  std::cout << "Simplifying bounding convex decomposition with the bounding "
               "mesh algorithm..."
            << std::endl;
  std::vector<std::shared_ptr<boundingmesh::Mesh> > decomposition_decimated;
  for (int i = 0; i < decomposition.size(); ++i) {
    boundingmesh::Decimator decimator;
    decimator.setMesh(*decomposition[i]);
    decimator.setMaximumError(
        std::pow(target_error * bounding_box_diagonal, 2));
    decomposition_decimated.push_back(decimator.compute());
  }

  boundingmesh::Mesh::writeMultimeshWrl(decomposition_decimated, filename_out,
                                        true);

  return 0;
}
