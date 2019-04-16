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

#include <iostream>
#include <sstream>

#include "boundingmesh/Decimator.h"
#include "boundingmesh/FileUtils.h"
#include "boundingmesh/Mesh.h"
#include "optionparser/optionparser.h"

option::ArgStatus checkDirection(const option::Option& option, bool msg) {
  char in[] = "Inward";
  char out[] = "Outward";
  char any[] = "Any";
  if (strcmp(option.arg, in) == 0 || strcmp(option.arg, out) == 0 ||
      strcmp(option.arg, any) == 0)
    return option::ARG_OK;
  if (msg) std::cout << "Direction option is invalid, ignored." << std::endl;
  return option::ARG_IGNORE;
}

option::ArgStatus checkMetric(const option::Option& option, bool msg) {
  char classicQEM[] = "QEM_Classic";
  char modfiedQEM[] = "QEM_Modified";
  char minimizedConst[] = "MinConstant";
  char diagonalization[] = "Diagonalization";
  char minMerge[] = "Average";
  if (strcmp(option.arg, classicQEM) == 0 ||
      strcmp(option.arg, modfiedQEM) == 0 ||
      strcmp(option.arg, minimizedConst) == 0 ||
      strcmp(option.arg, diagonalization) == 0 ||
      strcmp(option.arg, minMerge) == 0)
    return option::ARG_OK;
  if (msg) std::cout << "Metric option is invalid, ignored." << std::endl;
  return option::ARG_IGNORE;
}

option::ArgStatus checkInitialization(const option::Option& option, bool msg) {
  char midpoint[] = "Midpoint";
  char distancePrimitives[] = "DistancePrimitives";
  if (strcmp(option.arg, midpoint) == 0 ||
      strcmp(option.arg, distancePrimitives) == 0)
    return option::ARG_OK;
  if (msg)
    std::cout << "Initialization option is invalid, ignored." << std::endl;
  return option::ARG_IGNORE;
}

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

boundingmesh::DecimationDirection parseDirection(std::string direction) {
  if (direction == "Inward")
    return boundingmesh::Inward;
  else if (direction == "Outward")
    return boundingmesh::Outward;
  else if (direction == "Any")
    return boundingmesh::Any;
  else {
    std::cout << "tried to parse invalid direction " << direction
              << ", defaulting to outward" << std::endl;
    return boundingmesh::Outward;
  }
}

boundingmesh::Metric parseMetric(std::string metric) {
  if (metric == "QEM_Classic")
    return boundingmesh::ClassicQEM;
  else if (metric == "QEM_Modified")
    return boundingmesh::ModifiedQEM;
  else if (metric == "MinConstant")
    return boundingmesh::MinimizedConstant;
  else if (metric == "Diagonalization")
    return boundingmesh::Diagonalization;
  else if (metric == "Average")
    return boundingmesh::Average;
  else {
    std::cout << "Tried to parse metric " << metric
              << ", defaulting to Average." << std::endl;
    return boundingmesh::Average;
  }
}

boundingmesh::Initialization parseInitialization(std::string initialization) {
  if (initialization == "Midpoint")
    return boundingmesh::Midpoint;
  else if (initialization == "DistancePrimitives")
    return boundingmesh::DistancePrimitives;
  else {
    std::cout << "Tried to parse initialization " << initialization
              << ", defaulting to Midpoint." << std::endl;
    return boundingmesh::Midpoint;
  }
}

enum optionIndex {
  DUMMY,
  HELP,
  TARGET_VERTICES,
  DIRECTION,
  MAXIMUM_ERROR,
  METRIC,
  INITIALIZATION,
  WHICH_FACESET,
  COLORED
};
const option::Descriptor usage[] = {
    {DUMMY, 0, "", "", option::Arg::None,
     "\nUsage: boundingmesh [options] filename [filename_out]\nSupported file "
     "formats are .off, .obj, .stl and .wrl\n"},
    {HELP, 0, "h", "help", option::Arg::None, " --help: Print usage"},
    {TARGET_VERTICES, 0, "v", "vertices", checkInt,
     " --vertices, -v: Target number of vertices"},
    {DIRECTION, 0, "d", "direction", checkDirection,
     " --direction, -d: Direction of simplification. Allowed Values: Inward, "
     "Outward or Any"},
    {MAXIMUM_ERROR, 0, "e", "error", checkFloat,
     " --error, -e: Maximum error of a simplification step"},
    {METRIC, 0, "m", "metric", checkMetric,
     " --metric, -m: Used Error Metric. Allowed Values: QEM_Classic, "
     "QEM_Modified, MinConstant, Diagonalization, Average"},
    {INITIALIZATION, 0, "i", "init", checkInitialization,
     " --init, -i: Initialization method for vertex error metrices. Allowed "
     "Values: Midpoint, DistancePrimitives"},
    {WHICH_FACESET, 0, "", "faceset", checkInt,
     " --faceset: If input file is .wrl, loads only faceset with this number"},
    {COLORED, 0, "", "colored", option::Arg::None,
     " --colored: If output file is .wrl, different colors are used for "
     "non-connected submeshes."},
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
    filename_out.insert(filename_out.rfind("/") + 1, "boundingmesh_");
  } else {
    std::cout << "Expected filename(s)." << std::endl;
    option::printUsage(std::cout, usage);
    return 1;
  }

  boundingmesh::Real maximum_error =
      std::numeric_limits<boundingmesh::Real>::max();
  boundingmesh::DecimationDirection direction = boundingmesh::Outward;

  if (options[DIRECTION]) {
    direction = parseDirection(std::string(options[DIRECTION].arg));
  }

  boundingmesh::Mesh mesh;
  boundingmesh::FileFormat file_format_in = boundingmesh::Invalid;
  boundingmesh::FileFormat file_format_out = boundingmesh::Invalid;

  file_format_in = boundingmesh::getFileFormat(filename_in);
  file_format_out = boundingmesh::getFileFormat(filename_out);

  if (file_format_in == boundingmesh::Off)
    mesh.loadOff(filename_in);
  else if (file_format_in == boundingmesh::Obj)
    mesh.loadObj(filename_in);
  else if (file_format_in == boundingmesh::Wrl) {
    if (options[WHICH_FACESET])
      mesh.loadWrl(filename_in, atoi(options[WHICH_FACESET].arg));
    else
      mesh.loadWrl(filename_in);
  } else if (file_format_in == boundingmesh::Stl)
    mesh.loadStl(filename_in);
  else {
    std::cout << "Couldn't load " << filename_in << std::endl;
    return 1;
  }

  boundingmesh::Decimator decimator(direction);
  if (options[TARGET_VERTICES]) {
    decimator.setTargetVertices(atoi(options[TARGET_VERTICES].arg));
  }
  if (options[MAXIMUM_ERROR]) {
    decimator.setMaximumError(atof(options[MAXIMUM_ERROR].arg));
  }
  if (options[METRIC]) {
    decimator.setMetric(parseMetric(options[METRIC].arg));
  }
  if (options[INITIALIZATION]) {
    decimator.setInitialization(
        parseInitialization(options[INITIALIZATION].arg));
  }

  decimator.setMesh(mesh);
  ::std::shared_ptr<boundingmesh::Mesh> result_mesh = decimator.compute();

  if (file_format_out == boundingmesh::Off)
    result_mesh->writeOff(filename_out);
  else if (file_format_out == boundingmesh::Obj)
    result_mesh->writeObj(filename_out);
  else if (file_format_out == boundingmesh::Wrl) {
    if (options[COLORED])
      result_mesh->writeWrl(filename_out, true);
    else
      result_mesh->writeWrl(filename_out);

  } else if (file_format_out == boundingmesh::Stl)
    result_mesh->writeStl(filename_out, false);
  else {
    std::cout << "Couldn't write " << filename_out << std::endl;
    return 1;
  }

  return 0;
}
