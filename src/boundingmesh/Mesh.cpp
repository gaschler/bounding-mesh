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

#include "boundingmesh/Mesh.h"
#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <queue>
#include <sstream>
#include <unordered_map>
// for Debugging
#include <cassert>
#include <iostream>

// Use coin to load .wrl files if available
#ifdef COIN_AVAILABLE
#include <Inventor/SoDB.h>

#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoSeparator.h>

#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>

#include <Inventor/SbLinear.h>
#include <Inventor/SoPath.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoWriteAction.h>
#endif

#ifdef CGAL_AVAILABLE
#include "CGAL/Cartesian.h"
#include "CGAL/convex_hull_3.h"
#include "CGAL/Exact_predicates_inexact_constructions_kernel.h"
#include "CGAL/Polyhedron_3.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Cartesian<double>  K;
typedef CGAL::Polyhedron_3<K> Polyhedron_3;
// define point creator
typedef K::Point_3 Point_3;
typedef Polyhedron_3::Facet_iterator Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator
    Halfedge_facet_circulator;
#endif

#ifdef QHULL_AVAILABLE
// QHull for convex hull generation
// Using the C interface
extern "C" {
//#define qh_QHpointer 0
//#include <qhull/qhull_a.h>
}
#endif

namespace boundingmesh {

// Implementation of class VertexPosition
VertexPosition::VertexPosition(const Vector3& position)
    : position_(position), index_(0), searching_(true) {}

VertexPosition::VertexPosition(Index index, const Vector3& position)
    : position_(position), index_(index), searching_(false) {}

Index VertexPosition::index() const { return index_; }

bool operator<(const VertexPosition& lhs, const VertexPosition& rhs) {
  if (std::abs(lhs.position_(0) - rhs.position_(0)) > vertex_epsilon)
    return lhs.position_(0) < rhs.position_(0);
  else if (std::abs(lhs.position_(1) - rhs.position_(1)) > vertex_epsilon)
    return lhs.position_(1) < rhs.position_(1);
  else
    return lhs.position_(2) < rhs.position_(2);
}
bool operator>(const VertexPosition& lhs, const VertexPosition& rhs) {
  return operator<(rhs, lhs);
}

// Implementation of class VertexPositionSet
VertexPositionSet::VertexPositionSet(Mesh* mesh) : mesh_(mesh), set_() {}

Index VertexPositionSet::addVertex(const Vector3& position) {
  std::set<VertexPosition, std::less<VertexPosition>,
           Eigen::aligned_allocator<VertexPosition>>::iterator iterator =
      set_.find(VertexPosition(position));
  if (iterator == set_.end()) {
    // New position
    Index new_index = mesh_->addVertex(position);
    std::pair<std::set<VertexPosition, std::less<VertexPosition>,
                       Eigen::aligned_allocator<VertexPosition>>::iterator,
              bool>
        returned = set_.insert(VertexPosition(new_index, position));
    assert(returned.second == true);
    iterator = returned.first;
  }
  return iterator->index();
}

// Implementation of class Mesh
Mesh::Mesh()
    : n_valid_vertices_(0),
      n_valid_edges_(0),
      n_valid_triangles_(0),
      vertices_(),
      edges_(),
      triangles_(),
      dirty_(false),
      deleted_vertices_(),
      deleted_edges_(),
      deleted_triangles_(),
      n_original(0),
      debug_vrml() {}

Mesh::Mesh(const Mesh& mesh) {
  vertices_ = std::deque<Vertex>(mesh.vertices_);
  edges_ = std::deque<Edge>(mesh.edges_);
  triangles_ = std::deque<Triangle>(mesh.triangles_);

  dirty_ = mesh.dirty_;
  deleted_vertices_ = std::stack<Index>(mesh.deleted_vertices_);
  deleted_edges_ = std::stack<Index>(mesh.deleted_edges_);
  deleted_triangles_ = std::stack<Index>(mesh.deleted_triangles_);

  n_valid_vertices_ = mesh.n_valid_vertices_;
  n_valid_edges_ = mesh.n_valid_edges_;
  n_valid_triangles_ = mesh.n_valid_triangles_;

  n_original = mesh.n_original;
  debug_vrml = mesh.debug_vrml;
}

Mesh::Mesh(const std::vector<Vector3>& vertices,
           const std::vector<Index*>& triangles) {
  for (unsigned int i = 0; i < vertices.size(); ++i) addVertex(vertices[i]);

  for (unsigned int i = 0; i < triangles.size(); ++i)
    addTriangle(triangles[i][0], triangles[i][1], triangles[i][2]);
}

Mesh::~Mesh() {}

Mesh& Mesh::operator=(Mesh other) {
  swap(*this, other);
  return *this;
}

void swap(Mesh& first, Mesh& second) {
  first.vertices_.swap(second.vertices_);
  first.edges_.swap(second.edges_);
  first.triangles_.swap(second.triangles_);

  std::swap(first.n_valid_vertices_, second.n_valid_vertices_);
  std::swap(first.n_valid_edges_, second.n_valid_edges_);
  std::swap(first.n_valid_triangles_, second.n_valid_triangles_);

  std::swap(first.dirty_, second.dirty_);
  std::swap(first.deleted_vertices_, second.deleted_vertices_);
  std::swap(first.deleted_edges_, second.deleted_edges_);
  std::swap(first.deleted_triangles_, second.deleted_triangles_);

  std::swap(first.n_original, second.n_original);
  std::swap(first.debug_vrml, second.debug_vrml);
}

void Mesh::loadOff(const std::string& filename) {
  std::ifstream file(filename.c_str());
  std::string line;
  int state = 0;
  unsigned int n_vertices = 0;
  unsigned int n_faces = 0;
  unsigned int n_edges = 0;
  while (!file.eof() && file.good()) {
    std::getline(file, line);
    line.erase(0, line.find_first_not_of(' '));
    line.erase(line.find_last_not_of(" \n\r\t") + 1);
    if (line.size() == 0) continue;
    if (line.at(0) == '#') continue;

    std::stringstream sstream(line);
    Real x, y, z;
    Index a, b, c;

    int n_face_vertices;

    switch (state) {
      case 0:  // Reading Preamble
        if (line == "OFF")
          state = 1;
        else {
          std::cout << "Error reading .off file: first line is bad:"
                    << line.length() << (int)line.at(3) << std::endl;
          return;
        }
        break;
      case 1:  // Reading vertex/face count
        sstream >> n_vertices >> n_faces >> n_edges;
        if (n_vertices == 0 || n_faces == 0) {
          std::cout << "Error reading .off file: Vertex/face count is zero"
                    << std::endl;
          return;
        }
        state = 2;
        break;
      case 2:                    // Reading vertices
        sstream >> x >> y >> z;  // Only read position, ignore color if existent
        addVertex(Vector3(x, y, z));

        if (vertices_.size() == n_vertices) state = 3;
        break;
      case 3:  // Reading faces
        sstream >> n_face_vertices;
        if (n_face_vertices < 3) {
          std::cerr
              << "Error reading .off file: Reading face with 2 or less vertices"
              << std::endl;
          clean();
          return;
        } else if (n_face_vertices == 3) {
          sstream >> a >> b >>
              c;  // Only read geometric information, ignore color if existent
          if (a > vertices_.size() || b > vertices_.size() ||
              c > vertices_.size())
            std::cout << "Error reading .off file: invalid index" << std::endl;
          addTriangle(a, b, c);
        } else {
          std::cout << "Error reading .off file: Reading face with more than 3 "
                       "vertices"
                    << std::endl;
        }
        break;
      default:
        std::cout << "Error reading .off file: Broken reading state"
                  << std::endl;
        clean();
        return;
    }
  }
  n_original = vertices_.size();
  if (vertices_.size() != n_vertices || triangles_.size() != n_faces) {
    std::cerr << "Error reading .off file: Vertex/face count is incorrect"
              << std::endl;
    clean();
    return;
  }
}

void Mesh::loadObj(const std::string& filename) {
  std::ifstream file(filename.c_str());
  std::string line;

  Real x, y, z;
  std::string a_str, b_str, c_str;
  Index a, b, c;

  while (!file.eof() && file.good()) {
    std::getline(file, line);
    if (line.size() == 0) continue;
    if (line.at(0) == '#') continue;
    std::stringstream linestream(line);
    std::string mode;
    linestream >> mode;
    if (mode == "v") {
      // Reading vertex
      linestream >> x >> y >> z;  // Only read position, ignore color if
                                  // existent
      addVertex(Vector3(x, y, z));
    } else if (mode == "f") {
      // Reading face
      linestream >> a_str >> b_str >>
          c_str;  // Only read geometric information, ignore color if existent
      // if(linestream.rdbuf()->in_avail() != 0)
      //	std::cout << "Error reading .obj file: Reading face with more
      // than 3 vertices" << std::endl;
      std::stringstream component_stream(a_str);
      std::getline(component_stream, a_str, '/');
      component_stream.str(b_str);
      std::getline(component_stream, b_str, '/');
      component_stream.str(c_str);
      std::getline(component_stream, c_str, '/');
      a = std::atoi(a_str.c_str()) - 1;
      b = std::atoi(b_str.c_str()) - 1;
      c = std::atoi(c_str.c_str()) - 1;

      if (a > vertices_.size() || b > vertices_.size() || c > vertices_.size())
        std::cout << "Error reading .off file: invalid index" << std::endl;
      addTriangle(a, b, c);
    }
  }
  n_original = vertices_.size();
}

#ifdef COIN_AVAILABLE
struct CallbackData {
  Mesh* mesh;
  VertexPositionSet* vertices;

  SbMatrix* matrix;
};

void triangleCallback(void* userData, SoCallbackAction* action,
                      const SoPrimitiveVertex* v1, const SoPrimitiveVertex* v2,
                      const SoPrimitiveVertex* v3) {
  CallbackData* data = static_cast<CallbackData*>(userData);

  SbMatrix* matrix = data->matrix;
  VertexPositionSet* vertices = data->vertices;
  Mesh* mesh = data->mesh;

  SbVec3f points[3];

  matrix->multVecMatrix(v1->getPoint(), points[0]);
  matrix->multVecMatrix(v2->getPoint(), points[1]);
  matrix->multVecMatrix(v3->getPoint(), points[2]);

  Index indices[3];
  for (unsigned int i = 0; i < 3; ++i) {
    Vector3 position;
    position << points[i][0], points[i][1], points[i][2];
    indices[i] = vertices->addVertex(position);
  }
  if (indices[0] == indices[1] || indices[0] == indices[2] ||
      indices[1] == indices[2]) {
#ifndef NDEBUG
    std::cout << "Skipping degenerate triangle " << indices[0] << ", "
              << indices[1] << ", " << indices[2] << std::endl;
#endif
  } else
    mesh->addTriangle(indices[0], indices[1], indices[2]);
}
#endif

void Mesh::loadWrl(const std::string& filename, int faceset, bool debugOutput) {
#ifndef COIN_AVAILABLE
  std::cout << "Coin required, can't load .wrl." << std::endl;
#else
  SoDB::init();

  SoGroup* root;
  SoInput input;
  if (!input.openFile(filename.c_str(), true)) {
    std::cout << "Error: File not found." << std::endl;
    exit(-1);
  } else {
    root = SoDB::readAll(&input);
  }

  input.closeFile();
  if (NULL == root) {
    std::cerr << "Error: Could not read file." << std::endl;
    exit(-1);
  }

  root->ref();

  SbViewportRegion viewportRegion;
  SoSearchAction searchAction;
  searchAction.setInterest(SoSearchAction::ALL);
  searchAction.setType(SoShape::getClassTypeId());
  searchAction.apply(root);

  CallbackData cb_data;
  cb_data.mesh = this;

  if (debugOutput) {
    std::cout << "Note: VRML contains " << searchAction.getPaths().getLength()
              << " facesets." << std::endl;
  }
  for (int i = 0; i < searchAction.getPaths().getLength(); ++i) {
    if (faceset != -1 && i != faceset) continue;

    SoGetMatrixAction* getMatrixAction = new SoGetMatrixAction(viewportRegion);
    getMatrixAction->apply(searchAction.getPaths()[i]);
    SbMatrix matrix = getMatrixAction->getMatrix();

    VertexPositionSet vertices(this);
    cb_data.vertices = &vertices;
    cb_data.matrix = &matrix;

    SoCallbackAction callbackAction;
    callbackAction.addTriangleCallback(SoShape::getClassTypeId(),
                                       triangleCallback, &cb_data);
    callbackAction.apply(searchAction.getPaths()[i]);
  }
#endif
}

void Mesh::loadStl(const std::string& filename) {
  std::ifstream file(filename.c_str());
  std::string start;
  file >> start;
  std::cout << "start: " << start << std::endl;
  bool binary = true;
  if (start == "solid") binary = false;
  file.close();

  VertexPositionSet vertices(this);
  if (binary) {
    std::cout << "loading binary" << std::endl;
    if (sizeof(float) != 4) {
      std::cout << "Using floats with bad size. abort." << std::endl;
      return;
    }

    file.open(filename.c_str(), std::ios::binary);
    file.seekg(80, file.beg);
    uint32_t num_faces;
    file.read((char*)&num_faces, 4);
    while (!file.eof() && file.good()) {
      float x, y, z;
      file.read((char*)&x, 4);
      file.read((char*)&y, 4);
      file.read((char*)&z, 4);
      Vector3 normal;
      normal << x, y, z;
      Index indices[3];
      for (unsigned int i = 0; i < 3; ++i) {
        file.read((char*)&x, 4);
        file.read((char*)&y, 4);
        file.read((char*)&z, 4);
        Vector3 position;
        position << x, y, z;
        indices[i] = vertices.addVertex(position);
      }
      Vector3 v0v1 =
          vertex(indices[1]).position() - vertex(indices[0]).position();
      Vector3 v0v2 =
          vertex(indices[2]).position() - vertex(indices[0]).position();
      Vector3 computed_normal = v0v1.cross(v0v2).normalized();
      if (std::abs(computed_normal.dot(normal) - 1) > normal_epsilon) {
        //	std::cout << "Overriding bad normal: " << normal.transpose() <<
        //" computed " << computed_normal.transpose() << std::endl;
      }
      if (!(indices[0] == indices[1] || indices[0] == indices[2] ||
            indices[1] == indices[2]))
        addTriangle(indices[0], indices[1], indices[2]);
      file.seekg(2, file.cur);
    }
    file.close();
  } else {
    file.open(filename.c_str());
    std::string line;
    std::getline(file, line);
    // Skip header
    while (!file.eof() && file.good()) {
      std::getline(file, line);
      line.erase(0, line.find_first_not_of(' '));
      std::stringstream linestream(line);
      std::string token;
      linestream >> token;
      if (token == "endsolid")
        break;
      else if (token == "facet") {
        linestream >> token;
        if (token != "normal") {
          std::cout << "Bad stl file: expected \"normal\"" << std::endl;
          break;
        }
        float x, y, z;
        Vector3 normal;
        linestream >> x >> y >> z;
        normal << x, y, z;
        std::getline(file, line);
        line.erase(0, line.find_first_not_of(' '));
        if (line != "outer loop") {
          std::cout << "Bad stl file: expected \"outer loop\", got " << line
                    << std::endl;
          break;
        }
        Index indices[3];
        for (unsigned int i = 0; i < 3; ++i) {
          std::getline(file, line);
          line.erase(0, line.find_first_not_of(' '));
          linestream.str();
          linestream.clear();
          linestream.str(line);
          linestream >> token;
          if (token != "vertex") {
            std::cout << "bad stl file: expected \"vertex\" got " << token
                      << std::endl;
            break;
          }
          linestream >> x >> y >> z;
          Vector3 position;
          position << x, y, z;
          indices[i] = vertices.addVertex(position);
        }
        Vector3 v0v1 =
            vertex(indices[1]).position() - vertex(indices[0]).position();
        Vector3 v0v2 =
            vertex(indices[2]).position() - vertex(indices[0]).position();
        Vector3 computed_normal = v0v1.cross(v0v2).normalized();
        if (std::abs(computed_normal.dot(normal) - 1) > normal_epsilon) {
          std::cout << "Overriding bad normal: " << normal.transpose()
                    << " computed " << computed_normal.transpose() << std::endl;
        }

        if (indices[0] == indices[1] || indices[0] == indices[2] ||
            indices[1] == indices[2])
          std::cout << "Skipping degenerate triangle " << indices[0] << ", "
                    << indices[1] << ", " << indices[2] << std::endl;
        else
          addTriangle(indices[0], indices[1], indices[2]);
        std::getline(file, line);
        line.erase(0, line.find_first_not_of(' '));
        if (line != "endloop") {
          std::cout << "bad stl file: expected \"endloop\", got " << line
                    << std::endl;
          break;
        }
        std::getline(file, line);
        line.erase(0, line.find_first_not_of(' '));
        if (line != "endfacet") {
          std::cout << "bad stl file: expected \"endfacet\", got " << line
                    << std::endl;
          break;
        }
      } else {
        std::cout << "bad stl file: expected \"facet\" or \"endsolid\", got "
                  << token << std::endl;
        break;
      }
    }
    file.close();
  }
  n_original = vertices_.size();
}

void Mesh::writeOff(const std::string& filename) {
  cleanAndRenumber();

  std::cout << "Writing off " << filename << std::endl;

  std::ofstream file(filename.c_str());
  file << "OFF" << std::endl;
  file << nVertices() << " " << nTriangles() << " " << nEdges() << std::endl;
  for (unsigned int i = 0; i < nVertices(); ++i) {
    file << vertex(i).position()(0) << " " << vertex(i).position()(1) << " "
         << vertex(i).position()(2) << std::endl;
  }
  file << std::endl;
  for (unsigned int i = 0; i < nTriangles(); ++i) {
    file << "3 " << triangle(i).vertex(0) << " " << triangle(i).vertex(1) << " "
         << triangle(i).vertex(2) << std::endl;
  }
  file.close();
}

void Mesh::writeObj(const std::string& filename) {
  cleanAndRenumber();
  std::cout << "writing obj " << filename << std::endl;
  std::ofstream file(filename.c_str());
  file << "#Created by boundingmesh" << std::endl;
  file << "# " << vertices_.size() << " vertices total" << std::endl;
  for (unsigned int i = 0; i < vertices_.size(); ++i) {
    file << "v " << vertices_[i].position_[0] << " "
         << vertices_[i].position_[1] << " " << vertices_[i].position_[2]
         << std::endl;
  }
  file << "# " << triangles_.size() << " triangles total" << std::endl;
  for (unsigned int i = 0; i < triangles_.size(); ++i) {
    // Obj indices count from 1
    file << "f " << (triangles_[i].vertices_[0] + 1) << " "
         << (triangles_[i].vertices_[1] + 1) << " "
         << (triangles_[i].vertices_[2] + 1) << std::endl;
  }
  file.close();
}

void Mesh::writeWrl(const std::string& filename, bool colored) {
  cleanAndRenumber();

  std::vector<std::shared_ptr<Mesh>> submeshes = computeSubmeshes();

  writeMultimeshWrl(submeshes, filename, colored);
}

void Mesh::writeStl(const std::string& filename, bool binary) {
  std::cout << "Writing stl" << std::endl;
  cleanAndRenumber();
  if (binary) {
    std::ofstream file(filename.c_str(), std::ios::binary);
    std::string header("Binary STL " + filename + " created by BoundingMesh");
    header.resize(80);
    file.write(header.c_str(), 80);
    uint32_t num_faces = (uint32_t)triangles_.size();
    file.write((char*)&num_faces, 4);
    for (unsigned int i = 0; i < triangles_.size(); ++i) {
      Vector3 p1, p2, p3;
      p1 = vertices_[triangles_[i].vertices_[0]].position_;
      p2 = vertices_[triangles_[i].vertices_[1]].position_;
      p3 = vertices_[triangles_[i].vertices_[2]].position_;
      Vector3 normal = (p2 - p1).cross(p3 - p1).normalized();
      Real x, y, z;
      x = normal(0);
      y = normal(1);
      z = normal(2);
      file.write((char*)&x, 4);
      file.write((char*)&y, 4);
      file.write((char*)&z, 4);
      x = p1(0);
      y = p1(1);
      z = p1(2);
      file.write((char*)&x, 4);
      file.write((char*)&y, 4);
      file.write((char*)&z, 4);
      x = p2(0);
      y = p2(1);
      z = p2(2);
      file.write((char*)&x, 4);
      file.write((char*)&y, 4);
      file.write((char*)&z, 4);
      x = p3(0);
      y = p3(1);
      z = p3(2);
      file.write((char*)&x, 4);
      file.write((char*)&y, 4);
      file.write((char*)&z, 4);
      short attribute_byte_count = 0;
      file.write((char*)&attribute_byte_count, 2);
    }
    file.close();
  } else {
    std::ofstream file(filename.c_str());
    file << "solid " << filename << "_created_by_BoundingMesh" << std::endl;
    // Write triangles
    for (unsigned int i = 0; i < triangles_.size(); ++i) {
      Vector3 p1, p2, p3;
      p1 = vertices_[triangles_[i].vertices_[0]].position_;
      p2 = vertices_[triangles_[i].vertices_[1]].position_;
      p3 = vertices_[triangles_[i].vertices_[2]].position_;
      Vector3 normal = (p2 - p1).cross(p3 - p1).normalized();
      file << "    facet normal " << normal(0) << " " << normal(1) << " "
           << normal(2) << std::endl;
      file << "        outer loop" << std::endl;
      file << "            vertex " << p1(0) << " " << p1(1) << " " << p1(2)
           << std::endl;
      file << "            vertex " << p2(0) << " " << p2(1) << " " << p2(2)
           << std::endl;
      file << "            vertex " << p3(0) << " " << p3(1) << " " << p3(2)
           << std::endl;
      file << "        endloop" << std::endl;
      file << "    endfacet" << std::endl;
    }
    file << "endsolid " << filename << "_created_by_BoundingMesh";
    file.close();
  }
}

std::vector<std::shared_ptr<Mesh>> Mesh::computeSubmeshes() {
  std::vector<std::shared_ptr<Mesh>> submeshes;

  std::set<Index> used_vertices;
  Index next_start_vertex = 0;

  while (used_vertices.size() < nVertices()) {
    std::shared_ptr<Mesh> connected_mesh = std::make_shared<Mesh>();
    std::set<Index> visited_triangles;
    Index seed_vertex_index = next_start_vertex;
    while (used_vertices.find(seed_vertex_index) != used_vertices.end()) {
      seed_vertex_index++;
    }
    next_start_vertex = seed_vertex_index + 1;

    std::queue<Index> triangles_to_add;
    const Vertex& seed_vertex = vertex(seed_vertex_index);
    for (unsigned int i = 0; i < seed_vertex.nTriangles(); ++i) {
      triangles_to_add.push(seed_vertex.triangle(i));
      visited_triangles.insert(seed_vertex.triangle(i));
    }

    std::unordered_map<Index, Index> mapped_vertex_indices;

    while (triangles_to_add.size() > 0) {
      Index next_triangle_index = triangles_to_add.front();

      const Triangle& next_triangle = triangle(next_triangle_index);
      for (unsigned int i = 0; i < 3; ++i) {
        Index next_vertex = next_triangle.vertex(i);
        if (mapped_vertex_indices.count(next_vertex) == 0) {
          mapped_vertex_indices[next_vertex] = connected_mesh->nVertices();
          connected_mesh->addVertex(vertex(next_vertex).position());
          used_vertices.insert(next_vertex);

          const Vertex& added_vertex = vertex(next_vertex);
          for (unsigned int j = 0; j < added_vertex.nTriangles(); ++j) {
            Index triangle_to_visit = added_vertex.triangle(j);
            if (visited_triangles.find(triangle_to_visit) ==
                visited_triangles.end()) {
              triangles_to_add.push(triangle_to_visit);
              visited_triangles.insert(triangle_to_visit);
            }
          }
        }
      }

      connected_mesh->addTriangle(
          mapped_vertex_indices[next_triangle.vertex(0)],
          mapped_vertex_indices[next_triangle.vertex(1)],
          mapped_vertex_indices[next_triangle.vertex(2)]);
      triangles_to_add.pop();
    }

    submeshes.push_back(connected_mesh);
  }
  return submeshes;
}

void Mesh::writeMultimeshWrl(std::vector<std::shared_ptr<Mesh>> submeshes,
                             const std::string& filename, bool colored) {
  std::cout << "Info: Writing wrl " << filename << std::endl;
  if (submeshes.size() == 0) {
    std::cout << "No meshes supplied" << std::endl;
    return;
  }

  std::ofstream file(filename.c_str());
  file << "#VRML V2.0 utf8" << std::endl << std::endl;
  file << "#Created by boundingmesh" << std::endl << std::endl;

  std::vector<Vector3> colors;
  if (colored) colors = generateColors(submeshes.size());

  for (int submesh_i = 0; submesh_i < submeshes.size(); ++submesh_i) {
    const Mesh& submesh = *submeshes[submesh_i];

    if (submesh.nVertices() == 0) {
      std::cout << "Warning: Mesh is empty" << std::endl;
      continue;
    }

    file << "Shape {" << std::endl;
    if (!colored) {
      file << "\tappearance Appearance {\n\t\tmaterial Material "
              "{\n\t\t\ttransparency 0\n\t\t\tdiffuseColor 1 1 1"
           << std::endl;
      file << "\n\t\t}\n\t}" << std::endl;
    } else {
      file << "\tappearance Appearance {\n\t\tmaterial Material "
              "{\n\t\t\ttransparency 0.2\n\t\t\tdiffuseColor ";
      file << colors[submesh_i](0) << " " << colors[submesh_i](1) << " "
           << colors[submesh_i](2) << std::endl;
      file << "\n\t\t}\n\t}" << std::endl;
    }
    file << "geometry IndexedFaceSet {" << std::endl;
    file << "\tsolid TRUE" << std::endl;
    file << "\tcoord Coordinate {" << std::endl;
    file << "\t\tpoint [" << std::endl;

    // Write vertices
    for (unsigned int i = 0; i < submesh.nVertices(); ++i) {
      file << "\t\t\t" << submesh.vertex(i).position()(0) << " "
           << submesh.vertex(i).position()(1) << " "
           << submesh.vertex(i).position()(2) << ", " << std::endl;
    }

    file << "\t\t]" << std::endl;
    file << "\t}" << std::endl;
    file << "\tcoordIndex [" << std::endl;

    // Write triangles
    for (unsigned int i = 0; i < submesh.nTriangles(); ++i) {
      file << "\t\t" << submesh.triangle(i).vertex(0) << ", "
           << submesh.triangle(i).vertex(1) << ", "
           << submesh.triangle(i).vertex(2) << ", "
           << "-1," << std::endl;
    }

    file << "\t]" << std::endl;
    file << "\t}" << std::endl;

#if DEBUG_COLOR
    file << "\t\tcolor Color {" << std::endl;
    file << "\t\t\tcolor [" << std::endl;
    for (unsigned int i = 0; i < vertices_.size(); ++i) {
      if (i < n_original)
        file << "\t\t\t\t1 1 1" << std::endl;
      else {
        float val =
            (1 - (float)(i - n_original) / (vertices_.size() - n_original));
        file << "\t\t\t\t" << val << " " << 0 << " " << 0 << std::endl;
      }
    }
    file << "\t\t\t]" << std::endl;
    file << "\t\t}" << std::endl;
#endif
    file << "}" << std::endl;
  }
#if DEBUG_VRML
  file << debug_vrml << std::endl;
#endif
  file.close();
}

unsigned int Mesh::nVertices() const { return n_valid_vertices_; }
unsigned int Mesh::nEdges() const { return n_valid_edges_; }
unsigned int Mesh::nTriangles() const { return n_valid_triangles_; }

const Vertex& Mesh::vertex(Index i) const {
  assert(i < vertices_.size());
  return vertices_[i];
}
const Edge& Mesh::edge(Index i) const {
  assert(i < edges_.size());
  return edges_[i];
}
const Triangle& Mesh::triangle(Index i) const {
  assert(i < triangles_.size());
  return triangles_[i];
}

Index Mesh::addVertex(const Vector3& position) {
  Index new_index = 0;
  if (deleted_vertices_.size() > 0) {
    new_index = deleted_vertices_.top();
    deleted_vertices_.pop();
    vertices_[new_index] = Vertex(position);
  } else {
    new_index = vertices_.size();
    vertices_.push_back(Vertex(position));
  }
  n_valid_vertices_++;
  return new_index;
}

Index Mesh::addTriangle(Index vertex1, Index vertex2, Index vertex3) {
  assert(vertex1 < vertices_.size());
  assert(vertex2 < vertices_.size());
  assert(vertex3 < vertices_.size());
  assert(vertex1 != vertex2);
  assert(vertex1 != vertex3);
  assert(vertex2 != vertex3);
  Index new_index;
  if (deleted_triangles_.size() > 0) {
    new_index = deleted_triangles_.top();
    deleted_triangles_.pop();
    triangles_[new_index] = Triangle();
  } else {
    new_index = triangles_.size();
    triangles_.push_back(Triangle());
  }

  Triangle triangle = Triangle(vertex1, vertex2, vertex3);
  n_valid_triangles_++;

  triangle.plane_ =
      Plane(vertices_[vertex1].position_, vertices_[vertex2].position_,
            vertices_[vertex3].position_);
  triangle.edges_[0] = registerEdge(vertex1, vertex2, new_index);
  triangle.edges_[1] = registerEdge(vertex2, vertex3, new_index);
  triangle.edges_[2] = registerEdge(vertex3, vertex1, new_index);
  triangles_[new_index] = triangle;

  vertices_[vertex1].triangles_.push_back(new_index);
  vertices_[vertex2].triangles_.push_back(new_index);
  vertices_[vertex3].triangles_.push_back(new_index);

  return new_index;
}

void Mesh::removeVertex(Index vertex) {
  while (vertices_[vertex].triangles_.size() > 0) {
    removeTriangle(vertices_[vertex].triangles_[0]);
  }

  deleted_vertices_.push(vertex);
  vertices_[vertex] = Vertex();
  n_valid_vertices_--;
  dirty_ = true;
}

void Mesh::removeTriangle(Index triangle) {
  for (unsigned int i = 0; i < 3; ++i) {
    Vertex& vertex = vertices_[triangles_[triangle].vertices_[i]];
    for (unsigned int j = 0; j < vertex.triangles_.size(); ++j) {
      if (vertex.triangles_[j] == triangle) {
        vertex.triangles_.erase(vertex.triangles_.begin() + j);
        break;
      }
    }
  }
  for (unsigned int i = 0; i < 3; ++i) {
    Index edge_index = triangles_[triangle].edges_[i];
    if (edges_[edge_index].border()) {
      // Remove edge
      Vertex& vertex1 = vertices_[edges_[edge_index].vertex(0)];
      for (unsigned int j = 0; j < vertex1.nEdges(); ++j) {
        if (vertex1.edge(j) == edge_index) {
          vertex1.edges_.erase(vertex1.edges_.begin() + j);
          break;
        }
      }
      Vertex& vertex2 = vertices_[edges_[edge_index].vertex(1)];
      for (unsigned int j = 0; j < vertex2.nEdges(); ++j) {
        if (vertex2.edge(j) == edge_index) {
          vertex2.edges_.erase(vertex2.edges_.begin() + j);
          break;
        }
      }
      // std::cout<<"Removing edge "<<edge_index<<std::endl;
      deleted_edges_.push(edge_index);
      edges_[edge_index] = Edge();
      n_valid_edges_--;
    } else {
      Edge& edge = edges_[edge_index];
      for (unsigned int j = 0; j < edge.nTriangles(); ++j) {
        if (edge.triangle(j) == triangle) {
          edge.triangles_.erase(edge.triangles_.begin() + j);
          break;
        }
      }
    }
  }

  triangles_[triangle] = Triangle();
  deleted_triangles_.push(triangle);
  n_valid_triangles_--;
  dirty_ = true;
}

void Mesh::clean() {
  Mesh empty;
  swap(*this, empty);
}

Index Mesh::registerEdge(Index vertex1, Index vertex2, Index triangle) {
  assert(vertex1 != vertex2);
  assert(vertex1 < vertices_.size());
  assert(vertex2 < vertices_.size());
  assert(triangle < triangles_.size());
  // Adds edge between vertex1 and vertex2 belonging to triangle.
  // returns index of edge
  // for easy comparison, all edges satisfy egde.vertices[0] < egde.vertices[1]
  Index first_vertex = vertex1;
  Index second_vertex = vertex2;
  if (vertex1 > vertex2) {
    first_vertex = vertex2;
    second_vertex = vertex1;
  }
  bool already_exists = false;
  Index existing_edge_index = 0;
  for (unsigned int i = 0; i < vertices_[first_vertex].nEdges(); ++i) {
    Index edge_index = vertices_[first_vertex].edge(i);
    if (edges_[edge_index].vertex(1) == second_vertex) {
      already_exists = true;
      existing_edge_index = edge_index;
      break;
    }
  }
  Index new_index;
  if (!already_exists) {
    Edge edge(first_vertex, second_vertex);
    edge.triangles_.push_back(triangle);
    if (deleted_edges_.size() > 0) {
      new_index = deleted_edges_.top();
      deleted_edges_.pop();
      edges_[new_index] = edge;
    } else {
      new_index = edges_.size();
      edges_.push_back(edge);
    }

    n_valid_edges_++;

    vertices_[vertex1].edges_.push_back(new_index);
    vertices_[vertex2].edges_.push_back(new_index);
  } else {
    new_index = existing_edge_index;
    edges_[existing_edge_index].triangles_.push_back(triangle);
  }

  return new_index;
}

// Closes all holes within the given mesh. This is achieved by introducing a new
// vertex in the center of
// each hole and then adding new faces between each edge and that newly created
// vertex
void Mesh::closeHoles() {
  // Find edges that only have one adjacent triangle (are next to a hole)
  bool inf = true;
  bool done = true;
  while (inf && done) {
    inf = false;
    done = false;
    for (int i = 0; i < edges_.size(); i++) {
      if (edges_[i].triangles_.size() == 1) {
        // Determine all edges that are adjacent to the current whole by
        // searching along the hole
        // This is achieved by following along the edges of the hole until the
        // starting vertex
        // is reached again
        int startVertex = edges_[i].vertices_[0];
        int lastVertex = startVertex;
        int lastEdge = i;
        std::vector<Index> currentHoleEdges;
        bool quit = false;
        while (!quit) {
          Vertex v = vertices_[lastVertex];
          for (int j = 0; j < v.edges_.size(); j++) {
            if (edges_[v.edges_[j]].triangles_.size() == 1 &&
                v.edges_[j] != lastEdge) {
              lastEdge = v.edges_[j];
              currentHoleEdges.push_back(lastEdge);
              if (lastVertex == edges_[lastEdge].vertices_[0]) {
                lastVertex = edges_[lastEdge].vertices_[1];
              } else {
                lastVertex = edges_[lastEdge].vertices_[0];
              }
              if (startVertex == lastVertex) {
                quit = true;
              }
              break;
            }
          }
          if (currentHoleEdges.size() > 10000) {
            inf = true;
            break;
          }
        }
        if (inf) {
          continue;
        }

        // Calculate the center point of all edges of the whole
        Vector3 mean = Vector3::Zero();
        for (int j = 0; j < currentHoleEdges.size(); j++) {
          for (int k = 0; k < 2; k++) {
            mean +=
                vertices_[edges_[currentHoleEdges[j]].vertices_[k]].position_;
          }
        }
        mean /= 2.0 * currentHoleEdges.size();
        Index centerVertex = addVertex(mean);

        // Fill the whole by adding the necessary triangles
        for (int j = 0; j < currentHoleEdges.size(); j++) {
          addTriangle(edges_[currentHoleEdges[j]].vertices_[0],
                      edges_[currentHoleEdges[j]].vertices_[1], centerVertex);
        }
        std::cout << "Filled hole with " << currentHoleEdges.size()
                  << " vertices" << std::endl;
        done = true;
      }
    }
  }
}

void Mesh::cleanAndRenumber() {
  // Remove all deleted vertices/triangles/edges
  // Empty the deleted-stack and sort for convenient skipping later
  // Copy object, then propagate new index to all referencing objects

  // Clean vertices
  std::deque<Vertex> new_vertices;
  std::vector<Index> deleted_vertices_sorted;
  deleted_vertices_sorted.reserve(deleted_vertices_.size());
  while (deleted_vertices_.size() > 0) {
    deleted_vertices_sorted.push_back(deleted_vertices_.top());
    deleted_vertices_.pop();
  }
  std::sort(deleted_vertices_sorted.begin(), deleted_vertices_sorted.end());
  Index next_index = 0;
  unsigned int deleted_i = 0;
  for (unsigned int i = 0; i < n_valid_vertices_; ++i) {
    while (deleted_i < deleted_vertices_sorted.size() &&
           next_index == deleted_vertices_sorted[deleted_i]) {
      next_index++;
      deleted_i++;
    }
    Vertex& vertex = vertices_[next_index];
    Index new_index = new_vertices.size();

    for (unsigned int j = 0; j < vertex.nEdges(); ++j) {
      Edge& edge = edges_[vertex.edge(j)];
      for (unsigned int k = 0; k < 2; k++) {
        if (edge.vertex(k) == next_index) {
          edge.vertices_[k] = new_index;
          break;
        }
      }
    }
    for (unsigned int j = 0; j < vertex.nTriangles(); ++j) {
      Triangle& triangle = triangles_[vertex.triangle(j)];
      for (unsigned int k = 0; k < 3; k++)
        if (triangle.vertex(k) == next_index) {
          triangle.vertices_[k] = new_index;
          break;
        }
    }
    new_vertices.push_back(vertex);
    next_index++;
  }
  vertices_.swap(new_vertices);

  // Clean Triangles
  std::deque<Triangle> new_triangles;
  std::vector<Index> deleted_triangles_sorted;
  deleted_triangles_sorted.reserve(deleted_triangles_.size());
  while (deleted_triangles_.size() > 0) {
    deleted_triangles_sorted.push_back(deleted_triangles_.top());
    deleted_triangles_.pop();
  }
  std::sort(deleted_triangles_sorted.begin(), deleted_triangles_sorted.end());
  next_index = 0;
  deleted_i = 0;
  for (unsigned int i = 0; i < n_valid_triangles_; ++i) {
    while (deleted_i < deleted_triangles_sorted.size() &&
           next_index == deleted_triangles_sorted[deleted_i]) {
      next_index++;
      deleted_i++;
    }

    Triangle& triangle = triangles_[next_index];
    Index new_index = new_triangles.size();

    for (unsigned int j = 0; j < 3; ++j) {
      if (triangle.vertex(j) >= vertices_.size()) {
        std::cout << "found invalid vertex" << triangle.vertex(j) << " ("
                  << vertices_.size() << " total) while cleaning triangle " << i
                  << std::endl;
        continue;
      }
      Vertex& vertex = vertices_[triangle.vertex(j)];
      for (unsigned int k = 0; k < vertex.nTriangles(); ++k) {
        if (vertex.triangle(k) == next_index) {
          vertex.triangles_[k] = new_index;
          break;
        }
      }
    }
    for (unsigned int j = 0; j < 3; ++j) {
      Edge& edge = edges_[triangle.edge(j)];
      for (unsigned int k = 0; k < edge.nTriangles(); ++k) {
        if (edge.triangle(k) == next_index) {
          edge.triangles_[k] = new_index;
          break;
        }
      }
    }

    new_triangles.push_back(triangle);
    next_index++;
  }
  triangles_.swap(new_triangles);

  // Clean edges
  std::deque<Edge> new_edges;
  std::vector<Index> deleted_edges_sorted;
  deleted_edges_sorted.reserve(deleted_edges_.size());
  while (deleted_edges_.size() > 0) {
    deleted_edges_sorted.push_back(deleted_edges_.top());
    deleted_edges_.pop();
  }
  std::sort(deleted_edges_sorted.begin(), deleted_edges_sorted.end());
  next_index = 0;
  deleted_i = 0;
  for (unsigned int i = 0; i < n_valid_edges_; ++i) {
    while (deleted_i < deleted_edges_sorted.size() &&
           next_index == deleted_edges_sorted[deleted_i]) {
      next_index++;
      deleted_i++;
    }

    Edge& edge = edges_[next_index];
    Index new_index = new_edges.size();

    for (unsigned int j = 0; j < 2; ++j) {
      if (edge.vertex(j) >= vertices_.size()) {
        std::cout << "found invalid vertex " << edge.vertex(j) << " ("
                  << vertices_.size() << " total) while cleaning edge " << i
                  << std::endl;
        continue;
      }
      Vertex& vertex = vertices_[edge.vertex(j)];
      for (unsigned int k = 0; k < vertex.nEdges(); ++k)
        if (vertex.edge(k) == next_index) {
          vertex.edges_[k] = new_index;
          break;
        }
    }
    for (unsigned int j = 0; j < edge.nTriangles(); ++j) {
      if (edge.triangle(j) >= triangles_.size()) {
        std::cout << "found invalid triangle " << edge.triangle(j) << " ("
                  << triangles_.size() << " total) while cleaning edge  " << i
                  << std::endl;
        continue;
      }
      Triangle& triangle = triangles_[edge.triangle(j)];
      for (int k = 0; k < 3; ++k) {
        if (triangle.edge(k) == next_index) {
          triangle.edges_[k] = new_index;
          break;
        }
      }
    }

    new_edges.push_back(edge);
    next_index++;
  }
  edges_.swap(new_edges);

  dirty_ = false;
}

bool Mesh::isDirty() { return dirty_; }

void Mesh::setDebugData(std::string debug_string) { debug_vrml = debug_string; }

Vector3 Mesh::HSVtoRGB(Vector3 color) {
  Real hh, p, q, t, ff;
  int i;
  Vector3 out;

  if (color(1) <= 0.0) {
    out = Vector3(0, 0, 0);
    return out;
  }
  hh = color(0);
  hh *= 6;
  i = (int)hh;
  ff = hh - i;
  p = color(2) * (1.0 - color(1));
  q = color(2) * (1.0 - (color(1) * ff));
  t = color(2) * (1.0 - (color(1) * (1.0 - ff)));

  switch (i) {
    case 0:
      out(0) = color(2);
      out(1) = t;
      out(2) = p;
      break;
    case 1:
      out(0) = q;
      out(1) = color(2);
      out(2) = p;
      break;
    case 2:
      out(0) = p;
      out(1) = color(2);
      out(2) = t;
      break;

    case 3:
      out(0) = p;
      out(1) = q;
      out(2) = color(2);
      break;
    case 4:
      out(0) = t;
      out(1) = p;
      out(2) = color(2);
      break;
    case 5:
    default:
      out(0) = color(2);
      out(1) = p;
      out(2) = q;
      break;
  }
  return out;
}

std::vector<Vector3> Mesh::generateColors(int n) {
  std::vector<Vector3> colors;
  Real golden_ratio_conjugate = 0.618033988749895;
  Real h = 0;
  for (int i = 0; i < n; ++i) {
    Vector3 color_hsv(h, 0.6, 0.95);
    colors.push_back(HSVtoRGB(color_hsv));
    h += golden_ratio_conjugate;
    if (h > 1) h -= 1;
  }
  return colors;
}

// Calculate the volume of the mesh if it is convex
Real Mesh::calculateConvexVolume() {
  Real totalVolume = 0.0;

  // Calculate center point of Mesh
  Vector3 mean = Vector3::Zero();
  for (int i = 0; i < vertices_.size(); i++) {
    mean += vertices_[i].position_;
  }
  mean /= vertices_.size();

  // For each facet determine the tetrahedron with the center point and
  // calculate its volume
  // and add it to the total volume
  for (int i = 0; i < triangles_.size(); i++) {
    Eigen::Matrix<Real, 4, 4> mat;
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        mat(j, k) = vertices_[triangles_[i].vertices_[j]].position_[k];
      }
    }
    for (int j = 0; j < 3; j++) {
      mat(3, j) = mean[j];
    }
    for (int j = 0; j < 4; j++) {
      mat(j, 3) = 1;
    }
    totalVolume += mat.determinant() / 6.0;
  }

  return totalVolume;
}

Real Mesh::getBoundingBoxDiagonal() {
  if (nVertices() > 0) {
    Vector3 min = vertex(0).position(), max = vertex(0).position();
    for (int i = 0; i < nVertices(); ++i) {
      Vector3 v = vertex(i).position();
      min = min.cwiseMin(v);
      max = max.cwiseMax(v);
    }
    return (min - max).norm();
  } else {
    return 0;
  }
}

Convex::Convex() : mesh(NULL), volume(0) {}

Convex::Convex(const Convex& other) : mesh(other.mesh), volume(other.volume) {}

Convex::Convex(const std::vector<Vector3>& points) {
  std::vector<Point_3> cgal_points;
  for (unsigned int i = 0; i < points.size(); ++i) {
    cgal_points.push_back(Point_3(points[i](0), points[i](1), points[i](2)));
  }

  Polyhedron_3 poly;
  CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

  mesh = std::make_shared<Mesh>();
  VertexPositionSet vertices(mesh.get());

  for (Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
    Halfedge_facet_circulator j = i->facet_begin();
    Index indices[3];
    unsigned int vertex_index = 0;
    do {
      Vector3 position;
      position(0) = j->vertex()->point().x();
      position(1) = j->vertex()->point().y();
      position(2) = j->vertex()->point().z();
      indices[vertex_index] = vertices.addVertex(position);
      vertex_index++;
    } while (++j != i->facet_begin());

    if (indices[0] == indices[1] || indices[0] == indices[2] ||
        indices[1] == indices[2]) {
      std::cout << "Borked indices in convex hull computation " << indices[0]
                << " " << indices[1] << " " << indices[2] << std::endl;
      continue;
    }

    mesh->addTriangle(indices[0], indices[1], indices[2]);
  }

  volume = mesh->calculateConvexVolume();
}

Convex::~Convex() {}

Convex& Convex::operator=(Convex other) {
  swap(*this, other);
  return *this;
}

void swap(Convex& first, Convex& second) {
  first.mesh.swap(second.mesh);
  std::swap(first.volume, second.volume);
}

inline const double ComputeVolume4(const Vector3& a, const Vector3& b,
                                   const Vector3& c, const Vector3& d) {
  return (a - d).dot((b - d).cross(c - d));
}

double Convex::ComputeVolume() {
  if (!mesh) {
    return 0.0;
  }

  Vector3 bary = Vector3::Zero();
  for (int v = 0; v < mesh->nVertices(); v++) {
    bary += mesh->vertex(v).position();
  }
  bary /= static_cast<double>(mesh->nVertices());

  double totalVolume = 0.0;
  for (int t = 0; t < mesh->nTriangles(); t++) {
    const Triangle& tri = mesh->triangle(t);
    totalVolume += ComputeVolume4(mesh->vertex(tri.vertex(0)).position(),
                                  mesh->vertex(tri.vertex(1)).position(),
                                  mesh->vertex(tri.vertex(2)).position(), bary);
  }
  return totalVolume / 6.0;
}
}
