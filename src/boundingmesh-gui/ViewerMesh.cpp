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

#include "ViewerMesh.h"

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoNormalBinding.h>
#include <Inventor/nodes/SoShapeHints.h>

void ViewerMesh::setWireframe(bool wireframe) { wireframe_ = wireframe; }

void ViewerMesh::replaceByMesh(::std::shared_ptr<boundingmesh::Mesh> mesh) {
  delete[] norms;
  delete[] num_vertices;
  delete[] vertices;
  norms = new float[mesh->nTriangles()][3];
  num_vertices = new int[mesh->nTriangles()];
  vertices = new float[mesh->nTriangles() * 3][3];

  for (unsigned int i = 0; i < mesh->nTriangles(); ++i) {
    boundingmesh::Triangle t = mesh->triangle(i);
    boundingmesh::Vector3 normal = t.plane().normal;
    num_vertices[i] = 3;
    for (int j = 0; j < 3; ++j) {
      norms[i][j] = normal(j);
      boundingmesh::Vector3 v = mesh->vertex(t.vertex(j)).position();
      for (int k = 0; k < 3; ++k) {
        vertices[3 * i + j][k] = v(k);
      }
    }
  }

  model_root->removeAllChildren();

  SoDrawStyle *model_draw_style = new SoDrawStyle;
  if (wireframe_) {
    model_draw_style->style = SoDrawStyleElement::LINES;
    model_draw_style->lineWidth = 1.3f;
  }

  /*SoNormal *model_normals = new SoNormal;
  model_normals->vector.setValues(0, mesh->nTriangles(), norms);*/

  SoCoordinate3 *model_coords = new SoCoordinate3;
  model_coords->point.setValues(0, mesh->nTriangles() * 3, vertices);
  SoFaceSet *model_face_set = new SoFaceSet;
  model_face_set->numVertices.setValues(0, mesh->nTriangles(), num_vertices);

  /*SoNormalBinding *model_normal_binding = new SoNormalBinding;
  model_normal_binding->value = SoNormalBinding::PER_FACE;*/

  SoShapeHints *model_hint = new SoShapeHints;
  model_hint->vertexOrdering = SoShapeHintsElement::COUNTERCLOCKWISE;

  model_root->addChild(model_draw_style);
  model_root->addChild(model_hint);
  // model_root->addChild(model_normal_binding);
  // model_root->addChild(model_normals);
  model_root->addChild(model_coords);
  model_root->addChild(model_face_set);
}
