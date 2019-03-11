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

#include "ContractionUtils.h"
#include <cassert>

namespace boundingmesh {
// Implementation of class EdgeContraction
EdgeContraction::EdgeContraction()
    : edge_(0), new_point_(Vector3::Zero()), cost_(0), qem_(Matrix44::Zero()) {}

EdgeContraction::EdgeContraction(Index edge, Vector3 new_point, Real cost,
                                 const Matrix44& qem)
    : edge_(edge), new_point_(new_point), cost_(cost), qem_(qem) {}

Index EdgeContraction::edge() const { return edge_; }

Vector3 EdgeContraction::new_point() const { return new_point_; }

Real EdgeContraction::cost() const { return cost_; }

const Matrix44& EdgeContraction::qem() const { return qem_; }

bool operator<(const EdgeContraction& lhs, const EdgeContraction& rhs) {
  return lhs.cost_ < rhs.cost_;
}

bool operator>(const EdgeContraction& lhs, const EdgeContraction& rhs) {
  return operator<(rhs, lhs);
}

// Implementation of class ContractionIndex
ContractionIndex::ContractionIndex(Index index)
    : index_(index), contraction_(), searching_(true) {}

ContractionIndex::ContractionIndex(Index index, ContractionIterator contraction)
    : index_(index), contraction_(contraction), searching_(false) {}

ContractionIterator ContractionIndex::contraction() const {
  return contraction_;
}

bool operator<(const ContractionIndex& lhs, const ContractionIndex& rhs) {
  return lhs.index_ < rhs.index_;
}
bool operator>(const ContractionIndex& lhs, const ContractionIndex& rhs) {
  return operator<(rhs, lhs);
}

// Implementation of class ContractionQueue
ContractionQueue::ContractionQueue() : contractions_(), indices_() {}

unsigned int ContractionQueue::size() { return contractions_.size(); }

const EdgeContraction& ContractionQueue::first() {
  return *(contractions_.begin());
}

void ContractionQueue::insert(const EdgeContraction& contraction) {
  ContractionIterator contraction_it = contractions_.insert(contraction);
  indices_.insert(ContractionIndex(contraction.edge(), contraction_it));
}

void ContractionQueue::remove(Index index) {
  std::set<ContractionIndex>::iterator index_it =
      indices_.find(ContractionIndex(index));
  assert(index_it != indices_.end());
  ContractionIterator contraction_it = index_it->contraction();
  contractions_.erase(contraction_it);
  indices_.erase(index_it);
}

void swap(ContractionQueue& first, ContractionQueue& second) {
  first.contractions_.swap(second.contractions_);
  first.indices_.swap(second.indices_);
}
}
