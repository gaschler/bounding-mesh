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

#ifndef BOUNDINGMESH_CONTRACTIONUTILS_H
#define BOUNDINGMESH_CONTRACTIONUTILS_H

#include <set>
#include "Primitives.h"

namespace boundingmesh {
class EdgeContraction {
 public:
  EdgeContraction();
  EdgeContraction(Index edge, Vector3 new_point, Real cost,
                  const Matrix44& qem);

  Index edge() const;
  Vector3 new_point() const;
  Real cost() const;
  const Matrix44& qem() const;

  friend bool operator<(const EdgeContraction& lhs, const EdgeContraction& rhs);
  friend bool operator>(const EdgeContraction& lhs, const EdgeContraction& rhs);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Index edge_;
  Vector3 new_point_;
  Real cost_;
  Matrix44 qem_;
};

typedef std::multiset<EdgeContraction, std::less<EdgeContraction>,
                      Eigen::aligned_allocator<EdgeContraction> >::iterator
    ContractionIterator;

class ContractionIndex {
 public:
  ContractionIndex(Index index);
  ContractionIndex(Index index, ContractionIterator contraction);

  ContractionIterator contraction() const;

  friend bool operator<(const ContractionIndex& lhs,
                        const ContractionIndex& rhs);
  friend bool operator>(const ContractionIndex& lhs,
                        const ContractionIndex& rhs);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Index index_;
  ContractionIterator contraction_;
  bool searching_;
};

class ContractionQueue {
 public:
  ContractionQueue();
  unsigned int size();
  const EdgeContraction& first();
  void insert(const EdgeContraction& contraction);
  void remove(Index edge);

  friend void swap(ContractionQueue& first, ContractionQueue& second);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  std::multiset<EdgeContraction, std::less<EdgeContraction>,
                Eigen::aligned_allocator<EdgeContraction> >
      contractions_;
  std::set<ContractionIndex> indices_;

  friend class Decimator;
};
}

#endif  // BOUNDINGMESH_CONTRACTIONUTILS_H
