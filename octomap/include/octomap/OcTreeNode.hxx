/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bitset>
#include <cassert>
#include <math.h>
#include <fstream>
#include <stdlib.h>

#include <octomap/OcTreeNode.h>

namespace octomap {

  template <bool COPY_ON_WRITE>
  OcTreeNode<COPY_ON_WRITE>::OcTreeNode()
    : OcTreeDataNode<float, COPY_ON_WRITE>(0.0)
  {
  }

  template <bool COPY_ON_WRITE>
  OcTreeNode<COPY_ON_WRITE>::~OcTreeNode(){
  }

  // TODO: Use Curiously Recurring Template Pattern instead of copying full function
  // (same for getChild)
  template <bool COPY_ON_WRITE>
  bool OcTreeNode<COPY_ON_WRITE>::createChild(unsigned int i) {
    assert (this->unique());
    if (children == NULL) {
      allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new OcTreeNode<COPY_ON_WRITE>();
    return true;
  }

  // ============================================================
  // =  occupancy probability  ==================================
  // ============================================================

  template <bool COPY_ON_WRITE>
  double OcTreeNode<COPY_ON_WRITE>::getMeanChildLogOdds() const{
    double mean = 0;
    char c = 0;
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        mean += getChild(i)->getOccupancy();
        c++;
      }
    }
    if (c)
      mean /= (double) c;

    return log(mean/(1-mean));
  }

  template <bool COPY_ON_WRITE>
  float OcTreeNode<COPY_ON_WRITE>::getMaxChildLogOdds() const{
    float max = -std::numeric_limits<float>::max();
    for (unsigned int i=0; i<8; i++) {
      if (childExists(i)) {
        float l = getChild(i)->getLogOdds();
        if (l > max)
          max = l;
      }
    }
    return max;
  }

  template <bool COPY_ON_WRITE>
  void OcTreeNode<COPY_ON_WRITE>::addValue(const float& logOdds) {
    assert (this->unique());
    value += logOdds;
  }
  
} // end namespace


