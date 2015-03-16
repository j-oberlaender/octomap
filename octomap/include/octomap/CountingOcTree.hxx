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

#include <cassert>
#include <octomap/CountingOcTree.h>

namespace octomap {


  /// implementation of CountingOcTreeNode  ----------------------------------

  template <bool COPY_ON_WRITE>
  CountingOcTreeNode<COPY_ON_WRITE>::CountingOcTreeNode()
    : OcTreeDataNode<unsigned int, COPY_ON_WRITE>(0)
  {
  }

  template <bool COPY_ON_WRITE>
  CountingOcTreeNode<COPY_ON_WRITE>::~CountingOcTreeNode() {

  }

  template <bool COPY_ON_WRITE>
  void CountingOcTreeNode<COPY_ON_WRITE>::expandNode(){
    assert(!this->hasChildren());
    assert(this->unique());

    // divide "counts" evenly to children
    unsigned int childCount = (unsigned int)(this->value/ 8.0 +0.5);
    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      this->children[k]->setValue(childCount);
    }
  }

  template <bool COPY_ON_WRITE>
  bool CountingOcTreeNode<COPY_ON_WRITE>::createChild(unsigned int i) {
    assert (this->unique());
    if (this->children == NULL) {
      this->allocChildren();
    }
    assert (this->children[i] == NULL);
    this->children[i] = new CountingOcTreeNode<COPY_ON_WRITE>();
    return true;
  }


  /// implementation of CountingOcTree  --------------------------------------


  template <bool COPY_ON_WRITE>
  CountingOcTreeNode<COPY_ON_WRITE>* CountingOcTree<COPY_ON_WRITE>::updateNode(const point3d& value) {

    OcTreeKey key;
    if (!this->coordToKeyChecked(value, key)) return NULL;
    return updateNode(key);
  }


  // Note: do not inline this method, will decrease speed (KMW)
  template <bool COPY_ON_WRITE>
  CountingOcTreeNode<COPY_ON_WRITE>* CountingOcTree<COPY_ON_WRITE>::updateNode(const OcTreeKey& k) {

    this->makeRootUnique();
    CountingOcTreeNode<COPY_ON_WRITE>* curNode (this->root);
    curNode->increaseCount();


    // follow or construct nodes down to last level...
    for (int i=(this->tree_depth-1); i>=0; i--) {

      unsigned int pos = computeChildIdx(k, i);

      // requested node does not exist
      if (!curNode->childExists(pos)) {
        curNode->createChild(pos);
        this->tree_size++;
      }
      // descent tree
      curNode->makeUnique(pos);
      curNode = static_cast<CountingOcTreeNode<COPY_ON_WRITE>*> (curNode->getChild(pos));
      curNode->increaseCount(); // modify traversed nodes
    }

    return curNode;
  }


  template <bool COPY_ON_WRITE>
  void CountingOcTree<COPY_ON_WRITE>::getCentersMinHits(point3d_list& node_centers, unsigned int min_hits) const {

    OcTreeKey root_key;
    root_key[0] = root_key[1] = root_key[2] = this->tree_max_val;
    getCentersMinHitsRecurs(node_centers, min_hits, this->tree_depth, this->root, 0, root_key);
  }


  template <bool COPY_ON_WRITE>
  void CountingOcTree<COPY_ON_WRITE>::getCentersMinHitsRecurs( point3d_list& node_centers,
                                                               unsigned int& min_hits,
                                                               unsigned int max_depth,
                                                               CountingOcTreeNode<COPY_ON_WRITE>* node, unsigned int depth,
                                                               const OcTreeKey& parent_key) const {

    if (depth < max_depth && node->hasChildren()) {

      unsigned short int center_offset_key = this->tree_max_val >> (depth + 1);
      OcTreeKey search_key;

      for (unsigned int i=0; i<8; ++i) {
        if (node->childExists(i)) {
          computeChildKey(i, center_offset_key, parent_key, search_key);
          getCentersMinHitsRecurs(node_centers, min_hits, max_depth, node->getChild(i), depth+1, search_key);
        }
      }
    }

    else { // max level reached

      if (node->getCount() >= min_hits) {
        node_centers.push_back(this->keyToCoord(parent_key, depth));
      }
    }
  }


} // namespace
