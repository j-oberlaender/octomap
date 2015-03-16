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

#ifndef OCTOMAP_OCTREE_STAMPED_H
#define OCTOMAP_OCTREE_STAMPED_H


#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ctime>

namespace octomap {
  
  // node definition
  template <bool COPY_ON_WRITE=false>
  class OcTreeNodeStamped : public OcTreeNode<COPY_ON_WRITE> {    

  public:
    OcTreeNodeStamped() : OcTreeNode<COPY_ON_WRITE>(), timestamp(0) {}

    OcTreeNodeStamped(const OcTreeNodeStamped& rhs) : OcTreeNode<COPY_ON_WRITE>(rhs), timestamp(rhs.timestamp) {}

    bool operator==(const OcTreeNodeStamped& rhs) const{
      return (rhs.value == value && rhs.timestamp == timestamp);
    }
    
    // children
    inline OcTreeNodeStamped* getChild(unsigned int i) {
      return static_cast<OcTreeNodeStamped*> (OcTreeNode<COPY_ON_WRITE>::getChild(i));
    }
    inline const OcTreeNodeStamped* getConstChild(unsigned int i) const {
      return static_cast<const OcTreeNodeStamped*> (OcTreeNode<COPY_ON_WRITE>::getConstChild(i));
    }
    inline const OcTreeNodeStamped* getChild(unsigned int i) const {
      return getConstChild(i);
    }

    bool createChild(unsigned int i) {
      assert (this->unique());
      if (children == NULL) allocChildren();
      children[i] = new OcTreeNodeStamped();
      return true;
    }
    
    // timestamp
    inline unsigned int getTimestamp() const { return timestamp; }
    inline void updateTimestamp() { timestamp = (unsigned int) time(NULL);}
    inline void setTimestamp(unsigned int timestamp) {this->timestamp = timestamp; }

    // update occupancy and timesteps of inner nodes 
    inline void updateOccupancyChildren() {      
      assert (this->unique());
      this->setLogOdds(this->getMaxChildLogOdds());  // conservative
      updateTimestamp();
    }

    using OcTreeNode<COPY_ON_WRITE>::childExists;

  protected:
    using OcTreeNode<COPY_ON_WRITE>::value;
    using OcTreeNode<COPY_ON_WRITE>::children;
    using OcTreeNode<COPY_ON_WRITE>::allocChildren;

    unsigned int timestamp;
  };


  // tree definition
  template <bool COPY_ON_WRITE=false>
  class OcTreeStamped : public OccupancyOcTreeBase <OcTreeNodeStamped<COPY_ON_WRITE> > {    

  public:
    /// Default constructor, sets resolution of leafs
    OcTreeStamped(double resolution) : OccupancyOcTreeBase<OcTreeNodeStamped<COPY_ON_WRITE> >(resolution) {};    
      
    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    OcTreeStamped<COPY_ON_WRITE>* create() const {return new OcTreeStamped<COPY_ON_WRITE>(this->resolution); }

    std::string getTreeType() const {return "OcTreeStamped";}

    //! \return timestamp of last update
    unsigned int getLastUpdateTime();

    void degradeOutdatedNodes(unsigned int time_thres);
    
    virtual void updateNodeLogOdds(OcTreeNodeStamped<COPY_ON_WRITE>* node, const float& update) const;
    void integrateMissNoTime(OcTreeNodeStamped<COPY_ON_WRITE>* node) const;
  };

  namespace {

    class OcTreeStampedStaticInit : public AbstractOcTree{
    protected:
      /**
       * Static member object which ensures that this OcTree's prototype
       * ends up in the classIDMapping only once
       */
      class StaticMemberInitializer{
      public:
        StaticMemberInitializer() {
          OcTreeStamped<false>* ftree = new OcTreeStamped<false>(0.1);
          OcTreeStamped<true>*  ttree = new OcTreeStamped<true>(0.1);
          AbstractOcTree::registerTreeType(false, ftree);
          AbstractOcTree::registerTreeType(true,  ttree);
        }
      };
      /// to ensure static initialization (only once)
      static StaticMemberInitializer ocTreeMemberInit;
    };

  }

} // end namespace

#include "octomap/OcTreeStamped.hxx"

#endif
