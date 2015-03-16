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

#ifndef OCTOMAP_COW_POLICY_H
#define OCTOMAP_COW_POLICY_H


namespace octomap {

  template <bool COPY_ON_WRITE>
  class OcTreeCoWPolicy
  {
  protected:
    //// A no-op for standard OcTrees.
    template <typename T>
    void makeUnique(T **child) const;

    /// Deletes the given child.
    template <typename T>
    void derefChild(T *child) const;

    /// Makes @a root a deep copy of @a rhs_root.
    template <typename T>
    void deepCopy(T **root, T *rhs_root) const;
  };

  template <>
  class OcTreeCoWPolicy<true>
  {
  protected:
    /**
     * If the refcount is > 1, makes a unique copy if this node.  Its
     * children are lazily copied (their refcounts are increased).
     */
    template <typename T>
    void makeUnique(T **child) const;

    /// Dereferences the given child and deletes it if the refcount reaches 0.
    template <typename T>
    void derefChild(T *child) const;

    /// Makes @a root a lazy deep copy of @a rhs_root (i.e., just increases the refcount).
    template <typename T>
    void deepCopy(T **root, T *rhs_root) const;
  };

  template <bool COPY_ON_WRITE>
  class CoWPolicy : public OcTreeCoWPolicy<COPY_ON_WRITE>
  {
  public:
    /// Returns true.
    bool unique() const;

    /// Returns the refcount (always 1 for non-CoW nodes).
    unsigned int getRefcount() const { return 1; }

    /// Increases the refcount (no-op here).
    void ref();

    /// Decreases the refcount, returning true if the refcount has reached zero (no-op here).
    bool deref();

  protected:
    CoWPolicy() { }
  };

  template <>
  class CoWPolicy<true> : public OcTreeCoWPolicy<true>
  {
  public:
    /// Returns true if the refcount is 1.
    bool unique() const;

    /// Returns the refcount.
    unsigned int getRefcount() const { return refcount; }

    /// Increases the refcount.
    void ref();

    /// Decreases the refcount, returning true if the refcount has reached zero.
    bool deref();

  protected:
    CoWPolicy()
      : refcount(1)
    { }

    /// Reference count of this node (at least 1).
    unsigned int refcount;
  };

} // end namespace

#include "octomap/CoWPolicy.hxx"

#endif
