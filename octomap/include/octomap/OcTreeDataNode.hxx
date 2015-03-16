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

namespace octomap {

  template <typename T, bool COPY_ON_WRITE>
  OcTreeDataNode<T, COPY_ON_WRITE>::OcTreeDataNode()
   : children(NULL)
  {

  }

  template <typename T, bool COPY_ON_WRITE>
  OcTreeDataNode<T, COPY_ON_WRITE>::OcTreeDataNode(T initVal)
   : children(NULL), value(initVal)
  {

  }

  template <typename T, bool COPY_ON_WRITE>
  OcTreeDataNode<T, COPY_ON_WRITE>::OcTreeDataNode(const OcTreeDataNode<T, COPY_ON_WRITE>& rhs)
   : children(NULL), value(rhs.value)
  {
    if (rhs.hasChildren()){
      allocChildren();
      for (unsigned i = 0; i<8; ++i){
        this->deepCopy(&children[i], rhs.children[i]);
      }
    }
  }


  template <typename T, bool COPY_ON_WRITE>
  OcTreeDataNode<T, COPY_ON_WRITE>::~OcTreeDataNode()
  {
    if (children != NULL) {
      for (unsigned int i=0; i<8; i++)
        derefChild(i);
      delete[] children;
    }

  }

  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::operator== (const OcTreeDataNode<T, COPY_ON_WRITE>& rhs) const{
    return rhs.value == value;
  }

  // ============================================================
  // =  children          =======================================
  // ============================================================

  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::createChild(unsigned int i) {
    if (children == NULL) {
      allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new OcTreeDataNode<T, COPY_ON_WRITE>();
    return true;
  }

  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::childExists(unsigned int i) const {
    assert(i < 8);
    if ((children != NULL) && (children[i] != NULL))
      return true;
    else
      return false;
  }

  template <typename T, bool COPY_ON_WRITE>
  void OcTreeDataNode<T, COPY_ON_WRITE>::deleteChild(unsigned int i) {
    assert((i < 8) && (children != NULL));
    assert(children[i] != NULL);
    derefChild(i);
    children[i] = NULL;
  }

  template <typename T, bool COPY_ON_WRITE>
  OcTreeDataNode<T, COPY_ON_WRITE>* OcTreeDataNode<T, COPY_ON_WRITE>::getChild(unsigned int i) {
    assert((i < 8) && (children != NULL));
    assert(children[i] != NULL);
    makeUnique(i);
    return children[i];
  }

  template <typename T, bool COPY_ON_WRITE>
  const OcTreeDataNode<T, COPY_ON_WRITE>* OcTreeDataNode<T, COPY_ON_WRITE>::getConstChild(unsigned int i) const {
    assert((i < 8) && (children != NULL));
    assert(children[i] != NULL);
    return children[i];
  }

  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::hasChildren() const {
    if (children == NULL)
      return false;
    for (unsigned int i = 0; i<8; i++){
      // fast check, we know children != NULL
      if (children[i] != NULL)
        return true;
    }
    return false;
  }

  // ============================================================
  // =  pruning           =======================================
  // ============================================================


  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::collapsible() const {
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!childExists(0) || getChild(0)->hasChildren())
      return false;

    for (unsigned int i = 1; i<8; i++) {
      // comparison via getChild so that casts of derived classes ensure
      // that the right == operator gets called
      if (!childExists(i) || getChild(i)->hasChildren() || !(*(getChild(i)) == *(getChild(0))))
        return false;
    }
    return true;
  }

  template <typename T, bool COPY_ON_WRITE>
  bool OcTreeDataNode<T, COPY_ON_WRITE>::pruneNode() {

    if (!this->collapsible())
      return false;

    // set value to children's values (all assumed equal)
    setValue(getChild(0)->getValue());

    // delete children
    for (unsigned int i=0;i<8;i++)
      derefChild(i);
    delete[] children;
    children = NULL;

    return true;
  }

  template <typename T, bool COPY_ON_WRITE>
  void OcTreeDataNode<T, COPY_ON_WRITE>::expandNode() {
    assert(!hasChildren());

    for (unsigned int k=0; k<8; k++) {
      createChild(k);
      children[k]->setValue(value);
    }
  }

  // ============================================================
  // =  File IO           =======================================
  // ============================================================

  template <typename T, bool COPY_ON_WRITE>
  std::istream& OcTreeDataNode<T, COPY_ON_WRITE>::readValue(std::istream &s) {

    char children_char;

    // read data:
    s.read((char*) &value, sizeof(value));
    s.read((char*)&children_char, sizeof(char));
    std::bitset<8> children ((unsigned long long) children_char);

    // std::cout << "read: " << value << " "
    //           << children.to_string<char,std::char_traits<char>,std::allocator<char> >()
    //           << std::endl;

    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1){
        createChild(i);
        getChild(i)->readValue(s);
      }
    }
    return s;
  }


  template <typename T, bool COPY_ON_WRITE>
  std::ostream& OcTreeDataNode<T, COPY_ON_WRITE>::writeValue(std::ostream &s) const{

    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;

    for (unsigned int i=0; i<8; i++) {
      if (childExists(i))
        children[i] = 1;
      else
        children[i] = 0;
    }

    char children_char = (char) children.to_ulong();
    s.write((const char*) &value, sizeof(value));
    s.write((char*)&children_char, sizeof(char));

    // std::cout << "wrote: " << value << " "
    //           << children.to_string<char,std::char_traits<char>,std::allocator<char> >() 
    //           << std::endl;

    // write children's children
    for (unsigned int i=0; i<8; i++) {
      if (children[i] == 1) {
        this->getChild(i)->writeValue(s);
      }
    }
    return s;
  }


  // ============================================================
  // =  private methodes  =======================================
  // ============================================================
  template <typename T, bool COPY_ON_WRITE>
  void OcTreeDataNode<T, COPY_ON_WRITE>::allocChildren() {
    children = new OcTreeDataNode<T, COPY_ON_WRITE>*[8];
    for (unsigned int i=0; i<8; i++) {
      children[i] = NULL;
    }
  }

  template <typename T, bool COPY_ON_WRITE>
  void OcTreeDataNode<T, COPY_ON_WRITE>::makeUnique(unsigned int i) {
    CoWPolicy<COPY_ON_WRITE>::makeUnique(&children[i]);
  }

  template <typename T, bool COPY_ON_WRITE>
  void OcTreeDataNode<T, COPY_ON_WRITE>::derefChild(unsigned int i) {
    CoWPolicy<COPY_ON_WRITE>::derefChild(children[i]);
  }

} // end namespace

