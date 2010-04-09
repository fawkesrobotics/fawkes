
/***************************************************************************
 *  hough_transform.cpp - Hough Transform
 *
 *  Created: Mon Dec 28 2009 18:56:04
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "hough_transform.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

/** @class HoughTransform "hough_transform.h"
 * Hough Transformation for N-dimensional representations.
 * This class implements a generic Hough transformation, which can operate
 * on representations of arbitrary dimension (at least in theory ignoring
 * computational feasibility).
 * The implementation uses a tree structure to represent the buckets in the
 * Hough space, to reduce the amount of memory required on sparse data
 * sets and to allow fast insertion of new samples.
 * The code is based on ideas from a Hough transform implemented in
 * FireVision, but eliminating some of its limitations.
 * @author Tim Niemueller
 *
 * @fn inline Node * HoughTransform::create_node(Node *parent, unsigned int dims, int value = 0)
 * Create a new node.
 * @param parent parent node of the new node
 * @param dims Dimensions remaining
 * @param value initial value
 * @return new node with given parent, dimensions, and initial value
 */

/** Constructor.
 * @param num_dims number of dimensions
 */
HoughTransform::HoughTransform(unsigned int num_dims)
{
  __root = new Node(this, num_dims);

  __num_dims   = num_dims;

  __reuse_head = new Node(this);
  __reuse_cur  = __reuse_head;
  __reuse_tail = __reuse_head;

  __max_count  = 0;
  __max_values = new int[num_dims];
}

/** Destructor. */
HoughTransform::~HoughTransform()
{
  while (__reuse_head) {
    Node *n = __reuse_head;
    __reuse_head = __reuse_head->__reuse_next;
    delete n;
  }

  delete[] __max_values;
}

/** Process some samples.
 * @param values two dimensional array of values. The first index determines
 * the sample index, the second index the dimension index. Thus its an
 * array with the length of number of values of arrays with the length of
 * the number of dimensions.
 * @param num_values number of rows in values
 */
void
HoughTransform::process(int **values, unsigned int num_values)
{
  for (unsigned int i = 0; i < num_values; ++i) {
    unsigned int count = __root->insert(values[i]);
    if (count > __max_count) {
      __max_count = count;
      for (unsigned int d = 0; d < __num_dims; ++d) {
	__max_values[d] = values[i][d];
      }
    }
  }
}

/** Get maximum values.
 * During processing the maximum values, i.e. the candidate with the
 * maximum number of votes or the most filled bucket, is stored and can
 * be retrieved with this method.
 * @param values upon return contains the maximum voted values
 * @return number of votes of the values
 */
unsigned int
HoughTransform::max(int *values) const
{
  for (unsigned int d = 0; d < __num_dims; ++d) {
    values[d] = __max_values[d];
  }
  return __max_count;
}


/** Filter values by number of votes.
 * This method filters all created buckets and returns only the ones which
 * have at least @p min_count votes
 * @param values upon return points to a newly allocated array of values with
 * the size of number of values * number of dimensions. The memory must be
 * freed when done by using free().
 * @param min_count minimum number of votes required to consider a bucket
 * @return number of values found
 */
unsigned int
HoughTransform::filter(int **values, unsigned int min_count)
{
  return __root->filter(values, min_count);
}


/** Get root node.
 * @return root node of internal tree, meant for debugging and performance
 * evaluation
 */
HoughTransform::Node *
HoughTransform::root()
{
  return __root;
}

/** Reset Hough transform.
 * This deletes the internal tree and creates a new empty one. */
void
HoughTransform::reset()
{
  __reuse_cur = __reuse_head;
  __root = create_node(NULL, __num_dims);
  __max_count = 0;
  for (unsigned int d = 0; d < __num_dims; ++d) {
    __max_values[d] = 0;
  }
}

/** @class HoughTransform::Node "hough_transform.h"
 * Hough transform tree node.
 * The nodes are used to form a tree. The tree is organized as stacked
 * binary trees. At a certain stack level, a value of a specific dimension
 * is stored, with the left and right sub-trees pointing to smaller or
 * higher values respectively.
 * Nodes with a stack level of 1 (e.g. the bottom-most level) have a field
 * to count the number of votes (these are the bucket nodes). Nodes on
 * higher levels have a pointer to another node on a stack level one lower
 * than the own, which represents the next dimension of the values.
 * @author Tim Niemueller
 * @author Hu Yuxiao
 */

/** Constructor.
 * @param dims number of remaining dimensions (including the own)
 * @param value the initial value of the node
 */
HoughTransform::Node::Node(HoughTransform *ht, unsigned int dims, int value)
{
  __ht     = ht;
  __dims   = dims;
  __value  = value;
  __count  = 0;
  __parent = NULL;
  __left = __right = __dim_next = __filter_next = __reuse_next = 0;
}

/** Constructor with parent node.
 * @param parent parent node of the new node
 * @param dims number of remaining dimensions (including the own)
 * @param value the initial value of the node
 */
HoughTransform::Node::Node(HoughTransform *ht,
			   Node *parent, unsigned int dims, int value)
{
  __ht     = ht;
  __parent = parent;
  __dims   = dims;
  __value  = value;
  __count  = 0;
  __left = __right = __dim_next = __filter_next = __reuse_next = 0;
}

/** Constructor. */
HoughTransform::Node::Node(HoughTransform *ht)
{
  __ht     = ht;
  __dims   = 1;
  __value  = 0;
  __count  = 0;
  __parent = NULL;
  __left = __right = __dim_next = __filter_next = __reuse_next = 0;
}

/** Destructor. */
HoughTransform::Node::~Node()
{
  // sub-nodes delete by HoughTransform
}


/** Insert new values.
 * @param values array with new values, must be of the size of the number
 * of dimensions
 * @return number of votes of bucket the values have been inserted to
 */
unsigned int
HoughTransform::Node::insert(int *values)
{
  if (values[0] == __value) {
    if ( __dims > 1) {
      if (! __dim_next) {
	__dim_next = __ht->create_node(this, __dims - 1, values[1]);
      }

      return __dim_next->insert(&(values[1]));
    } else {
      return ++__count;
    }
  } else if (values[0] < __value) {
    if (! __left) {
      __left = __ht->create_node(__parent, __dims, values[0]);
    }
    return __left->insert(values);
  } else { // values[0] > __value
    if (! __right) {
      __right = __ht->create_node(__parent, __dims, values[0]);
    }
    return __right->insert(values);
  }
}


/** Get number of nodes.
 * @return number of nodes
 */
unsigned int
HoughTransform::Node::num_nodes()
{
  unsigned int rv = 1;
  if (__left)     rv += __left->num_nodes();
  if (__right)    rv += __right->num_nodes();
  if (__dim_next) rv += __dim_next->num_nodes();
  return rv;
}


/** Depth of the tree.
 * @return maximum depth of tree
 */
unsigned int
HoughTransform::Node::depth()
{
  unsigned int d = 0;
  if (__left)     d = std::max(d, __left->depth());
  if (__right)    d = std::max(d, __right->depth());
  if (__dim_next) d = std::max(d, __dim_next->depth());
  return d + 1;
}


/** Get length of filtered list.
 * @return length of filtered list
 */
unsigned int
HoughTransform::Node::filtered_length()
{
  Node *t = this;
  // do not count first, is unused head element
  unsigned int rv = 0;
  while (t->__filter_next) {
    ++rv;
    t = t->__filter_next;
  }
  return rv;
}


/** Filter values by number of votes.
 * This method filters all created buckets and returns only the ones which
 * have at least @p min_count votes
 * @param values upon return points to a newly allocated array of values with the
 * size of number of values * number of dimensions. The memory must be freed
 * when done by using free().
 * @param min_count minimum number of votes required to consider a bucket
 * @return number of values found
 */
unsigned int
HoughTransform::Node::filter(int **values, unsigned int min_count)
{
  Node *filtered_root = new Node();
  filter(filtered_root, min_count);
  unsigned int flen = filtered_root->filtered_length();

  int *fvals = (int *)calloc(flen, __dims * sizeof(int));
  Node *t = filtered_root;
  unsigned int f = 0;
  while ((t = t->__filter_next) != NULL) {
    Node *s = t;
    for (unsigned int i = 1; i <= __dims; ++i) {
      fvals[ __dims * (f + 1) - i ] = s->__value;
      s = s->__parent;
    }
    ++f;
  }

  delete filtered_root;

  *values = fvals;
  return flen;
}


/** Internal filter recursion function.
 * @param tail current tail
 * @param min_count minimum number of votes required to consider a bucket
 * @return new tail node
 */
HoughTransform::Node *
HoughTransform::Node::filter(Node *tail, unsigned int min_count)
{
  if (__dims == 1) {
    if (__count >= min_count) {
      // add this node
      this->__filter_next = NULL;
      tail->__filter_next = this;
      tail = this;
    }
    if (__left)   tail = __left->filter(tail, min_count);
    if (__right)  tail = __right->filter(tail, min_count);      

  } else {
    if (__dim_next)  tail = __dim_next->filter(tail, min_count);
    if (__left)      tail = __left->filter(tail, min_count);
    if (__right)     tail = __right->filter(tail, min_count);
  }

  return tail;
}
