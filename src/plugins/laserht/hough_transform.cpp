
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

HoughTransform::HoughTransform(unsigned int num_dims)
{
  __root = new Node(num_dims);

  __num_dims   = num_dims;

  __max_count  = 0;
  __max_values = new int[num_dims];
}

HoughTransform::~HoughTransform()
{
  delete __root;
  delete[] __max_values;
}


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


unsigned int
HoughTransform::max(int *values) const
{
  for (unsigned int d = 0; d < __num_dims; ++d) {
    values[d] = __max_values[d];
  }
  return __max_count;
}


unsigned int
HoughTransform::filter(int **values, unsigned int min_count)
{
  return __root->filter(values, min_count);
}


HoughTransform::Node *
HoughTransform::root()
{
  return __root;
}

void
HoughTransform::reset()
{
  delete __root;
  __root = new Node(__num_dims);
  __max_count = 0;
  for (unsigned int d = 0; d < __num_dims; ++d) {
    __max_values[d] = 0;
  }
}

HoughTransform::Node::Node(unsigned int dims, int value)
{
  __dims   = dims;
  __value  = value;
  __count  = 0;
  __parent = NULL;
  __left = __right = __dim_next = __filter_next = 0;
  //if (parent) {
  //  __reuse_next = parent->__reuse_next;
  //  parent->__reuse_next = this;
  //}
}

HoughTransform::Node::Node(Node *parent, unsigned int dims, int value)
{
  __parent = parent;
  __dims   = dims;
  __value  = value;
  __count  = 0;
  __left = __right = __dim_next = __filter_next = 0;
  //if (parent) {
  //  __reuse_next = parent->__reuse_next;
  //  parent->__reuse_next = this;
  //}
}

HoughTransform::Node::Node()
{
  __dims   = 123;
  __value  = 0;
  __count  = 0;
  __parent = NULL;
  __left = __right = __dim_next = __filter_next = 0;
}

HoughTransform::Node::~Node()
{
  delete __left;
  delete __right;
  delete __dim_next;
}


unsigned int
HoughTransform::Node::insert(int *values)
{
  if (values[0] == __value) {
    if ( __dims > 1) {
      if (! __dim_next) {
	//if (__reuse_next) {
	//  __dim_next = __reuse_next;
	//  __reuse_next = __reuse_next->__reuse_next;
	//} else {
	__dim_next = new Node(this, __dims - 1, values[1]);
	//}
      }

      return __dim_next->insert(&(values[1]));
    } else {
      return ++__count;
    }
  } else if (values[0] < __value) {
    if (! __left) {
      __left = new Node(__parent, __dims, values[0]);
    }
    return __left->insert(values);
  } else { // values[0] > __value
    if (! __right) {
      __right = new Node(__parent, __dims, values[0]);
    }
    return __right->insert(values);
  }
}


unsigned int
HoughTransform::Node::num_nodes()
{
  unsigned int rv = 1;
  if (__left)     rv += __left->num_nodes();
  if (__right)    rv += __right->num_nodes();
  if (__dim_next) rv += __dim_next->num_nodes();
  return rv;
}

unsigned int
HoughTransform::Node::depth()
{
  unsigned int d = 0;
  if (__left)     d = std::max(d, __left->depth());
  if (__right)    d = std::max(d, __right->depth());
  if (__dim_next) d = std::max(d, __dim_next->depth());
  return d + 1;
}


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

  *values = fvals;
  return flen;
}


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
