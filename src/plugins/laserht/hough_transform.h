
/***************************************************************************
 *  hough_transform.h - Hough Transform
 *
 *  Created: Mon Dec 28 2009 18:65:04 (based on FireVision's HtAccum)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *             2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de> (FireVision)
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

#ifndef __PLUGINS_LASERHT_HT_ACCUM_H_
#define __PLUGINS_LASERHT_HT_ACCUM_H_

class HoughTransform {
 private:
  class Node {
  public:
    Node(unsigned int dims, int value = 0);
    ~Node();

    unsigned int insert(int *values);

    unsigned int num_nodes();
    unsigned int depth();

    unsigned int filter(int **values, unsigned int min_count);

  private:
    Node(Node *parent, unsigned int dims, int value = 0);
    Node();

    Node * filter(Node *tail, unsigned int min_count);
    unsigned int filtered_length();

  private:
    unsigned int __dims;

    unsigned int __count;
    int   __value;

    Node *__parent; // that is the "value parent", not necessarily tree parent
    Node *__left;
    Node *__right;
    Node *__dim_next;

    Node *__filter_next;

    // for re-use (avoiding re-allocations)
    // Node *__reuse_next;
  };

 public:
  HoughTransform(unsigned int num_dims);
  ~HoughTransform();

  void process(int **values, unsigned int num_values);
  unsigned int max(int *values) const;

  unsigned int filter(int **values, unsigned int min_count);

  void reset();

  Node *  root();

 private:
  Node *__root;

  unsigned int __num_dims;
  unsigned int __max_count;
  int *__max_values;
};


#endif
