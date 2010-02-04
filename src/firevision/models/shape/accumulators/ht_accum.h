
/***************************************************************************
 *  ht_accum.h - Accumulator class for HoughTransform
 *
 *  Created: Tue Jun 28 00:00:00 2005
 *  Copyright  2005  Hu Yuxiao <Yuxiao.Hu@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_SHAPE_ACCUMULATORS_HT_ACCUM_H_
#define __FIREVISION_MODELS_SHAPE_ACCUMULATORS_HT_ACCUM_H_

#include <stdlib.h>
#include <ostream>
#include <vector>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RhtAccNode
{
 public:
  RhtAccNode();
  virtual ~RhtAccNode();
  virtual void clear(int ignore);

 protected:
  /** left */
  RhtAccNode* left;
  /** right */
  RhtAccNode* right;
  /** used for recycling */
  RhtAccNode* next;
};

class RhtRNode : public RhtAccNode
{
 public:
  RhtRNode(int r);
  void clear(void);
  int  insert(int r);
  void dump(std::ostream&, int x, int y);
  void clear(int r);
  void getNodes(std::vector< std::vector< int > > *rv, int min_votes, int x, int y);

  static RhtRNode* generate(int r);
  static void reset(void);
  static void cleanup(void);

 protected:
  /** r */
  int r;
  /** count */
  int count;

 private:
  static RhtRNode* reuse_head;
  static RhtRNode* reuse_tail;
};

class RhtYNode : public RhtAccNode
{
 private:
  static RhtYNode* reuse_head;
  static RhtYNode* reuse_tail;
 public:
  static RhtYNode* generate(int y);
  static void reset(void);
  static void cleanup(void);
	
 protected:
  /** y */
  int y;
  /** r_root */
  RhtRNode* r_root;
	
 public:
  RhtYNode(int y);
  int insert(int y, int r);
  void dump(std::ostream&, int x);
  void clear(int y);
  void getNodes(std::vector< std::vector< int > > *rv, int min_votes, int x);
};

class RhtXNode : public RhtAccNode
{
 private:
  static RhtXNode* reuse_head;
  static RhtXNode* reuse_tail;
 public:
  static RhtXNode* generate(int x);
  static void reset(void);
  static void cleanup(void);

 protected:
  /** x */
  int x;
  /** y root */
  RhtYNode* y_root;

 public:
  RhtXNode(int x);
  int insert(int x, int y, int r);
  void dump(std::ostream&);
  void clear (int x);
  void getNodes(std::vector< std::vector< int > > *rv, int min_votes);
};

class RhtAccumulator
{
 private:
  int		x_max;
  int		y_max;
  int		r_max;
  int		max;

  RhtXNode*	root;

  int             num_votes;

 public:
  RhtAccumulator();
  ~RhtAccumulator();
  int accumulate(int x, int y, int r);
  int getMax(int& x, int& y, int& r) const;
  void dump(std::ostream&);
  void reset(void);
  unsigned int getNumVotes() const;
  std::vector< std::vector< int > > * getNodes(int min_count);
};

} // end namespace firevision

#endif
