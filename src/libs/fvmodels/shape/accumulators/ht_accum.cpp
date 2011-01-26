
/***************************************************************************
 *  ht_accum.cpp - Accumulator class for HoughTransform
 *
 *  Generated: Tue Jun 28 2005
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
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

#include <fvmodels/shape/accumulators/ht_accum.h>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

RhtXNode* RhtXNode::reuse_head = NULL;
RhtYNode* RhtYNode::reuse_head = NULL;
RhtRNode* RhtRNode::reuse_head = NULL;

RhtXNode* RhtXNode::reuse_tail = NULL;
RhtYNode* RhtYNode::reuse_tail = NULL;
RhtRNode* RhtRNode::reuse_tail = NULL;

/** @class RhtAccNode <fvmodels/shape/accumulators/ht_accum.h>
 * Hough-Transform accumulator node.
 */

/** @class RhtRNode <fvmodels/shape/accumulators/ht_accum.h>
 * Hough-Transform accumulator node.
 */

/** @class RhtXNode <fvmodels/shape/accumulators/ht_accum.h>
 * Hough-Transform accumulator node.
 */

/** @class RhtYNode <fvmodels/shape/accumulators/ht_accum.h>
 * Hough-Transform accumulator node.
 */

/** @class RhtAccumulator <fvmodels/shape/accumulators/ht_accum.h>
 * Hough-Transform accumulator.
 */

/** Constructor. */
RhtAccNode::RhtAccNode()
{
  left = right = next = NULL;
}


/** Destructor. */
RhtAccNode::~RhtAccNode()
{
}

/** Clear.
 * @param ignore ignored
 */
void
RhtAccNode::clear(int ignore)
{
  left = right = NULL;
}


/** Constructor.
 * @param x x
 */
RhtXNode::RhtXNode(int x)
  : RhtAccNode()
{
  this->x=x;
  y_root = NULL;
}


/** Insert node.
 * @param x0 x
 * @param y0 y
 * @param r0 r
 * @return ?
 */
int
RhtXNode::insert(int x0, int y0, int r0)
{
  if (x == x0)
    {
      if (!y_root)
	y_root = RhtYNode::generate(y0);
      return y_root->insert(y0, r0);
    }
  else if (x0 < x)
    {
      if (!left)
	left = generate(x0);
      return ((RhtXNode*)left)->insert(x0, y0, r0);
    }
  else
    {
      if (!right)
	right = generate(x0);
      return ((RhtXNode*)right)->insert(x0, y0, r0);
    }
}

/** Get nodes.
 * @param rv return value
 * @param min_votes minimum nomber of votes
 */
void
RhtXNode::getNodes(std::vector< std::vector< int > > *rv, int min_votes)
{
  if (left) {
    ((RhtXNode*)left)->getNodes(rv, min_votes);
  }

  if (y_root) {
    y_root->getNodes(rv, min_votes, x);
  }

  if (right) {
    ((RhtXNode*)right)->getNodes(rv, min_votes);
  }
}

/** Dump to stream.
 * @param s stream to dump to.
 */
void
RhtXNode::dump(std::ostream& s)
{
  if (left)
    ((RhtXNode*)left)->dump(s);
  y_root->dump(s, x);
  if (right)
    ((RhtXNode*)right)->dump(s);
}

/** Generate.
 * @param x ?
 * @return node
 */
RhtXNode *
RhtXNode::generate(int x)
{
  if (reuse_tail == NULL)
    {
      RhtXNode* p=new RhtXNode(x);
      p->next = reuse_head;
      reuse_head = p;
      return reuse_head;
    }
  else
    {
      RhtXNode* p=reuse_tail;
      reuse_tail = (RhtXNode*)(reuse_tail->next);
      p->clear(x);
      return p;
    }
}


/** Clear.
 * @param x x to clear
 */
void
RhtXNode::clear(int x)
{
  RhtAccNode::clear(x);
  this->x = x;
  y_root = NULL;
}

/** Reset. */
void
RhtXNode::reset(void)
{
  reuse_tail = reuse_head;
}


/** Cleanup. */
void
RhtXNode::cleanup(void)
{
  while(reuse_head)
    {
      reuse_tail = (RhtXNode*)reuse_head->next;
      delete reuse_head;
      reuse_head = reuse_tail;
    }
}


/** Constructor.
 * @param y y
 */
RhtYNode::RhtYNode(int y)
  : RhtAccNode()
{
  this->y=y;
  r_root = NULL;
}

/** Insert.
 * @param y0 y
 * @param r0 r
 * @return number of sub-elements
 */
int
RhtYNode::insert(int y0, int r0)
{
  if (y == y0)
    {
      if (!r_root)
	r_root = RhtRNode::generate(r0);
      return r_root->insert(r0);
    }
  else if (y0 < y)
    {
      if (!left)
	left = generate(y0);
      return ((RhtYNode*)left)->insert(y0, r0);
    }
  else
    {
      if (!right)
	right = generate(y0);
      return ((RhtYNode*)right)->insert(y0, r0);
    }
}

/** Get nodes.
 * @param rv return value
 * @param min_votes min votes
 * @param x x
 */
void
RhtYNode::getNodes(std::vector< std::vector< int > > *rv, int min_votes, int x)
{
  if (left) {
    ((RhtYNode*)left)->getNodes(rv, min_votes, x);
  }

  if (r_root) {
    r_root->getNodes(rv, min_votes, x, y);
  }

  if (right) {
    ((RhtYNode*)right)->getNodes(rv, min_votes, x);
  }
}


/** Dump.
 * @param s dump to s
 * @param x x
 */
void
RhtYNode::dump(std::ostream& s, int x)
{
  if (left)
    ((RhtYNode*)left)->dump(s, x);
  r_root->dump(s, x, y);
  if (right)
    ((RhtYNode*)right)->dump(s, x);
}


/** Generate.
 * @param y y
 * @return node
 */
RhtYNode *
RhtYNode::generate(int y)
{
  if (reuse_tail == NULL)
    {
      RhtYNode* p=new RhtYNode(y);
      p->next = reuse_head;
      reuse_head = p;
      return reuse_head;
    }
  else
    {
      RhtYNode* p=reuse_tail;
      reuse_tail = (RhtYNode*)(reuse_tail->next);
      p->clear(y);
      return p;
    }
}


/** Clear.
 * @param y y
 */
void
RhtYNode::clear(int y)
{
  RhtAccNode::clear(y);
  this->y = y;
  r_root = NULL;
}

/** Reset. */
void
RhtYNode::reset(void)
{
  reuse_tail = reuse_head;
}


/** Cleanup. */
void
RhtYNode::cleanup(void)
{
  while(reuse_head)
    {
      reuse_tail = (RhtYNode*)reuse_head->next;
      delete reuse_head;
      reuse_head = reuse_tail;
    }
}

/** Constructor.
 * @param r r
 */
RhtRNode::RhtRNode(int r)
  : RhtAccNode()
{
  this->r=r; count = 0;
}


/** Clear. */
void
RhtRNode::clear(void)
{
  count = 0;
}



/** Insert.
 * @param r0 r
 * @return ?
 */
int RhtRNode::insert(int r0)
{
  if (r == r0)
    {
      return ++count;
    }
  else if (r0 < r)
    {
      if (!left)
	left = generate(r0);
      return ((RhtRNode*)left)->insert(r0);
    }
  else
    {
      if (!right)
	right = generate(r0);
      return ((RhtRNode*)right)->insert(r0);
    }
}

/** Get nodes.
 * @param rv return value
 * @param min_votes min votes
 * @param x x
 * @param y y
 */
void
RhtRNode::getNodes(std::vector< std::vector< int > > *rv, int min_votes, int x, int y)
{
  if (left) {
    ((RhtRNode*)left)->getNodes(rv, min_votes, x, y);
  }
  if (count >= min_votes) {
    vector< int > node;
    node.push_back( x );
    node.push_back( y );
    node.push_back( r );
    node.push_back( count );
    rv->push_back( node );
  }
  if (right) {
    ((RhtRNode*)right)->getNodes(rv, min_votes, x, y);
  }
}


/** Dump.
 * @param s dump to s
 * @param x x
 * @param y y
 */
void
RhtRNode::dump(std::ostream& s, int x, int y)
{
  if (left)
    ((RhtRNode*)left)->dump(s, x, y);
  s << "("<<x<<","<<y<<","<<r<<") with vote "<<count<<endl;
  if (right)
    ((RhtRNode*)right)->dump(s, x, y);
}


/** Generate.
 * @param r r
 * @return node
 */
RhtRNode *
RhtRNode::generate(int r)
{
  if (reuse_tail == NULL)
    {
      RhtRNode* p=new RhtRNode(r);
      p->next = reuse_head;
      reuse_head = p;
      return reuse_head;
    }
  else
    {
      RhtRNode* p=reuse_tail;
      reuse_tail = (RhtRNode*)(reuse_tail->next);
      p->clear(r);
      return p;
    }
}

/** Clear.
 * @param r r
 */
void
RhtRNode::clear(int r)
{
  RhtAccNode::clear(r);
  this->r = r;
  count = 0;
}

/** Reset. */
void
RhtRNode::reset(void)
{
  reuse_tail = reuse_head;
}

/** Cleanup. */
void
RhtRNode::cleanup(void)
{
  while(reuse_head)
    {
      reuse_tail = (RhtRNode*)reuse_head->next;
      delete reuse_head;
      reuse_head = reuse_tail;
    }
}


/** Constructor. */
RhtAccumulator::RhtAccumulator()
{
  root = NULL;
  max=0;
}


/** Destructor. */
RhtAccumulator::~RhtAccumulator()
{
  RhtXNode::cleanup();
  RhtYNode::cleanup();
  RhtRNode::cleanup();
}


/** Reset. */
void
RhtAccumulator::reset(void)
{
  max = 0;
  root = NULL;
  num_votes = 0;
  RhtXNode::reset();
  RhtYNode::reset();
  RhtRNode::reset();
}


/** Accumulate new candidate.
 * @param x x
 * @param y y
 * @param r r
 * @return count
 */
int
RhtAccumulator::accumulate(int x, int y, int r)
{
  ++num_votes;

  if (!root)
    root = RhtXNode::generate(x);
  int count = root->insert(x, y, r);
  if (count > max) {
    max = count;
    x_max = x;
    y_max = y;
    r_max = r;
  }
  return count;
}


/** Get maximum
 * @param x x return value
 * @param y y return value
 * @param r r return value
 * @return max
 */
int
RhtAccumulator::getMax(int &x, int &y, int &r) const
{
  x = x_max;
  y = y_max;
  r = r_max;
  return max;
}

/** Dump.
 * @param s stream
 */
void
RhtAccumulator::dump(std::ostream& s)
{
  if (root)
    root->dump(s);
}


/** Get number of votes.
 * @return number of votes
 */
unsigned int
RhtAccumulator::getNumVotes() const
{
  return num_votes;
}


/** Get nodes.
 * @param min_votes min votes
 * @return nodes
 */
vector< vector< int > > *
RhtAccumulator::getNodes(int min_votes)
{
  vector< vector< int > > *rv = new vector< vector< int > >();

  if ( (min_votes <= num_votes) && (root != NULL) ) {
    root->getNodes( rv, min_votes );
  }

  return rv;
}

} // end namespace firevision
