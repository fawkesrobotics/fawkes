
/***************************************************************************
 *  astar_state.h - Abstract class of a astar state.
 *
 *  Generated: Mon Sep 15 18:48:00 2002
 *  Copyright  2002  Stefan Jacobs
 *             2007  Martin Liebenberg
 *             2012  Tim Niemueller [www.niemueller.de]
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

#ifndef _ASTAR_ABSTRACT_STATE_H_
#define _ASTAR_ABSTRACT_STATE_H_

#include <vector>
#include <cstdlib>

namespace fawkes {

/** @class AStarState <utils/search/astar_state.h>
 *  This is the abstract(!) class for an A* State.
 * 
 * @author Stefan Jacobs
 */
class AStarState
{
 public:
  
  /** Constructor. */
  AStarState() {};
  
  /** Destructor. */
  virtual ~AStarState() {};

  
  // ***** You have to implement the following 4 methods! ***** //
  // ***** ============================================== ***** //
  
  /** Generates a unique key for this state.
   * There has to be a unique key for each state (fast closed list -> bottleneck!)
   * @return unique key
   */
  virtual size_t key() = 0;
  
  /** Estimate the heuristic cost to the goal.
   * @return estimated cost as double
   */
  virtual double estimate() = 0;
  
  /** Check, wether we reached a goal or not.
   * @return true, if this state is a goal, else false
   */
  virtual bool is_goal() = 0;
  
  /** Generate all successors and put them to this vector.
   *  @return a vector of pointers of AStarState to a successor
   */
  virtual std::vector<AStarState *> children() = 0;

  /** Predecessor. */
  AStarState *parent;

  /** Past cost. */
  double past_cost;
  /** Total estimated cost. */
  double total_estimated_cost;

};

} // end namespace fawkes

#endif
