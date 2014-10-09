
/***************************************************************************
 *  astar_state.h - Representation of a state in the A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_COLLI_SEARCH_ASTAR_STATE_H_
#define __PLUGINS_COLLI_SEARCH_ASTAR_STATE_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class AStarState <plugins/colli/search/astar_state.h>
 *  This is the class for an A* State.
 */
class AStarState
{
 public:
  AStarState( );
  AStarState( int x, int y, int past_cost, AStarState * father );
  ~AStarState();

  int x_;  /**< x coordinate of the state */
  int y_;  /**< y coordinate of the state */

  AStarState * father_; /**< The predecessor state */

  int past_cost_;  /**< The past cost */
  int total_cost_; /**< The total cost */
};


/* ************************************************************************** */
/* ***********************  IMPLEMENTATION DETAILS  ************************* */
/* ************************************************************************** */

/**  This is the standard constructor. */
inline
AStarState::AStarState( )
{
  father_ =  0;
  x_ = y_ = 0;
  total_cost_ = 0;
  past_cost_ = 0;
}

/**  This is another standard constuctor, this time parametrized.
 * @param x is the x coordinate.
 * @param y is the y coordinate.
 * @param past_cost is the total left cost.
 * @param father is a pointer to the predecessor of this AStarState.
 */
inline
AStarState::AStarState( int x, int y, int past_cost, AStarState * father )
{
  x_ = x;
  y_ = y;
  past_cost_ = past_cost;
  father_ = father;
}

/** Standard Destructor */
inline
AStarState::~AStarState( )
{
}

} // namespace fawkes

#endif
