
/***************************************************************************
 *  astar_state.h - Representation of a state in the A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

/** AStarState.
 *  This is the class for an A* State.
 */
class CAStarState
{
 public:
  /**  This is the standard constructor. */
  CAStarState( );

  /**  This is another standard constuctor, this time parametrized.
   *   @param x is the x coordinate.
   *   @param y is the y coordinate.
   *   @param pastCost is the total left cost.
   *   @param father is a pointer to the predecessor of this
   *          AStarState.
   */
  CAStarState( int x, int y, int pastCost, CAStarState * father );

  /// Destructor.
  ~CAStarState();

  // Coordinates
  int m_X, m_Y;

  // Predecessor
  CAStarState * m_pFather;

  // Costs
  int m_PastCost, m_TotalCost;
};



/* ************************************************************************** */
/* ***********************  IMPLEMENTATION DETAILS  ************************* */
/* ************************************************************************** */

// Standard Constructor
inline
CAStarState::CAStarState( )
{
  m_pFather =  0;
  m_X = m_Y = 0;
  m_TotalCost = 0;
  m_PastCost = 0;
}

// Another Constructor
inline
CAStarState::CAStarState( int x, int y,
                          int pastCost, CAStarState * father )
{
  m_X = x;
  m_Y = y;
  m_PastCost = pastCost;
  m_pFather = father;
}

// Standard Destructor
inline
CAStarState::~CAStarState( )
{
}

} // namespace fawkes

#endif
