//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$            */
/*                                                                      */
/* Description: This is the search class for A* of Colli-A*             */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_SEARCH_ASTAR_STATE_H_
#define _COLLI_SEARCH_ASTAR_STATE_H_

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
