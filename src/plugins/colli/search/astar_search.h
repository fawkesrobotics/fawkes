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
/* $Id$             */
/*                                                                      */
/* Description: This is the interpretation class interface for A* of    */
/*              Colli-A*                                                */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This class tries to translate the found plan to interpreteable */
/*       things for the rest of the program.                            */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_SEARCH_ASTAR_SEARCH_H_
#define _COLLI_SEARCH_ASTAR_SEARCH_H_

#include "abstract_search.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CLaserOccupancyGrid;
class CAStar;
class Logger;
class Configuration;

typedef struct point_struct point_t;

/** This is the plan class.
 *  Here the plan from A* is managed and cut into small pieces.
 *    Also usable methods for managing the plan are implemented here.
 */
class CSearch: public CAbstractSearch
{
 public:
  /** Constructor */
  CSearch( CLaserOccupancyGrid * occGrid , Logger* logger, Configuration* config);

  /** Destructor */
  virtual ~CSearch();

  /** update complete plan things
   * precondition: the occupancy grid has to be updated previously!
   */
  void Update( int roboX, int roboY, int targetX, int targetY );

  /** returns, if the update was successful or not.
   * precondition: update had to be called.
   */
  bool UpdatedSuccessful();

 private:

  /** Returns the current, modified waypoint to drive to. */
  point_t CalculateLocalTarget();

  /** Adjust the waypoint if it is not the final point. */
  point_t AdjustWaypoint( const point_t &local_target );

  /** Returns the current trajectory point to drive to. */
  point_t CalculateLocalTrajectoryPoint( );

  /** Method for checking if an obstacle is between two points. */
  bool IsObstacleBetween( const point_t &a, const point_t &b, const int maxcount );



  // --------------------------------- //
  //    VARIABLES
  // --------------------------------- //

  CAStar * m_pAStar;              /**< the A* search algorithm */
  std::vector< point_t > m_vPlan; /**< the local representation of the plan */

  point_t m_RoboPosition, m_TargetPosition;
  bool m_UpdatedSuccessful;

  fawkes::Logger* logger_;
};

} // namespace fawkes

#endif
