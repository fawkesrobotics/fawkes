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
/* Description: This is the occ-grid interface for colli_a*,            */
/*              the search algorithm searches on.                       */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This interface is mainly out of implementation reasons given   */
/*       here. It includes a occ-grid and the laserinterface, so no one */
/*       else has to care about.                                        */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_SEARCH_OG_LASER_H_
#define _COLLI_SEARCH_OG_LASER_H_

#include "../utils/occupancygrid/occupancygrid.h"
#include <utils/math/types.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Laser;
class RoboShape;
class TrigTable;
class CEllipseMap;

class Logger;
class Configuration;

/** CLaserOccupancyGrid.
 *  This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
 *    but modified for speed purposes.
 */
class CLaserOccupancyGrid : public OccupancyGrid
{
public:

  /** Constructor. */
  CLaserOccupancyGrid( Laser * laser, Logger* logger, Configuration* config,
                       int width = 150, int height = 150,
                       int cell_width = 5, int cell_height = 5);

  /** Descturctor. */
  ~CLaserOccupancyGrid();

  /** Put the laser readings in the occupancy grid
   *  Also, every reading gets a radius according to the relative direction
   *  of this reading to the robot.
   *  @param midX is the current point of the robot.
   *  @param midY is the current point of the robot.
   *  @param inc is the current constant to increase the obstacles.
   */
  void UpdateOccGrid( int midX, int midY, float inc, float vel,
                      float xdiff, float ydiff, float oridiff );

  /** Reset all old readings and forget about the world state! */
  void ResetOld( int max_age = -1 );

private:

  /** Integrate historical readings to the current occgrid. */
  void IntegrateOldReadings( int midX, int midY, float inc, float vel,
                             float xdiff, float ydiff, float oridiff );

  /** Integrate the current readings to the current occgrid. */
  void IntegrateNewReadings( int midX, int midY, float inc, float vel );

  /** Check if the current value is contained in the history. */
  bool Contained( float p_x, float p_y );

  /** Integrate a single ellipse
   * @param ellipse the ellipse that is to be integrated
   */
  void integrateObstacle( ellipse_t ellipse );

  //  void integrateEllipseCell( int centerx, int centery,
  //                         int i, int j, float prob );

  Laser       *m_pLaser;     /**< pointer to the laser */
  RoboShape   *m_pRoboShape; /**< my roboshape */
  TrigTable   *m_pTrigTable; /**< fast trigonometry table */
  CEllipseMap *ellipse_map;  /**< fast ellipse map */

  std::vector< float > m_vOldReadings; /**< readings history */

  int m_TrigTableResolution; /**< Trigonometry table resolution */

  /** History concerned constants */
  int m_MaxHistoryLength, m_MinHistoryLength, m_InitialHistorySize;

  /** Laser concerned settings */
  float m_MinimumLaserLength, m_EllipseDistance;

  int m_RobocupMode; /**< robocup mode */
};

} // namespace fawkes

#endif
