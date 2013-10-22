
/***************************************************************************
 *  og_laser.h - Occupancy grid for colli's A* search
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

#ifndef __PLUGINS_COLLI_SEARCH_OG_LASER_H_
#define __PLUGINS_COLLI_SEARCH_OG_LASER_H_

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

class CLaserOccupancyGrid : public OccupancyGrid
{
 public:

  CLaserOccupancyGrid( Laser * laser, Logger* logger, Configuration* config,
                       int width = 150, int height = 150,
                       int cell_width = 5, int cell_height = 5);

  ~CLaserOccupancyGrid();

  ///\brief Put the laser readings in the occupancy grid
  void UpdateOccGrid( int midX, int midY, float inc, float vel,
                      float xdiff, float ydiff, float oridiff );

  ///\brief Reset all old readings and forget about the world state!
  void ResetOld( int max_age = -1 );

  ///\brief Get the laser's position in the grid
  point_t GetLaserPosition();

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

  point_t m_LaserPosition; /**< the laser's position in the grid */

  int m_TrigTableResolution; /**< Trigonometry table resolution */

  /** History concerned constants */
  int m_MaxHistoryLength, m_MinHistoryLength, m_InitialHistorySize;

  /** Laser concerned settings */
  float m_MinimumLaserLength, m_EllipseDistance;

  bool cfg_obstacle_inc_ ; /**< increasing obstacles or not */
};

} // namespace fawkes

#endif
