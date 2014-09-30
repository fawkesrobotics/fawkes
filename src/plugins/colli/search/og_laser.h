
/***************************************************************************
 *  og_laser.h - Occupancy grid for colli's A* search
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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
#include "../common/types.h"

#include <utils/math/types.h>
#include <utils/time/time.h>
#include <string>

#include <tf/transformer.h>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Laser360Interface;
class CRoboShape_Colli;
class ColliObstacleMap;

class Logger;
class Configuration;

class CLaserOccupancyGrid : public OccupancyGrid
{
 public:
  CLaserOccupancyGrid( Laser360Interface * laser, Logger* logger, Configuration* config, tf::Transformer* listener,
                       int width = 150, int height = 150,
                       int cell_width = 5, int cell_height = 5);

  ~CLaserOccupancyGrid();

  ///\brief Put the laser readings in the occupancy grid
  float UpdateOccGrid( int midX, int midY, float inc, float vx, float vy );

  ///\brief Reset all old readings and forget about the world state!
  void ResetOld();

  ///\brief Get the laser's position in the grid
  point_t GetLaserPosition();

  ///\brief Set the offset of base_link from laser
  void set_base_offset(float x, float y);

  ///\brief Get cell costs
  colli_cell_cost_t get_cell_costs() const;

 private:

  class LaserPoint {
  public:
    cart_coord_2d_struct coord;
    Time timestamp;

    LaserPoint() { }
//    LaserPoint(LaserPoint& src) {
//      coord     = src.coord;
//      timestamp = src.timestamp;
//    }
//    LaserPoint operator=(LaserPoint src) {
//      coord     = src.coord;
//      timestamp = src.timestamp;
//      return src;
//    }
  };

  void updateLaser();

  float obstacle_in_path_distance( float vx, float vy );

  void validate_old_laser_points(cart_coord_2d_t pos_robot, cart_coord_2d_t pos_new_laser_point);

  std::vector< LaserPoint >* transformLaserPoints(std::vector< LaserPoint >& laserPoints, tf::StampedTransform& transform);

  /** Integrate historical readings to the current occgrid. */
  void IntegrateOldReadings( int midX, int midY, float inc, float vel,
                             tf::StampedTransform& transform );

  /** Integrate the current readings to the current occgrid. */
  void IntegrateNewReadings( int midX, int midY, float inc, float vel,
                             tf::StampedTransform& transform );

  /** Integrate a single obstacle
   * @param x x coordinate of obstacle center
   * @param y y coordinate of obstacle center
   * @param width total width of obstacle
   * @param height total height of obstacle
   */
  void integrateObstacle( int x, int y, int width, int height );

  tf::Transformer* tf_listener;
  std::string m_reference_frame;
  std::string m_laser_frame;
  Logger* logger_;
  bool cfg_write_spam_debug;

  fawkes::Laser360Interface *if_laser_;
  CRoboShape_Colli *m_pRoboShape; /**< my roboshape */
  ColliObstacleMap *obstacle_map;  /**< fast obstacle map */

  std::vector< LaserPoint > m_vNewReadings;
  std::vector< LaserPoint > m_vOldReadings; /**< readings history */

  point_t m_LaserPosition; /**< the laser's position in the grid */

  /** Costs for the cells in grid */
  colli_cell_cost_t cell_costs_;

  /* interface buffer history */
  int m_if_buffer_size;
  std::vector<bool> m_if_buffer_filled;

  /** History concerned constants */
  float m_MaxHistoryLength, m_MinHistoryLength;
  int m_InitialHistorySize;

  /** Laser concerned settings */
  float m_MinimumLaserLength, m_ObstacleDistance;

  int cfg_emergency_stop_beams_used;  /**< number of beams that are used to calculate the min distance to obstacle */

  bool cfg_obstacle_inc_ ;          /**< increasing obstacles or not */
  bool cfg_force_elipse_obstacle_;  /**< the used shape for obstacles */

  bool  cfg_delete_invisible_old_obstacles_; /**< delete old invalid obstables or not */
  int   cfg_delete_invisible_old_obstacles_angle_min_ ;  /**< the min angle for old obstacles */
  int   cfg_delete_invisible_old_obstacles_angle_max_ ;  /**< the max angle for old obstacles */
  float m_angle_min_;   /**< the angle min in rad */
  float m_angle_range_; /**< the angle range from min - max */

  /** Offsets to robot center */
  cart_coord_2d_t offset_laser_; /**< in meters */
  point_t         offset_base_; /**< in grid cells */
};

} // namespace fawkes

#endif
