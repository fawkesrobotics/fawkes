
/***************************************************************************
 *  og_laser.cpp - Occupancy grid for colli's A* search
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

#include "og_laser.h"
#include "obstacle_map.h"

#include "../common/defines.h"
#include "../utils/rob/roboshape_colli.h"
#include "../utils/rob/robo_laser.h"

#include <utils/math/angle.h>
#include <logging/logger.h>
#include <config/config.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLaserOccupancyGrid <plugins/colli/search/og_laser.h>
 *  This OccGrid is derived by the Occupancy Grid originally from Andreas Strack,
 *    but modified for speed purposes.
 */

/** Constructor.
 * @param laser The Laser object
 * @param logger The fawkes logger
 * @param config The fawkes configuration
 * @param width The width of the grid (in m)
 * @param height The height of the grid (in m)
 * @param cell_width The width of a cell (in cm)
 * @param cell_height The height of a cell (in cm)
 */
CLaserOccupancyGrid::CLaserOccupancyGrid( Laser * laser, Logger* logger, Configuration* config,
                                          int width, int height,
                                          int cell_width, int cell_height)
 : OccupancyGrid( width, height, cell_width, cell_height )
{
  logger->log_info("CLaserOccupancyGrid", "(Constructor): Entering");
  std::string cfg_prefix = "/plugins/colli/";

  m_pLaser = laser;
  m_pRoboShape = new CRoboShape_Colli( (cfg_prefix + "roboshape/").c_str(), logger, config );
  m_vOldReadings.clear();
  initGrid();

  m_EllipseDistance     = config->get_float((cfg_prefix + "laser_occupancy_grid/distance_account").c_str());
  m_InitialHistorySize  = 3*config->get_int((cfg_prefix + "laser_occupancy_grid/history/initial_size").c_str());
  m_MaxHistoryLength    = config->get_int(  (cfg_prefix + "laser_occupancy_grid/history/max_length").c_str());
  m_MinHistoryLength    = config->get_int(  (cfg_prefix + "laser_occupancy_grid/history/min_length").c_str());
  m_MinimumLaserLength  = config->get_float((cfg_prefix + "laser/min_reading_length").c_str());


  cfg_obstacle_inc_         = config->get_bool((cfg_prefix + "obstacle_increasement").c_str());

  logger->log_debug("CLaserOccupancyGrid", "Generating obstacle map");
  obstacle_map = new ColliObstacleMap(m_pRoboShape->IsAngularRobot());
  logger->log_debug("CLaserOccupancyGrid", "Generating obstacle map done");

  m_LaserPosition = point_t(0,0);

  // calculate laser offset from robot center
  offset_base_.x=0;
  offset_base_.y=0;
  offset_laser_.x = m_pRoboShape->GetCompleteWidthX()/2.f - m_pRoboShape->GetRobotLengthforDegree(0);
  offset_laser_.y = m_pRoboShape->GetCompleteWidthY()/2.f - m_pRoboShape->GetRobotLengthforDegree(90);
  logger->log_debug("CLaserOccupancyGrid", "Laser (x,y) offset from robo-center is (%f, %f)",
                    offset_laser_.x, offset_laser_.y);


  logger->log_info("CLaserOccupancyGrid", "(Constructor): Exiting");
}

/** Descturctor. */
CLaserOccupancyGrid::~CLaserOccupancyGrid()
{
  delete m_pRoboShape;
}

/** Reset all old readings and forget about the world state!
 * @param max_age The maximum age of for a reading to be considered.
 */
void
CLaserOccupancyGrid::ResetOld( int max_age )
{
  if ( max_age == -1 ) {
    m_vOldReadings.clear();
    m_vOldReadings.reserve( m_InitialHistorySize );
    return;

  } else if ( max_age > 0 ) {
    std::vector< float > old_readings;
    old_readings.reserve( m_InitialHistorySize );

    for ( unsigned int i = 0; i < m_vOldReadings.size(); i+=3 ) {
      if ( m_vOldReadings[i+2] < max_age ) {
        old_readings.push_back( m_vOldReadings[i] );
        old_readings.push_back( m_vOldReadings[i+1] );
        old_readings.push_back( m_vOldReadings[i+2] );
      }
    }

    m_vOldReadings.clear();
    m_vOldReadings.reserve( m_InitialHistorySize );

    // integrate the new calculated old readings
    for ( unsigned int i = 0; i < old_readings.size(); i++ )
      m_vOldReadings.push_back( old_readings[i] );
  }
}


/** Put the laser readings in the occupancy grid
 *  Also, every reading gets a radius according to the relative direction
 *  of this reading to the robot.
 * @param midX is the current x position of the robot in the grid.
 * @param midY is the current y position of the robot in the grid.
 * @param inc is the current constant to increase the obstacles.
 * @param vel Translation velocity of the motor
 * @param xdiff The traveled distance on x-axis since previous call
 * @param ydiff The traveled distance on y-axis since previous call
 * @param oridiff The rotation around z-axis since previous call
 */
void
CLaserOccupancyGrid::UpdateOccGrid( int midX, int midY, float inc, float vel,
                                    float xdiff, float ydiff, float oridiff )
{
  m_LaserPosition.x = midX;
  m_LaserPosition.y = midY;

  for ( int y = 0; y < m_Width; ++y )
    for ( int x = 0; x < m_Height; ++x )
      m_OccupancyProb[x][y] = _COLLI_CELL_FREE_;

  IntegrateOldReadings( midX, midY, inc, vel, xdiff, ydiff, oridiff );
  IntegrateNewReadings( midX, midY, inc, vel );
}


/** Get the laser's position in the grid
 * @return point_t structure containing the laser's position in the grid
 */
point_t
CLaserOccupancyGrid::GetLaserPosition()
{
  return m_LaserPosition;
}

/** Set the offset of base_link from laser.
 * @param x offset in x-direction (in meters)
 * @param y offset in y-direction (in meters)
 */
void
CLaserOccupancyGrid::set_base_offset(float x, float y)
{
  offset_base_.x = (int)round( (offset_laser_.x + x)*100.f/m_CellHeight ); // # in grid-cells
  offset_base_.y = (int)round( (offset_laser_.y + y)*100.f/m_CellWidth  );
}


void
CLaserOccupancyGrid::IntegrateOldReadings( int midX, int midY, float inc, float vel,
                                           float xdiff, float ydiff, float oridiff )
{
  std::vector< float > old_readings;
  old_readings.reserve( m_InitialHistorySize );

  float oldpos_x, oldpos_y;
  float newpos_x, newpos_y;

  float history = std::max( m_MinHistoryLength,
                            m_MaxHistoryLength - (int)std::max( 0.0, fabs(vel)-0.5 ) * 20 );

  // update all old readings
  for ( unsigned int i = 0; i < m_vOldReadings.size(); i+=3 ) {

    if ( m_vOldReadings[i+2] < history ) {
      oldpos_x = m_vOldReadings[i];
      oldpos_y = m_vOldReadings[i+1];

      newpos_x =  -xdiff + (  oldpos_x * std::cos( oridiff ) +
                              oldpos_y * std::sin( oridiff ) );
      newpos_y =  -ydiff + ( -oldpos_x * std::sin( oridiff ) +
                              oldpos_y * std::cos( oridiff ) );

      float angle_to_old_reading = atan2( newpos_y, newpos_x );
      float sqr_distance_to_old_reading = sqr( newpos_x ) + sqr( newpos_y );

      int number_of_old_reading = (int)rad2deg(
          normalize_rad(360.f/m_pLaser->GetNumberOfReadings() * angle_to_old_reading) );
      // This was RCSoftX, now ported to fawkes:
      //int number_of_old_reading = (int) (normalize_degree( ( 360.0/(m_pLaser->GetNumberOfReadings()) ) *
      //         rad2deg(angle_to_old_reading) ) );


      bool SollEintragen = true;

      // do not insert if current reading at that angle deviates more than 30cm from old reading
      // TODO. make those 30cm configurable
      if ( sqr( m_pLaser->GetReadingLength( number_of_old_reading ) - 0.3 ) > sqr_distance_to_old_reading )
        SollEintragen = false;

      if ( SollEintragen == true ) {
        int posX = midX + (int)((newpos_x*100.f) / ((float)m_CellHeight ));
        int posY = midY + (int)((newpos_y*100.f) / ((float)m_CellWidth ));
        if( posX > 4 && posX < m_Height-5
         && posY > 4 && posY < m_Width-5 )
          {
          old_readings.push_back( newpos_x );
          old_readings.push_back( newpos_y );
          old_readings.push_back( m_vOldReadings[i+2]+1.f );

          // 25 cm's in my opinion, that are here: 0.25*100/m_CellWidth
          //int size = (int)(((0.25f+inc)*100.f)/(float)m_CellWidth);
          int width = m_pRoboShape->GetCompleteWidthY();
          width = std::max( 4.f, ((width + inc)*100.f)/m_CellWidth );
          int height = m_pRoboShape->GetCompleteWidthX();
          height = std::max( 4.f, ((height + inc)*100.f)/m_CellHeight );
          integrateObstacle( posX, posY, width, height );
        }
      }

    }
  }

  m_vOldReadings.clear();
  m_vOldReadings.reserve( old_readings.size() );

  // integrate the new calculated old readings
  for ( unsigned int i = 0; i < old_readings.size(); i++ )
    m_vOldReadings.push_back( old_readings[i] );
}


void
CLaserOccupancyGrid::IntegrateNewReadings( int midX, int midY,
                                           float inc, float vel )
{
  int numberOfReadings = m_pLaser->GetNumberOfReadings();
  int posX, posY;
  cart_coord_2d_t point;
  float p_x, p_y;
  float oldp_x = 1000.f;
  float oldp_y = 1000.f;

  for ( int i = 0; i < numberOfReadings; i++ ) {

    if( m_pLaser->IsValid(i) && m_pLaser->GetReadingLength(i) >= m_MinimumLaserLength ) {
      // point = transformLaser2Motor(Point(m_pLaser->GetReadingPosX(i), m_pLaser->GetReadingPosY(i)));
      point.x = m_pLaser->GetReadingPosX(i);
      point.y = m_pLaser->GetReadingPosY(i);

      p_x = point.x;
      p_y = point.y;

      if( !((p_x == 0.f) && (p_y == 0.f)) && sqr(p_x-oldp_x)+sqr(p_y-oldp_y) > sqr(m_EllipseDistance) ) {
        oldp_x = p_x;
        oldp_y = p_y;
        posX = midX + (int)((p_x*100.f) / ((float)m_CellHeight ));
        posY = midY + (int)((p_y*100.f) / ((float)m_CellWidth ));

        if ( !( posX <= 5 || posX >= m_Height-6 || posY <= 5 || posY >= m_Width-6 ) ) {
          float width = 0.f;
          width = m_pRoboShape->GetCompleteWidthY();
          width = std::max( 4.f, ((width + inc)*100.f)/m_CellWidth );

          float height = 0.f;
          height = m_pRoboShape->GetCompleteWidthX();
          height = std::max( 4.f, ((height + inc)*100.f)/m_CellHeight );

          integrateObstacle( posX, posY, width, height );

          if ( !Contained( p_x, p_y ) ) {
            m_vOldReadings.push_back( p_x );
            m_vOldReadings.push_back( p_y );
            m_vOldReadings.push_back( 0.f );
          }
        }
      }
    }
  }
}


bool
CLaserOccupancyGrid::Contained( float p_x, float p_y )
{
  for( unsigned int i = 0; i < m_vOldReadings.size(); i+=3 ) {
    if ( sqr(p_x - m_vOldReadings[i]) + sqr(p_y - m_vOldReadings[i+1]) < sqr( m_EllipseDistance ) )
      return true;
  }

  return false;
}



void
CLaserOccupancyGrid::integrateObstacle( int x, int y, int width, int height )
{
  std::vector< int > fast_obstacle = obstacle_map->get_obstacle( width, height, cfg_obstacle_inc_ );

  int posX = 0;
  int posY = 0;

  // i = x offset, i+1 = y offset, i+2 is cost
  for( unsigned int i = 0; i < fast_obstacle.size(); i+=3 ) {
    /* On the laser-points, we draw obstacles based on base_link. The obstacle has the robot-shape,
     * which means that we need to rotate the shape 180° around base_link and move that rotation-
     * point onto the laser-point on the grid. That's the same as adding the center_to_base_offset
     * to the calculated position of the obstacle-center ("x + fast_obstacle[i]" and "y" respectively).
     */
    posX = x + fast_obstacle[i]   + offset_base_.x;
    posY = y + fast_obstacle[i+1] + offset_base_.y;

    if( (posX > 0) && (posX < m_Height)
     && (posY > 0) && (posY < m_Width)
     && (m_OccupancyProb[posX][posY] < fast_obstacle[i+2]) )
      {
      m_OccupancyProb[posX][posY] = fast_obstacle[i+2];
    }
  }
}

} // namespace fawkes
