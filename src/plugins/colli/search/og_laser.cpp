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
/* $Id$           */
/*                                                                      */
/* Description: This is the occ-grid implementation for colli_a*,       */
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


#include "og_laser.h"
#include "ellipse_map.h"

#include "../common/defines.h"
#include "../utils/rob/roboshape.h"
#include "../utils/rob/robo_laser.h"
#include "../utils/geometry/trig_table.h"

#include <utils/math/types.h>
#include <utils/math/angle.h>
#include <logging/logger.h>
#include <config/config.h>

#include <cmath>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

// Constructor
CLaserOccupancyGrid::CLaserOccupancyGrid( Laser * laser, Logger* logger, Configuration* config,
                                          int width, int height,
                                          int cell_width, int cell_height)
 : OccupancyGrid( width, height, cell_width, cell_height )
{
  logger->log_info("CLaserOccupancyGrid", "(Constructor): Entering");
  std::string cfg_prefix = "/plugins/colli/";

  m_pLaser = laser;
  m_pRoboShape = new RoboShape( (cfg_prefix + "Roboshape/").c_str(), logger, config );
  m_vOldReadings.clear();
  initGrid();

  m_MaxHistoryLength    = config->get_int((cfg_prefix + "LaserOccupancyGrid/MAX_HISTORY_LENGTH").c_str());
  m_MinHistoryLength    = config->get_int((cfg_prefix + "LaserOccupancyGrid/MIN_HISTORY_LENGTH").c_str());
  m_InitialHistorySize  = 3*config->get_int((cfg_prefix + "LaserOccupancyGrid/INITIAL_HISTORY_SIZE").c_str());
  m_TrigTableResolution = config->get_int((cfg_prefix + "TrigTable/RESOLUTION").c_str());
  m_MinimumLaserLength  = config->get_float((cfg_prefix + "Laser/MINIMUM_READING_LENGTH").c_str());
  m_EllipseDistance     = config->get_float((cfg_prefix + "LaserOccupancyGrid/DISTANCE_ACCOUNT").c_str());

  m_RobocupMode         = config->get_int((cfg_prefix + "ROBOCUP_MODE").c_str());

  logger->log_debug("CLaserOccupancyGrid", "Generating trigonometry table");
  m_pTrigTable = new TrigTable( m_TrigTableResolution );
  logger->log_debug("CLaserOccupancyGrid", "Generating trigonometry table done");

  logger->log_debug("CLaserOccupancyGrid", "Generating ellipse map");
  ellipse_map = new CEllipseMap();
  logger->log_debug("CLaserOccupancyGrid", "Generating ellipse map done");

  m_LaserPosition = point_t(0,0);

  logger->log_info("CLaserOccupancyGrid", "(Constructor): Exiting");
}



// Destructor
CLaserOccupancyGrid::~CLaserOccupancyGrid()
{
  delete m_pTrigTable;
  delete m_pRoboShape;
}


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



// update the occ grid by putting the laser readings in it.
// the current robopos is the midx and the midy
// and the inc variable is the increase of the obstacles
void
CLaserOccupancyGrid::UpdateOccGrid( int midX, int midY, float inc, float vel,
                                    float xdiff, float ydiff, float oridiff )
{
  m_LaserPosition.x = midX;
  m_LaserPosition.y = midY;

  for ( int y = 0; y < m_Height; ++y )
    for ( int x = 0; x < m_Width; ++x )
      m_OccupancyProb[x][y] = _COLLI_CELL_FREE_;

  IntegrateOldReadings( midX, midY, inc, vel, xdiff, ydiff, oridiff );
  IntegrateNewReadings( midX, midY, inc, vel );
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

      newpos_x =  -xdiff + (  oldpos_x * m_pTrigTable->GetCos( oridiff ) +
                              oldpos_y * m_pTrigTable->GetSin( oridiff ) );
      newpos_y =  -ydiff + ( -oldpos_x * m_pTrigTable->GetSin( oridiff ) +
                              oldpos_y * m_pTrigTable->GetCos( oridiff ) );

      float angle_to_old_reading = atan2( newpos_y, newpos_x );
      float sqr_distance_to_old_reading = sqr( newpos_x ) + sqr( newpos_y );

      int number_of_old_reading = (int)rad2deg(
          normalize_rad(360.0/m_pLaser->GetNumberOfReadings() * angle_to_old_reading) );
      // This was RCSoftX, now ported to fawkes:
      //int number_of_old_reading = (int) (normalize_degree( ( 360.0/(m_pLaser->GetNumberOfReadings()) ) *
      //         rad2deg(angle_to_old_reading) ) );


      bool SollEintragen = true;

      if ( sqr( m_pLaser->GetReadingLength( number_of_old_reading ) - 0.3 ) > sqr_distance_to_old_reading )
        SollEintragen = false;

      if ( SollEintragen == true ) {
        float posX = midX + (int)((newpos_x*100.0) / ((float)m_CellWidth ));
        float posY = midY + (int)((newpos_y*100.0) / ((float)m_CellHeight ));
        if( posX > 4.0 && posX < m_Width-5
         && posY > 4.0 && posY < m_Height-5 )
          {
          old_readings.push_back( newpos_x );
          old_readings.push_back( newpos_y );
          old_readings.push_back( m_vOldReadings[i+2]+1.0 );

          // 25 cm's in my opinion, that are here: 0.25*100/m_CellWidth
          int size = (int)(((0.25+inc)*100.0)/(float)m_CellWidth);
          integrateObstacle( ellipse_t( posX, posY, size-2, size-2 ) );
        }
      }

    }
  }

  m_vOldReadings.clear();
  m_vOldReadings.reserve( m_InitialHistorySize );

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
  float oldp_x = 1000.0;
  float oldp_y = 1000.0;

  for ( int i = 0; i < numberOfReadings; i++ ) {

    if ( m_pLaser->GetReadingLength(i) >= m_MinimumLaserLength ) {
      // point = transformLaser2Motor(Point(m_pLaser->GetReadingPosX(i), m_pLaser->GetReadingPosY(i)));
      point.x = m_pLaser->GetReadingPosX(i);
      point.y = m_pLaser->GetReadingPosY(i);

      p_x = point.x;
      p_y = point.y;

      if( !((p_x == 0.0) && (p_y == 0.0))
       && sqr(p_x-oldp_x)+sqr(p_y-oldp_y) > sqr(m_EllipseDistance) )
        {
        oldp_x = p_x;
        oldp_y = p_y;
        posX = midX + (int)((p_x*100.0) / ((float)m_CellWidth ));
        posY = midY + (int)((p_y*100.0) / ((float)m_CellHeight ));

        if ( !( posX <= 5 || posX >= m_Width-6 || posY <= 5 || posY >= m_Height-6 ) ) {
          // float dec = max( (sqrt(sqr(p_x)+sqr(p_y))/3.0-1.0), 0.0 );
          // float dec = max((m_pLaser->GetReadingLength(i)/2.0)-1.0, 0.0 );
          float dec = 0.0;

          float height = 0.0;
          height = m_pRoboShape->GetRobotLengthforRad( deg2rad( 90 ) );
          height = std::max( 4.0, ((height + inc - dec)*100.0)/m_CellHeight );

          float rad = normalize_rad( m_pLaser->GetRadiansForReading( i ) );
          float length = 0.0;
          //length = m_pRoboShape->GetRobotLengthforRad( rad );

          if (fabs(normalize_mirror_rad(rad)) < M_PI_2)
            length = m_pRoboShape->GetRobotLengthforRad( deg2rad( 90 ) );
          else
            length = m_pRoboShape->GetRobotLengthforRad( rad );

          length = std::max( 4.0, ((length + inc - dec)*100.0)/m_CellWidth );

          if ( !m_pLaser->IsPipe( rad ) ) {
            integrateObstacle( ellipse_t(  posX, posY, height, length ) );

            if ( !Contained( p_x, p_y ) ) {
              m_vOldReadings.push_back( p_x );
              m_vOldReadings.push_back( p_y );
              m_vOldReadings.push_back( 0.0 );
            }
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
CLaserOccupancyGrid::integrateObstacle( ellipse_t ellipse )
{
  int centerx = (int)(ellipse.center.x);
  int centery = (int)(ellipse.center.y);

  int width = (int)(ellipse.width);
  int height = (int)(ellipse.height);

  std::vector< int > fast_ellipse = ellipse_map->GetEllipse( width, height, m_RobocupMode );

  int posX = 0;
  int posY = 0;

  // i = x offset, i+1 = y offset, i+2 is cost
  for( unsigned int i = 0; i < fast_ellipse.size(); i+=3 ) {
    posY = centery + fast_ellipse[i];
    posX = centerx + fast_ellipse[i+1];

    if( (posX > 0) && (posX < m_Width)
     && (posY > 0) && (posY < m_Height)
     && (m_OccupancyProb[posX][posY] < fast_ellipse[i+2]) )
      {
      m_OccupancyProb[posX][posY] = fast_ellipse[i+2];
    }
  }
}

point_t
CLaserOccupancyGrid::GetLaserPosition()
{
  return m_LaserPosition;
}

} // namespace fawkes
