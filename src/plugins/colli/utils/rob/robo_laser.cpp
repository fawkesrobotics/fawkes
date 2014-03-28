
/***************************************************************************
 *  robo_laser.cpp - Provide access to the laser for colli
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

#include "robo_laser.h"

#include <interfaces/Laser360Interface.h>
#include <logging/logger.h>
#include <config/config.h>

#include <utils/time/time.h>
#include <utils/math/angle.h>

#include <cmath>
#include <cstdio>

#define MIN_READING_LENGTH 0.01f

using namespace std;

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Laser <plugins/colli/utils/rob/robo_laser.h>
 *  This is a class to access the laserreadings in a
 *   fast and efficient way.
 */

/** Constructor. Initializes the Laser and calculates the number of readings
 * we get ( the size of the pos array ) and initialize it directly.
 * @param laser The laser-interface containing all the laser readings.
 * @param logger The fawkes logger.
 * @param config The fawkes configuration.
 * @exception (int 1) The number of readings given by the laser are
 *  smaller or equal 0.... Perhaps the laser is currently offline. Try again!
 * @exception (int 2) The readings array could not be allocated!
 */
Laser::Laser( Laser360Interface* laser,
              Logger* logger,
              Configuration* config) throw (int)
{
  newtime = new Time();
  newtime->stamp();
  oldtime = new Time();
  m_pLaserScannerObj = laser;

  m_NumberOfReadings = m_pLaserScannerObj->maxlenof_distances();

  if ( m_NumberOfReadings < 1 ) {
    logger->log_error("Laser", "EXCEPTION in Laser(Constructor):  Found less than 1 readings! Throwing exception 1");
    throw ( 1 );
    return;
  }

  try {
      m_pReadings = new LaserPoint( m_NumberOfReadings );
  } catch(...)  {
    throw ( 2 );
    return;
  }
}

/** Desctructor. */
Laser::~Laser( )
{
  delete this->newtime;
  delete this->oldtime;
  delete this->m_pReadings;
}

/** Updates the laserdata.
 *  Call this with the Laser_Clientect in your Loop (the laserobject has to be updated
 *  previously, or you get no new data!).
 *  @return -1 is no new data, so nothing is to do; 0 is ok;  1 is an error occured;
 */
int
Laser::UpdateLaser( )
{
  oldtime->set_time( newtime );

  newtime->stamp();

  m_NumberOfReadings = m_pLaserScannerObj->maxlenof_distances();

  // get all readings
  CalculateReadings();
  CalculatePositions();

  return 0;
}


// put all readings in scan object
void
Laser::CalculateReadings()
{
  float rad = 0.f;
  float rad_inc = (2*M_PI) / m_NumberOfReadings;
  for (int i = 0; i < m_NumberOfReadings; i++) {
    m_pReadings->SetRadians(i, rad);
    m_pReadings->SetLength(i, max(m_pLaserScannerObj->distances(i), 0.f ) );
    rad += rad_inc;
  }
}

void
Laser::CalculatePositions()
{
  for ( int i = 0; i < m_NumberOfReadings; i++ )
    m_pReadings->SetPos(i);
}

/* =========================================================================================== */
/* GETTER STUFF */
/* =========================================================================================== */
/** Return laser readings length.
 * @param number is the number of this reading.
 * @return float is the numbers length.
 */
float
Laser::GetReadingLength( const int number ) const
{
  return m_pReadings->GetLength( number );
}

/** Returns laser readings x coordinate.
 * @param number is the number of this reading.
 * @return float is the numbers x coordinate.
 */
float
Laser::GetReadingPosX( const int number ) const
{
  return m_pReadings->GetPosX(number);
}

/** Returns laser readings y coordinate.
 * @param number is the number of this reading.
 * @return float is the numbers y coordinate.
 */
float
Laser::GetReadingPosY( const int number ) const
{
  return m_pReadings->GetPosY(number);
}

/** Get number of available laser readings
 * @return number of available readings
 */
int
Laser::GetNumberOfReadings() const
{
  return m_NumberOfReadings;
}

/** Get angle for reading
 * @param number Number of the laser reading
 * @return The angle of the reading in radians
 */
float
Laser::GetRadiansForReading( const int number ) const
{
  return m_pReadings->GetRadians(number);
}

/** Get current laserdata timestamp
 * @return current laserdata timestamp
 */
Time
Laser::GetCurrentTimestamp() const
{
  return m_pLaserScannerObj->timestamp();
}

/** Get the time difference to previous readings
 * @return time difference to previous readings in seconds
 */
float
Laser::TimeDiff() const
{
  return ((*newtime - *oldtime).in_sec());
}

/** Checks if a laser reading is valid
 * @param i Angle of the laser readin.
 * @return true if the reading is valid
 */
bool
Laser::IsValid( const int i ) const
{
  return GetReadingLength(i) >= MIN_READING_LENGTH;
}

/** Checks if a laser reading is valid
 * @param i Angle of the laser readin.
 * @return true if the reading is valid
 */
bool
Laser::IsValid( float i ) const
{
  i = normalize_rad( i );
  int reading_num = (int)( (i+0.001) * (m_NumberOfReadings / (2.0*M_PI)) );
  return IsValid( reading_num );
}

} // namespace fawkes
