
/***************************************************************************
 *  robo_laser.h - Provide access to the laser for colli
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

#ifndef __PLUGINS_COLLI_UTILS_ROB_ROBO_LASER_H_
#define __PLUGINS_COLLI_UTILS_ROB_ROBO_LASER_H_

#include "robo_laserpoint.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Laser360Interface;
class Logger;
class Configuration;
class Time;

class Laser
{
 public:

  Laser( fawkes::Laser360Interface* laser,
         fawkes::Logger* logger,
         fawkes::Configuration* config) throw (int);
  ~Laser();

  ///\brief Updates the laserdata.
  int UpdateLaser( );

  ///\brief Returns laser readings length.
  float GetReadingLength( const int number ) const;

  ///\brief Returns laser readings x coordinate.
  float GetReadingPosX  ( const int number ) const;

  ///\brief Returns laser readings y coordinate.
  float GetReadingPosY  ( const int number ) const;

  ///\brief Get number of available laser readings
  int GetNumberOfReadings() const;

  ///\brief Get angle for a laser reading
  float GetRadiansForReading( const int number ) const;

  ///\brief Get current laserdata timestamp
  Time GetCurrentTimestamp() const;

  ///\brief Get the time difference to previous readings
  float TimeDiff() const;

  bool IsPipe( float i ) const;
  bool IsOnlyPipe( float i ) const;


 protected:

  int m_NumberOfReadings; /**< The total number of laser readings */
  float m_Resolution;     /**< The resolution of the laser readings */

  LaserPoint * m_pReadings; /**< Our readings */

 private:

  // METHODS, you don't have to care about

  void CalculateReadings();
  void CalculatePositions();

  // VARIABLES

  // the laser
  fawkes::Laser360Interface *m_pLaserScannerObj;
  Time * newtime;
  Time * oldtime;


  // are the pipes valid???
  bool m_bValidConfig;


  // ignore following readings
  int m_IgnoreFRStart;
  int m_IgnoreFREnd;

  int m_IgnoreRRStart;
  int m_IgnoreRREnd;

  int m_IgnoreRLStart;
  int m_IgnoreRLEnd;

  int m_IgnoreFLStart;
  int m_IgnoreFLEnd;

  // ignore following float readings
  float m_fIgnoreFRStart;
  float m_fIgnoreFREnd;

  float m_fIgnoreRRStart;
  float m_fIgnoreRREnd;

  float m_fIgnoreRLStart;
  float m_fIgnoreRLEnd;

  float m_fIgnoreFLStart;
  float m_fIgnoreFLEnd;


};

} // namespace fawkes

#endif

