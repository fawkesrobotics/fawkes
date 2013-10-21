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

/* Written by Stefan Jacobs
 * for module Colli-A*
 *
 * Containing Implementation for laser interface.
 *
 */

/***********************************************************************
 *
 * $Id$
 *
 * description:
 *
 * last modification: $Date$
 *         by author: $Author$
 *
 **********************************************************************/

/*! \file Laser.cpp
<pre>
<b>File:</b>          Laser.cpp
<b>Project:</b>       Collision Avoidance, Gruppe 1
<b>Authors:</b>       Stefan Jacobs <Stefan_J@gmx.de>, Gruppe 1
<b>Created:</b>       03/06/2002
<b>Last Revision:</b> 10/06/2002
<b>Contents:</b>      Laser Access.
                       Gets pure Laserdata.
</pre>
*/

#include "robo_laser.h"

#include <interfaces/Laser360Interface.h>
#include <logging/logger.h>
#include <config/config.h>

#include <utils/time/time.h>
#include <utils/math/angle.h>

#include <cmath>
#include <cstdio>
#include <iostream>


using namespace std;

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

// Initialize the Laser by giving the Laser_Client to it, and
// calculating the number of readings we get ( the size of the pos array )
// and initialize it directly
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

  string cfg_path = "/plugins/colli/laser_calibration/";

  if( config->exists((cfg_path + "IgnoreFRStart").c_str()) ) {
    // query config file for ignore-readings
    m_IgnoreFRStart = config->get_int((cfg_path + "IgnoreFRStart").c_str());
    m_IgnoreFREnd = config->get_int((cfg_path + "IgnoreFREnd").c_str());
    m_IgnoreRRStart = config->get_int((cfg_path + "IgnoreRRStart").c_str());
    m_IgnoreRREnd = config->get_int((cfg_path + "IgnoreRREnd").c_str());
    m_IgnoreRLStart = config->get_int((cfg_path + "IgnoreRLStart").c_str());
    m_IgnoreRLEnd = config->get_int((cfg_path + "IgnoreRLEnd").c_str());
    m_IgnoreFLStart = config->get_int((cfg_path + "IgnoreFLStart").c_str());
    m_IgnoreFLEnd = config->get_int((cfg_path + "IgnoreFLEnd").c_str());

    // calculate ignore reagions to floats from degree readings
    m_fIgnoreFRStart = deg2rad(m_IgnoreFRStart);
    m_fIgnoreFREnd   = deg2rad(m_IgnoreFREnd);

    m_fIgnoreRRStart = deg2rad(m_IgnoreRRStart);
    m_fIgnoreRREnd   = deg2rad(m_IgnoreRREnd);

    m_fIgnoreRLStart = deg2rad(m_IgnoreRLStart);
    m_fIgnoreRLEnd   = deg2rad(m_IgnoreRLEnd);

    m_fIgnoreFLStart = deg2rad(m_IgnoreFLStart);
    m_fIgnoreFLEnd   = deg2rad(m_IgnoreFLEnd);

  } else {
    logger->log_warn("Laser", "Config path '%s' not existing, using default ignore-readings calibration", cfg_path.c_str());

    // Config non valid!
    m_fIgnoreFRStart = -1;
    m_fIgnoreFREnd   = -1;

    m_fIgnoreRRStart = -1;
    m_fIgnoreRREnd   = -1;

    m_fIgnoreRLStart = -1;
    m_fIgnoreRLEnd   = -1;

    m_fIgnoreFLStart = -1;
    m_fIgnoreFLEnd   = -1;
  }
}


// Destruct objects is killing all memory allocation
Laser::~Laser( )
{
  delete this->newtime;
  delete this->oldtime;
  delete this->m_pReadings;
}


// Update the laserdata. Call this with the Laser_Clientect in your Loop.
// -1 is no new data, so nothing is to do; 0 is ok; 1 is error
int
Laser::UpdateLaser( )
{
  oldtime->set_time( newtime );

  newtime->stamp();

  m_pLaserScannerObj->read();
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
    m_pReadings->SetLength(i, max(m_pLaserScannerObj->distances(i) - 0.02, 0.0 ) );
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


// -------------------------- //
// LASERREADINGS GETTER       //
// -------------------------- //


// returns laser reading length
float
Laser::GetReadingLength( const int number ) const
{
  return m_pReadings->GetLength( number );
}


// returns laser reading its positions x coordinate
float
Laser::GetReadingPosX( const int number ) const
{
  return m_pReadings->GetPosX(number);
}


// returns laser reading its positions y coordinate
float
Laser::GetReadingPosY( const int number ) const
{
  return m_pReadings->GetPosY(number);
}


// -------------------------- //
// MISC GETTER                //
// -------------------------- //


// returns number of readings available
int
Laser::GetNumberOfReadings() const
{
  return m_NumberOfReadings;
}


// returns current laserdata timestamp
Time
Laser::GetCurrentTimestamp() const
{
  return m_pLaserScannerObj->timestamp();
}


// returns angle for reading
float
Laser::GetRadiansForReading( const int number ) const
{
  return m_pReadings->GetRadians(number);
}


float
Laser::TimeDiff() const
{
  return ((*newtime - *oldtime).in_sec());
}


// the famous is pipe method
bool
Laser::IsPipe( float i ) const
{
  i = normalize_rad( i );

  int reading_num = (int)( ( (i+0.001) / (2.0*M_PI)) * m_NumberOfReadings );
  // look below for explanation

  if ( GetReadingLength( reading_num ) == 0.0 )
    return true;

  if( ( (i >= m_fIgnoreFRStart) && (i <= m_fIgnoreFREnd) )
   || ( (i >= m_fIgnoreRRStart) && (i <= m_fIgnoreRREnd) )
   || ( (i >= m_fIgnoreRLStart) && (i <= m_fIgnoreRLEnd) )
   || ( (i >= m_fIgnoreFLStart) && (i <= m_fIgnoreFLEnd) ) ) {
    // Meist sind die Kabel das Problem!!!!
    // Wenn Danger if turning im Colli erscheint, erst Kabel checken.
    //      cout << "Got pipe at " << i << endl;
    return true;
  }

  return false;
}


bool
Laser::IsOnlyPipe( float i ) const
{
  i = normalize_rad( i );

  //  int reading_num = (int)( ( (i+0.001) / (2.0*M_PI)) * m_NumberOfReadings );
  // -0.01 because of numerical problems...
  // numbers were before something like 1 2 3 5 6 6 7
  //    cout << i << " has reading num " << reading_num << " and length : " << GetReadingLength( reading_num ) << endl;

//   if ( ( (i > (26.0*M_PI)/180.0) && (i < (37.0*M_PI)/180.0) )
//        || ( (i > (95.0*M_PI)/180.0) && (i < (115.0*M_PI)/180.0) )
//        || ( (i > ((246.0-360.0)*M_PI)/180.0) && (i < ((263.0-360.0)*M_PI)/180.0) )
//        || ( (i > ((323.0-360.0)*M_PI)/180.0) && (i < ((334.0-360.0)*M_PI)/180.0) )
//       )

  if( ( (i > m_fIgnoreFRStart) && (i < m_fIgnoreFREnd) )
   || ( (i > m_fIgnoreRRStart) && (i < m_fIgnoreRREnd) )
   || ( (i > m_fIgnoreRLStart) && (i < m_fIgnoreRLEnd) )
   || ( (i > m_fIgnoreFLStart) && (i < m_fIgnoreFLEnd ) ) ) {
    // Meist sind die Kabel das Problem!!!!
    // Wenn Danger if turning im Colli erscheint, erst Kabel checken.
    //      cout << "Got pipe at " << i << endl;
    return true;
  }

  return false;
}

} // namespace fawkes