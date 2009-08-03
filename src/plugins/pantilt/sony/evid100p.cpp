
/***************************************************************************
 *  evid100p.cpp - Sony EviD100P Visca wrapper
 *
 *  Created: Sun Jun 21 13:10:51 2009
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "evid100p.h"

#include <core/exceptions/software.h>
#include <utils/math/angle.h>

using namespace fawkes;

/** @class SonyEviD100PVisca "evid100p.h"
 * Sony EviD100P Visca controller.
 * This sub-class using the Visca protocol contains some constants specific
 * for the Sony EviD100P camera.
 * @author Tim Niemueller
 */

/** Maximum pan. */
const int   SonyEviD100PVisca::MAX_PAN       =  1440;
/** Minimum pan. */
const int   SonyEviD100PVisca::MIN_PAN       = -1439;
/** Max Tilt. */
const int   SonyEviD100PVisca::MAX_TILT      =   360;
/** Min tilt .*/
const int   SonyEviD100PVisca::MIN_TILT      = - 359;

/** Max pan in degrees. */
const float SonyEviD100PVisca::MAX_PAN_DEG   =  100.f;
/** Min pan in degrees. */
const float SonyEviD100PVisca::MIN_PAN_DEG   = -100.f;
/** Max tilt in degrees. */
const float SonyEviD100PVisca::MAX_TILT_DEG  =   25.f;
/** Min tilt in degrees. */
const float SonyEviD100PVisca::MIN_TILT_DEG  = - 25.f;

/** Max pan in rad. */
const float SonyEviD100PVisca::MAX_PAN_RAD   = deg2rad(MAX_PAN_DEG);
/** Min pan in rad. */
const float SonyEviD100PVisca::MIN_PAN_RAD   = deg2rad(MIN_PAN_DEG);
/** Max tilt in rad. */
const float SonyEviD100PVisca::MAX_TILT_RAD  = deg2rad(MAX_TILT_DEG);
/** Min tilt in rad. */
const float SonyEviD100PVisca::MIN_TILT_RAD  = deg2rad(MIN_TILT_DEG);

/** Pan steps per degree */
const float SonyEviD100PVisca::PAN_STEPS_PER_DEG  = MAX_PAN  / MAX_PAN_DEG;
/** Tilt steps per degree */
const float SonyEviD100PVisca::TILT_STEPS_PER_DEG = MAX_TILT / MAX_TILT_DEG;

/** Pan steps per rad */
const float SonyEviD100PVisca::PAN_STEPS_PER_RAD  = MAX_PAN  / MAX_PAN_RAD;
/** Tilt steps per rad */
const float SonyEviD100PVisca::TILT_STEPS_PER_RAD = MAX_TILT / MAX_TILT_RAD;

/** Pastel effect. */
const unsigned int SonyEviD100PVisca::EFFECT_PASTEL   = 1;
/** Negative effect. */
const unsigned int SonyEviD100PVisca::EFFECT_NEGATIVE = 2;
/** Sepia effect. */
const unsigned int SonyEviD100PVisca::EFFECT_SEPIA    = 3;
/** B/W effect. */
const unsigned int SonyEviD100PVisca::EFFECT_BW       = 4;
/** Solarize effect. */
const unsigned int SonyEviD100PVisca::EFFECT_SOLARIZE = 5;
/** Mosaic effect. */
const unsigned int SonyEviD100PVisca::EFFECT_MOSAIC   = 6;
/** Slim effect. */
const unsigned int SonyEviD100PVisca::EFFECT_SLIM     = 7;
/** Stretch effect. */
const unsigned int SonyEviD100PVisca::EFFECT_STRETCH  = 8;


/** Speed table for supported pan speed values in radians.
 * Has been created empirically.
 */
const float SonyEviD100PVisca::SPEED_TABLE_PAN[] =
  {0.03548, 0.04138, 0.05319, 0.06497, 0.08262, 0.10608, 0.12951, 0.15865,
   0.19933, 0.24535, 0.30159, 0.35137, 0.43540, 0.53611, 0.67246, 0.81519,
   0.99870, 1.20673, 1.45304, 1.70703, 1.99278, 2.25729, 2.44293, 2.71852};

/** Speed table for supported tilt speed values in radians.
 * Has been created empirically.
 */
const float SonyEviD100PVisca::SPEED_TABLE_TILT[] =
  {0.03541, 0.04127, 0.05298, 0.06449, 0.08195, 0.10480, 0.12741, 0.15535,
   0.19356, 0.23685, 0.28438, 0.33367, 0.41066, 0.49517, 0.59622, 0.71474,
   0.83085, 0.97431, 1.08745, 1.20977};


/** Constructor.
 * @param device_file serial device file (e.g. /dev/ttyUSB0)
 * @param def_timeout_ms default read timeout, used if no specific timeout
 * is passed
 * @param blocking true to make gathering pan/tilt information wait for
 * the reponse, false to be able to split the operation
 */
SonyEviD100PVisca::SonyEviD100PVisca(const char *device_file,
				     unsigned int def_timeout_ms,
				     bool blocking)
  : Visca(device_file, def_timeout_ms, blocking)
{
}


/** Destructor. */
SonyEviD100PVisca::~SonyEviD100PVisca()
{
}


/** Set pan/tilt in radians.
 * @param pan pan value in radians
 * @param tilt tilt value in radians
 */
void
SonyEviD100PVisca::set_pan_tilt_rad(float pan, float tilt)
{
  if ( (pan < MIN_PAN_RAD) || (pan > MAX_PAN_RAD) ) {
    throw OutOfBoundsException("Illegal pan value", pan, MIN_PAN_RAD, MAX_PAN_RAD);
  }
  if ( (tilt < MIN_TILT_RAD) || (tilt > MAX_TILT_RAD) ) {
    throw OutOfBoundsException("Illegal tilt value", tilt, MIN_TILT_RAD, MAX_TILT_RAD);
  }

  int tpan = 0, ttilt = 0;

  tpan = (int)rint(  pan  * PAN_STEPS_PER_RAD  );
  ttilt = (int)rint( tilt * TILT_STEPS_PER_RAD );

  set_pan_tilt(tpan, ttilt);
}


/** Get pan/tilt in radians.
 * @param pan upon return contains the current pan value
 * @param tilt upone return contains the current tilt value
 */
void
SonyEviD100PVisca::get_pan_tilt_rad(float &pan, float &tilt)
{
  int tpan = 0, ttilt = 0;
  get_pan_tilt(tpan, ttilt);

  pan  = tpan  / PAN_STEPS_PER_RAD;
  tilt = ttilt / PAN_STEPS_PER_RAD;
}


/** Set speed given in rad/sec.
 * Note that not the exact speed is taken, but rather the closes equivalent in
 * motor ticks is taken.
 * @param pan_speed desired pan speed in rad/sec
 * @param tilt_speed desired tilt speed in rad/sec
 * @exception OutOfBoundsException thrown if desired speed is out of range
 */
void
SonyEviD100PVisca::set_speed_radsec(float pan_speed, float tilt_speed)
{
  if ( (pan_speed < 0) ||
       (pan_speed > SPEED_TABLE_PAN[SONY_EVID100P_NUM_PAN_SPEEDS - 1]) ) {
    throw OutOfBoundsException("Illegal pan speed", pan_speed, 0,
			       SPEED_TABLE_PAN[SONY_EVID100P_NUM_PAN_SPEEDS - 1]);
  }
  if ( (tilt_speed < 0) ||
       (tilt_speed > SPEED_TABLE_TILT[SONY_EVID100P_NUM_TILT_SPEEDS - 1]) ) {
    throw OutOfBoundsException("Illegal tilt speed", tilt_speed, 0,
			       SPEED_TABLE_TILT[SONY_EVID100P_NUM_TILT_SPEEDS - 1]);
  }

  unsigned int pan_ind = SONY_EVID100P_NUM_PAN_SPEEDS - 1;
  float min_pan_dist = SPEED_TABLE_PAN[pan_ind];
  float last_dist = min_pan_dist;;
  for (unsigned int i = 0; i < SONY_EVID100P_NUM_PAN_SPEEDS; ++i) {
    float dist = 0;
    if ( (dist = fabs(pan_speed - SPEED_TABLE_PAN[i])) < min_pan_dist ) {
      min_pan_dist = dist;
      pan_ind = i;
    } else if (dist > last_dist) {
      break; // times are growing now, found best
    }
    last_dist = dist;
  }

  unsigned int tilt_ind = SONY_EVID100P_NUM_TILT_SPEEDS - 1;
  float min_tilt_dist = SPEED_TABLE_TILT[tilt_ind];
  last_dist = min_tilt_dist;
  for (unsigned int i = 0; i < SONY_EVID100P_NUM_TILT_SPEEDS; ++i) {
    float dist = 0;
    if ( (dist = fabs(tilt_speed - SPEED_TABLE_TILT[i])) < min_tilt_dist ) {
      min_tilt_dist = dist;
      tilt_ind = i;
    } else if (dist > last_dist) {
      break; // times are growing now, found best
    }
    last_dist = dist;
  }

  set_pan_tilt_speed(pan_ind, tilt_ind);
}


/** Get current speed in rad/sec.
 * @param pan_speed upon return contains pan speed in rad/sec
 * @param tilt_speed upon return contains tilt speed in rad/sec
 */
void
SonyEviD100PVisca::get_speed_radsec(float &pan_speed, float &tilt_speed)
{
  unsigned char ps, ts;
  get_pan_tilt_speed(ps, ts);
  pan_speed = SPEED_TABLE_PAN[ps - 1];
  tilt_speed = SPEED_TABLE_TILT[ps - 1];
}


/** Get speed limits.
 * @param pan_min minimum pan speed possible
 * @param pan_max maximum pan speed possible
 * @param tilt_min minimum tilt speed possible
 * @param tilt_max maximum tilt speed possible
 */
void
SonyEviD100PVisca::get_speed_limits(float &pan_min, float &pan_max,
				    float &tilt_min, float &tilt_max)
{
  pan_min = SPEED_TABLE_PAN[0];
  pan_max = SPEED_TABLE_PAN[SONY_EVID100P_NUM_PAN_SPEEDS - 1];
  tilt_min = SPEED_TABLE_TILT[0];
  tilt_max = SPEED_TABLE_TILT[SONY_EVID100P_NUM_TILT_SPEEDS - 1];
}
