
/***************************************************************************
 *  sensproc_thread.cpp - Laser HT sensor processing thread
 *
 *  Created: Sat Jul 04 21:35:37 2009 (RoboCup 2009, Graz)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: sensor_thread.cpp 2627 2009-06-25 18:08:09Z tim $
 *
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

#include "sensproc_thread.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/ObjectPositionInterface.h>

#include <utils/math/angle.h>

using namespace fawkes;

/** @class LaserHtSensorProcThread "sensproc_thread.h"
 * Laser Hough Transform sensor processing thread.
 * This thread integrates into the Fawkes main loop at the sensor processing
 * hook and uses the Hough Transform to extract shapes.
 * @author Tim Niemueller
 */


/** Constructor. */
LaserHtSensorProcThread::LaserHtSensorProcThread()
  : Thread("LaserHtSensorProcThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


void
LaserHtSensorProcThread::init()
{
  //__cfg_error_threshold = config->get_float("/plugins/laserline/error_threshold");

  __laser360_if = NULL;
  __line_if   = NULL;
  try {
    __laser360_if = blackboard->open_for_reading<Laser360Interface>("Laser");
    __line_if = blackboard->open_for_writing<ObjectPositionInterface>("LaserLine");
    __line_if->set_object_type(ObjectPositionInterface::TYPE_LINE);
  } catch (Exception &e) {
    blackboard->close(__laser360_if);
    blackboard->close(__line_if);
    throw;
  }
}


void
LaserHtSensorProcThread::finalize()
{
  blackboard->close(__laser360_if);
  blackboard->close(__line_if);
}


void
LaserHtSensorProcThread::loop()
{
  __laser360_if->read();
  float *distances = __laser360_if->distances();

  std::vector<laser_reading_t> readings;

  for (unsigned int i = 0; i < 30; ++i) {
    if (distances[330+i] != 0.0) {
      float angle = deg2rad(((330 + i) /* * 0.5 */));
      float dist  = distances[330 + i];
      float x     = dist *  sin(angle);
      float y     = dist * -cos(angle);
      laser_reading_t l = {angle, dist, x, y};
      readings.push_back(l);
    }

    if (distances[i] != 0.0) {
      float angle = deg2rad(i /* * 0.5*/);
      float dist  = distances[i];
      float x     = dist *  sin(angle);
      float y     = dist * -cos(angle);
      //float x     = dist *  sin(angle);
      //float y     = dist * -cos(angle);
      laser_reading_t l = {angle, dist, x, y};
      readings.push_back(l);
    }
  }

  if (readings.empty()) {
    logger->log_debug(name(), "No valid readings");
    return;
  }

  float a = 0, b = 0, e = 0;
  fit_line(readings, 0, a, b, e);

  float lx = readings.front().x;
  float ly = a * lx + b;
  float rx = readings.back().x;
  float ry = a * rx + b;

  logger->log_debug(name(), "lx=%f  ly=%f  rx=%f  ry=%f  a=%f  b=%f  e=%f",
		    lx, ly, rx, ry, a, b, e);

  __line_if->set_world_x(-ly);
  __line_if->set_world_y(-lx);

  __line_if->set_relative_x(-ry);
  __line_if->set_relative_y(-rx);

  __line_if->set_slope(-a);
  __line_if->set_bearing(atan2f(-a, 1));
  __line_if->set_distance(b);
  __line_if->set_roll(e);

  __line_if->write();
}

#define sqr(x) ((x) * (x))

void
LaserHtSensorProcThread::fit_line(const std::vector<laser_reading_t> &points,
				const unsigned int first_index,
				float &a, float &b, float &least_square_error)
{
  size_t n = points.size();
  float sum_x = 0.0, sum_xy = 0.0, sum_y = 0.0, sum_xx = 0.0;

  float x, y;
  register float e = 0.0;
  for (size_t i = first_index; i < n; ++i ) {
    x = points[i].x;
    y = points[i].y;

    sum_y  += y;
    sum_x  += x;
    sum_xy += x*y;
    sum_xx += x*x;

  }

  b = ( sum_y * sum_xx - sum_x * sum_xy ) / ( n * sum_xx - sum_x * sum_x );
  a = ( n * sum_xy - sum_x * sum_y ) / ( n * sum_xx - sum_x * sum_x );

  
  for (size_t i = first_index; i < n; ++i ) {
    // Compute least-square error if desired
    e += sqr( points[i].y - (points[i].x*a + b) );
  }

  least_square_error = e;
}
