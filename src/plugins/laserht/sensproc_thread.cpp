
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
#include "hough_transform.h"

#include <interfaces/Laser360Interface.h>
#include <interfaces/ObjectPositionInterface.h>

#include <utils/math/angle.h>
#include <utils/math/coord.h>

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

  __ht = new HoughTransform(2);

  __num_vals   = 24;
  __angle_step = 360 / __num_vals;
  __r_scale    = 0.01;
  __values = new int*[__num_vals];
  for (unsigned int i = 0; i < __num_vals; ++i) {
    __values[i] = new int[2];
  }
}


void
LaserHtSensorProcThread::finalize()
{
  blackboard->close(__laser360_if);
  blackboard->close(__line_if);

  delete __ht;
  for (unsigned int i = 0; i < __num_vals; ++i) {
    delete[] __values[i];
  }
  delete[] __values;
}


void
LaserHtSensorProcThread::loop()
{
  __laser360_if->read();
  float *distances = __laser360_if->distances();
  const size_t num_dist = __laser360_if->maxlenof_distances();

  __ht->reset();

  for (size_t i = 0; i < num_dist; ++i) {
    // generate candidates
    if (distances[i] > 0) {
      for (unsigned int j = 0; j < __num_vals; ++j) {
	float phi   = deg2rad(i);
	float theta = deg2rad(j * __angle_step);
	float x, y;
	polar2cart2d(phi, distances[i], &x, &y);
	float r   = fabs(x * cos(theta) + y * sin(theta)) / __r_scale;
	__values[j][0] = (int)roundf(r);
	__values[j][1] = j * __angle_step;
	/*
	if (__values[j][0] == 0) {
	  logger->log_debug(name(), "j=%zu  phi=%f  r=%f v[0]=%i  v[1]=%i",
			    j, phi, r, __values[j][0], __values[j][1]);
	}
	*/
      }
      __ht->process(__values, __num_vals);
    }
  }

  int max_values[2];
  unsigned int max_count = __ht->max(max_values);
  logger->log_debug(name(), "Max count: %u  (%i, %i)", max_count, max_values[0],
		    max_values[1]);

  float phi = deg2rad(max_values[1]);
  float r   = max_values[0] * __r_scale;
  float x1, y1, x2, y2;
  polar2cart2d(phi, r, &x1, &y1);

  float alpha, y_factor = 1;
  if ( ((max_values[1] >= 0) && (max_values[1] < 90)) ||
       (max_values[1] >= 270) ) {
    y_factor = -1;
    alpha = deg2rad(90 - (max_values[1] % 90));
  } else {
    alpha = deg2rad((max_values[1] % 90));
  }
  float dx   = 1 * cos(alpha);
  float dy   = 1 * y_factor * sin(alpha);
  x2 = x1 + dx;
  y2 = y1 + dy;

  logger->log_debug(name(), "r=%f  phi=%f  p1=(%f,%f)  p2=(%f,%f)",
		    r, phi, x1, y1, x2, y2);
  logger->log_debug(name(), "Tree depth: %u  num_nodes: %u\n",
		    __ht->root()->depth(), __ht->root()->num_nodes());
  //logger->log_debug(name(), "Line from (%f,%f) to (%f,%f)", x1, y1, x2, y2);

  __line_if->set_world_x(x1);
  __line_if->set_world_y(y1);

  __line_if->set_relative_x(x2);
  __line_if->set_relative_y(y2);

  __line_if->write();

  /*
  float x[2], y[2];
  x[0] =  1; y[0] = 1;
  x[1] = -1; y[0] = 1;

  for (unsigned int i = 0; i < 2; ++i) {
    for (unsigned int j = 0; j < __num_vals; ++j) {
      float theta = deg2rad(j * 20);
      float r   = fabs(x[i] * cos(theta) + y[i] * sin(theta));
      r *= 100.;
      __values[j][0] = (int)roundf(r);
      __values[j][1] = j * 20;
      logger->log_debug(name(), "i=%u  j=%u  theta=%f  r=%f v[0]=%i  v[1]=%i",
			i, j, theta, r, __values[j][0], __values[j][1]);
    }
    __ht->process(__values, __num_vals);
  }

  int max_values[2];
  unsigned int max_count = __ht->max(max_values);
  logger->log_debug(name(), "Max count: %u  (%i, %i)", max_count, max_values[0],
		    max_values[1]);

  logger->log_debug(name(), "Tree depth: %u  num_nodes: %u\n",
		    __ht->root()->depth(), __ht->root()->num_nodes());
  */
  /*
  std::vector<laser_reading_t> readings;

  for (unsigned int i = 0; i < 30; ++i) {
    if (distances[330+i] != 0.0) {
      float angle = deg2rad(((330 + i) / * 0.5 /));
      float dist  = distances[330 + i];
      float x     = dist *  sin(angle);
      float y     = dist * -cos(angle);
      laser_reading_t l = {angle, dist, x, y};
      readings.push_back(l);
    }

    if (distances[i] != 0.0) {
      float angle = deg2rad(i / * 0.5/);
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
*/
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
