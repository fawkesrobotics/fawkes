
/***************************************************************************
 *  sensproc_thread.cpp - Laser HT sensor processing thread
 *
 *  Created: Sat Jul 04 21:35:37 2009 (RoboCup 2009, Graz)
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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
#include <interfaces/VisualDisplay2DInterface.h>

#include <utils/math/angle.h>
#include <utils/math/coord.h>
#ifdef LASERHT_TIMETRACKER
#  include <utils/time/tracker.h>
#endif

#include <cstdlib>

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
  __laser360_if = NULL;
  __visdisp_if  = NULL;
  __line_if     = NULL;

  __cfg_num_samples    = config->get_uint("/plugins/laserht/line/num_samples");
  __cfg_r_scale        = config->get_float("/plugins/laserht/line/r_scale");
  __cfg_laser_ifid     = config->get_string("/plugins/laserht/laser_interface_id");
  __cfg_enable_disp    = config->get_bool("/plugins/laserht/line/enable_display");
  __cfg_vote_threshold = config->get_uint("/plugins/laserht/line/vote_threshold");
  __cfg_dist_threshold = config->get_float("/plugins/laserht/line/dist_threshold");
  __cfg_fitting_error_threshold = config->get_float("/plugins/laserht/line/fitting_error_threshold");

  __laser360_if = NULL;
  __line_if   = NULL;
  try {
    __laser360_if = blackboard->open_for_reading<Laser360Interface>(__cfg_laser_ifid.c_str());
    if (__cfg_enable_disp) {
      __visdisp_if = blackboard->open_for_reading<VisualDisplay2DInterface>("LaserGUI");
    }
    __line_if = blackboard->open_for_writing<ObjectPositionInterface>("LaserLine");
    __line_if->set_object_type(ObjectPositionInterface::TYPE_LINE);
  } catch (Exception &e) {
    blackboard->close(__laser360_if);
    blackboard->close(__line_if);
    throw;
  }

  __ht = new HoughTransform(2);

  __num_vals   = __cfg_num_samples;
  __angle_step = 180.f / __num_vals;
  __r_scale    = __cfg_r_scale;
  __values = new int*[__num_vals];
  for (unsigned int i = 0; i < __num_vals; ++i) {
    __values[i] = new int[2];
  }

#ifdef LASERHT_TIMETRACKER
  __tt          = new TimeTracker();
  __tt_loop     = 0;
  __ttc_reset   = __tt->add_class("Reset");
  __ttc_process = __tt->add_class("Processing");
  __ttc_fitting = __tt->add_class("Fitting");
  __ttc_total   = __tt->add_class("Total");
#endif
}


void
LaserHtSensorProcThread::finalize()
{
  __line_if->set_valid(false);
  __line_if->write();

  blackboard->close(__laser360_if);
  blackboard->close(__visdisp_if);
  blackboard->close(__line_if);

  delete __ht;
  for (unsigned int i = 0; i < __num_vals; ++i) {
    delete[] __values[i];
  }
  delete[] __values;
}


void
LaserHtSensorProcThread::line_points_from_params(float r, float phi,
						 float &x1, float &y1,
						 float &x2, float &y2)
{
  float phi_rad  = deg2rad(phi);
  float phi_mod  = phi - (floorf(phi / 90.) * 90);
  float r_scaled = r * __r_scale;
  float tx, ty;
  polar2cart2d(phi_rad, r_scaled, &tx, &ty);
  x1 = tx;
  y1 = ty;

  float alpha, y_factor = 1;
  if ( ((phi >= 0) && (phi < 90)) ||
       (phi >= 270) ) {
    y_factor = -1;
    alpha = deg2rad(90 - phi_mod);
  } else {
    alpha = deg2rad(phi_mod);
  }
  float dx   = 1 * cos(alpha);
  float dy   = 1 * y_factor * sin(alpha);
  x2 = x1 + dx;
  y2 = y1 + dy;
}


void
LaserHtSensorProcThread::loop()
{
  __laser360_if->read();
  float *distances = __laser360_if->distances();
  const size_t num_dist = __laser360_if->maxlenof_distances();

#ifdef LASERHT_TIMETRACKER
  __tt->ping_start(__ttc_total);
  __tt->ping_start(__ttc_reset);
#endif
  __ht->reset();
#ifdef LASERHT_TIMETRACKER
  __tt->ping_end(__ttc_reset);
  __tt->ping_start(__ttc_process);
#endif

  for (size_t i = 0; i < num_dist; ++i) {
    // generate candidates
    if (distances[i] > 0) {
      for (unsigned int j = 0; j < __num_vals; ++j) {
	float phi   = deg2rad(i);
	float theta = deg2rad(j * __angle_step);
	float x, y;
	polar2cart2d(phi, distances[i], &x, &y);
	float r   = x * cos(theta) + y * sin(theta);
	r /= __r_scale;
	__values[j][0] = (int)roundf(r);
	__values[j][1] = (int)roundf(j * __angle_step);
      }
      __ht->process(__values, __num_vals);
    }
  }
#ifdef LASERHT_TIMETRACKER
  __tt->ping_end(__ttc_process);
#endif

  int max_values[2];
  unsigned int max_count = __ht->max(max_values);

  if (max_count >= __cfg_vote_threshold) {
    float x1, y1, x2, y2;
    line_points_from_params(max_values[0], max_values[1], x1, y1, x2, y2);

    try {
      if (__cfg_enable_disp && __visdisp_if->has_writer()) {
	__visdisp_if->msgq_enqueue(new VisualDisplay2DInterface::DeleteAllMessage());
	float x[2] = {x1, x2};
	float y[2] = {y1, y2};
	unsigned char color[4] = {0, 255, 0, 255};
	VisualDisplay2DInterface::AddCartLineMessage *lm;
	lm = new VisualDisplay2DInterface::AddCartLineMessage(x, y,
							      VisualDisplay2DInterface::LS_SOLID, color);
	__visdisp_if->msgq_enqueue(lm);

	/*
	color[0] = 0;
	color[1] = 255;

	int *values;
	unsigned int num_v = __ht->filter(&values, __cfg_vote_threshold);
	for (unsigned int i = 0; i < num_v; ++i) {
	  line_points_from_params(values[i * 2 + 0], values[i * 2 + 1], x1, y1, x2, y2);
	  float x[2] = {x1, x2};
	  float y[2] = {y1, y2};
	  lm = new VisualDisplay2DInterface::AddCartLineMessage(x, y, 
								VisualDisplay2DInterface::LS_SOLID, color);
	  __visdisp_if->msgq_enqueue(lm);
	}
	free(values);
	*/
      }
    } catch (Exception &e) {} // ignored

    // Calculate points contributing to the primary line
    float theta  = deg2rad(max_values[1]);
    float alpha  = 0.5 * M_PI - theta;
    float r_scaled = max_values[0] * __r_scale;
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);
    float threshold = __cfg_dist_threshold;
    float r_min = r_scaled - threshold;
    float r_max = r_scaled + threshold;

    bool  first_x_minmax = true;
    float x_min = 0, x_max = 0;

    std::vector<laser_reading_t> readings;

    for (size_t i = 0; i < num_dist; ++i) {
      // calculate r with r(theta) = x_i * cos(theta) + y_i * sin(theta)
      if (distances[i] > 0) {
	float x, y;
	float phi = deg2rad(i);
	polar2cart2d(phi, distances[i], &x, &y);
	float r = x * cos(theta) + y * sin(theta);

	if ( (r >= r_min) && (r <= r_max) ) {
	  // valid!
	  /* generally too much, might be useful for debugging
	  if (__cfg_enable_disp && __visdisp_if->has_writer()) {
	    float xp[2] = {0, x};
	    float yp[2] = {0, y};
	    unsigned char color[4] = {0, 0, 255, 255};
	    VisualDisplay2DInterface::AddCartLineMessage *lm;
	    lm = new VisualDisplay2DInterface::AddCartLineMessage(xp, yp, 
								  VisualDisplay2DInterface::LS_SOLID, color);
	    __visdisp_if->msgq_enqueue(lm);
	  }
	  */

	  // now rotate all values to get a horizontal line, otherwise
	  // line fitting could fail (for vertical lines)

	  // note: x_rot = x * cos_alpha - y * sin_alpha
	  //       y_rot = x * sin_alpha + y * cos_alpha

	  // Therefore, to rotate line to horizontal position, i.e. by alpha
	  float x_rot = x * cos_alpha - y * sin_alpha;
	  float y_rot = x * sin_alpha + y * cos_alpha;

	  laser_reading_t l = {phi, distances[i], x_rot, y_rot};
	  readings.push_back(l);
	  if (first_x_minmax) {
	    first_x_minmax = false;
	    x_min = x_rot;
	    x_max = x_rot;
	  } else {
	    if (x_rot < x_min) x_min = x_rot;
	    if (x_rot > x_max) x_max = x_rot;
	  }
	}
      }
    }

#ifdef LASERHT_TIMETRACKER
  __tt->ping_start(__ttc_fitting);
#endif
    // fit line through determined readings
    float a = 0, b = 0, e = 0;
    fit_line(readings, 0, a, b, e);
#ifdef LASERHT_TIMETRACKER
  __tt->ping_end(__ttc_fitting);
#endif

    if ( e <= __cfg_fitting_error_threshold ) {
      // calculate y values of min and max point
      float y_min = a * x_min + b;
      float y_max = a * x_max + b;

      // rotate min/max points back, remember rotation from above,
      // and note: sin(-alpha) = - sin(alpha)
      //           cos(-alpha) =   cos(alpha)
      float x_min_rot = x_min * cos_alpha + y_min * sin_alpha;
      float y_min_rot = y_min * cos_alpha - x_min * sin_alpha;
      float x_max_rot = x_max * cos_alpha + y_max * sin_alpha;
      float y_max_rot = y_max * cos_alpha - x_max * sin_alpha;

      // calculate parameters of fitted line
      float alpha_fit = atan2f(y_max_rot - y_min_rot,
			       x_max_rot - x_min_rot);
      if ( (theta <= 0.5 * M_PI) ||
      	   ((theta >= M_PI) && (theta <= 1.5 * M_PI)) ) {
	alpha_fit = 0.5 * M_PI + alpha_fit;
      }
      float theta_fit = floorf(theta / (0.5*M_PI)) * 0.5*M_PI + alpha_fit;
      float r_fit     = x_min_rot * cos(theta_fit) + y_min_rot * sin(theta_fit);

      if (__cfg_enable_disp && __visdisp_if->has_writer()) {
	float x1, y1, x2, y2;
	line_points_from_params(r_fit / __r_scale, rad2deg(theta_fit),
				x1, y1, x2, y2);

	try {
	  __visdisp_if->msgq_enqueue(new VisualDisplay2DInterface::DeleteAllMessage());
	  float x[2] = {x1, x2};
	  float y[2] = {y1, y2};
	  unsigned char color[4] = {0, 0, 255, 255};
	  VisualDisplay2DInterface::AddCartLineMessage *lm;
	  lm = new VisualDisplay2DInterface::AddCartLineMessage(x, y,
								VisualDisplay2DInterface::LS_SOLID, color);
	  __visdisp_if->msgq_enqueue(lm);
	} catch (Exception &e) {} // ignored
      }


      // write data to interface
      __line_if->set_world_x(x_min_rot);
      __line_if->set_world_y(y_min_rot);

      __line_if->set_relative_x(x_max_rot);
      __line_if->set_relative_y(y_max_rot);

      __line_if->set_bearing(theta_fit);
      __line_if->set_distance(r_fit);

      __line_if->set_roll(e);
      __line_if->set_visible(true);
    } else {
      logger->log_debug(name(), "Fitting error above threshold: %f > %f",
			e, __cfg_fitting_error_threshold);
      __line_if->set_roll(e);
      __line_if->set_visible(false);
    }
  } else {
    logger->log_debug(name(), "Votes below threshold: %u < %u",
		      max_count, __cfg_vote_threshold);
    __line_if->set_visible(false);
  }

  __line_if->set_valid(true);
  __line_if->write();
#ifdef LASERHT_TIMETRACKER
  __tt->ping_end(__ttc_total);
  if (++__tt_loop >= 100) {
    __tt->print_to_stdout();
    __tt_loop = 0;
  }
#endif
}

#define sqr(x) ((x) * (x))

void
LaserHtSensorProcThread::fit_line(const std::vector<laser_reading_t> &points,
				const unsigned int first_index,
				float &a, float &b, float &least_square_error)
{
  const size_t n = points.size();
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
