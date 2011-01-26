
/***************************************************************************
 *  omni_relative.cpp - Implementation of the relative ball model
 *                      for the omni cam
 *
 *  Created: Thu Mar 23 22:00:15 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <cmath>
#include <iostream>
#include <fvmodels/relative_position/omni_relative.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OmniRelative <fvmodels/relative_position/omni_relative.h>
 * Omni vision relative position model.
 */

/** Constructor.
 * @param mirror_model mirror model
 */
OmniRelative::OmniRelative(MirrorModel *mirror_model)
{
  this->mirror_model = mirror_model;

  avg_x = avg_y = avg_x_sum = avg_y_sum = 0.f;
  avg_x_num = avg_y_num = 0;

  ball_x = ball_y = bearing = slope = distance_ball_motor = distance_ball_cam = 0.f;

  // cannot calculate yet
  slope = 0;

  DEFAULT_X_VARIANCE = 36.f;
  DEFAULT_Y_VARIANCE = 25.f;

  /*
  // initial variance for ball pos kf
  kalman_filter = new kalmanFilter2Dim();
  CMatrix<float> initialStateVarianceBall(2,2);
  initialStateVarianceBall[0][0] = DEFAULT_X_VARIANCE;
  initialStateVarianceBall[1][0] = 0.00;
  initialStateVarianceBall[0][1] = 0.00;
  initialStateVarianceBall[1][1] = DEFAULT_Y_VARIANCE;
  kalman_filter->setInitialStateCovariance( initialStateVarianceBall ); 

  // process noise for ball pos kf
  kalman_filter->setProcessCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  kalman_filter->setMeasurementCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  */

}


float
OmniRelative::get_distance() const 
{
  return distance_ball_motor;
}


float
OmniRelative::get_bearing(void) const
{
  return bearing;
}


float
OmniRelative::get_slope() const
{
  return slope;
}


float
OmniRelative::get_y(void) const
{
  return ball_y;
}


float
OmniRelative::get_x(void) const
{
  return ball_x;
}


void
OmniRelative::set_center(float x, float y)
{
  image_x = (unsigned int)roundf(x);
  image_y = (unsigned int)roundf(y);
}


void
OmniRelative::set_center(const center_in_roi_t& c)
{
}


void
OmniRelative::set_radius(float r)
{
}


/** Get radius.
 * @return always 0
 */
float
OmniRelative::get_radius() const
{
  return 0;
}


void
OmniRelative::set_pan_tilt(float pan, float tilt)
{
}


void
OmniRelative::get_pan_tilt(float *pan, float *tilt) const
{
}


const char *
OmniRelative::get_name() const
{
  return "OmniRelative";
}


void
OmniRelative::reset()
{
  last_available = false;
  //kalman_filter->reset();
}


void
OmniRelative::calc()
{

  if ( mirror_model->isValidPoint( image_x, image_y ) ) {
    fawkes::polar_coord_2d_t rel_pos = mirror_model->getWorldPointRelative( image_x, image_y );

    distance_ball_cam = rel_pos.r;
    bearing           = rel_pos.phi;

    // assumes camera to be centered on robot
    ball_x = cos( bearing ) * distance_ball_cam;
    ball_y = sin( bearing ) * distance_ball_cam;

    // applyKalmanFilter();

    distance_ball_motor = sqrt( ball_x * ball_x  +  ball_y * ball_y );
  }
}


bool
OmniRelative::is_pos_valid() const
{
  return mirror_model->isValidPoint( image_x, image_y );
}


void
OmniRelative::calc_unfiltered()
{

  fawkes::polar_coord_2d_t rel_pos = mirror_model->getWorldPointRelative( image_x, image_y );

  distance_ball_cam = rel_pos.r;
  bearing           = rel_pos.phi;

  // cannot calculate yet
  slope = 0;

  // assumes camera to be centered on robot
  ball_x = cos( bearing ) * distance_ball_cam;
  ball_y = sin( bearing ) * distance_ball_cam;

  distance_ball_motor = sqrt( ball_x * ball_x  +  ball_y * ball_y );
}


/*
void
OmniRelative::applyKalmanFilter()
{

  if (last_available) {
    avg_x_sum += ball_x - last_x;
    avg_y_sum += ball_y - last_y;

    ++avg_x_num;
    ++avg_y_num;

    avg_x = avg_x_sum / avg_x_num;
    avg_y = avg_y_sum / avg_y_num;

    rx = (ball_x - avg_x) * distance_ball_cam;
    ry = (ball_y - avg_y) * distance_ball_cam;

    kalman_filter->setMeasurementCovariance( rx * rx, ry * ry );
  } else {
    kalman_filter->setMeasurementCovariance( DEFAULT_X_VARIANCE, DEFAULT_Y_VARIANCE );
  }

  last_x = ball_x;
  last_y = ball_y;

  kalman_filter->setMeasurementX( ball_x );
  kalman_filter->setMeasurementY( ball_y );
  kalman_filter->doCalculation();

  ball_x = kalman_filter->getStateX();
  ball_y = kalman_filter->getStateY();

  if ( isnan( ball_x ) || isinf( ball_x ) ||
       isnan( ball_y ) || isinf( ball_y ) ) {
    // Kalman is wedged, use unfiltered result and reset filter
    kalman_filter->reset();
    ball_x = last_x;
    ball_y = last_y;
  }

}
*/

} // end namespace firevision
