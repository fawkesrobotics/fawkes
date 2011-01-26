
/****************************************************************************
 *  ball_trigo.cpp - Ball relpos for pan/tilt camera using basic trigonometry
 *
 *  Created: Mon Mar 23 10:03:48 2009
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#include <fvmodels/relative_position/ball_trigo.h>
#include <utils/math/angle.h>

#include <cmath>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BallTrigoRelativePos <fvmodels/relative_position/ball_trigo.h>
 * Relative ball position model for pan/tilt camera.
 * This uses basic trigonometry to calculate the position of the ball given
 * only the center of the ball in the image as variable parameters, and the
 * camera parameters as static parameters.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param image_width width of image in pixels
 * @param image_height height of image in pixels
 * @param camera_height height of camera in meters
 * @param camera_offset_x camera offset of the motor axis in x direction
 * @param camera_offset_y camera offset of the motor axis in y direction
 * @param camera_base_pan camera base pan in rad
 * @param camera_base_tilt camera base tilt in rad
 * @param horizontal_angle horizontal viewing angle (in degree)
 * @param vertical_angle vertical viewing angle (in degree)
 * @param ball_circumference ball circumference
 */
BallTrigoRelativePos::BallTrigoRelativePos(unsigned int image_width,
					   unsigned int image_height,
					   float camera_height,
					   float camera_offset_x,
					   float camera_offset_y,
					   float camera_base_pan,
					   float camera_base_tilt,
					   float horizontal_angle,
					   float vertical_angle,
					   float ball_circumference)
{
  __image_width        = image_width;
  __image_width_2      = __image_width / 2;
  __image_height       = image_height;
  __image_height_2     = __image_height / 2;
  __ball_circumference = ball_circumference;
  __camera_height      = camera_height;
  __camera_offset_x    = camera_offset_x;
  __camera_offset_y    = camera_offset_y;
  __camera_base_pan    = camera_base_pan;
  __camera_base_tilt   = camera_base_tilt;
  __horizontal_angle   = deg2rad( horizontal_angle );
  __vertical_angle     = deg2rad( vertical_angle   );

  __cirt_center.x      = 0.0f;
  __cirt_center.y      = 0.0f;
  __pan                = 0.0f;
  __tilt               = 0.0f;

  __pan_rad_per_pixel  = __horizontal_angle / (float)__image_width;
  __tilt_rad_per_pixel = __vertical_angle   / (float)__image_height;
  __ball_radius        = __ball_circumference / ( 2.0 * M_PI );

  __ball_x = __ball_y = __bearing = __slope = __distance = 0.f;
}


float
BallTrigoRelativePos::get_distance() const
{
  return __distance;
}


float
BallTrigoRelativePos::get_bearing() const
{
  return __bearing;
}


float
BallTrigoRelativePos::get_slope() const
{
  return __slope;
}


float
BallTrigoRelativePos::get_y() const
{
  return __ball_y;
}


float
BallTrigoRelativePos::get_x() const
{
  return __ball_x;
}


void
BallTrigoRelativePos::set_center(float x, float y)
{
  __cirt_center.x = x;
  __cirt_center.y = y;
}


void
BallTrigoRelativePos::set_center(const center_in_roi_t& c)
{
  __cirt_center.x = c.x;
  __cirt_center.y = c.y;
}


void
BallTrigoRelativePos::set_radius(float r)
{
}


void
BallTrigoRelativePos::set_pan_tilt(float pan, float tilt)
{
  __pan = pan;
  __tilt = tilt;
}


void
BallTrigoRelativePos::get_pan_tilt(float *pan, float *tilt) const
{
  *pan  = __pan;
  *tilt = __tilt;
}


const char *
BallTrigoRelativePos::get_name() const
{
  return "BallTrigoRelativePos";
}


void
BallTrigoRelativePos::reset()
{
}

void
BallTrigoRelativePos::calc()
{
#ifdef OLD_COORD_SYS
  /* Bearing shall be clockwise positive. */
  __bearing =   (((__cirt_center.x - __image_width_2) * __pan_rad_per_pixel
		  + __pan + __camera_base_pan));
#else
  /* Bearing shall be counter-clockwise positive. */
  __bearing = - (((__cirt_center.x - __image_width_2) * __pan_rad_per_pixel
		  + __pan + __camera_base_pan));
#endif

  /* Slope shall be downward negative */
  __slope   = ((__image_height_2 - __cirt_center.y) * __tilt_rad_per_pixel
		 + __tilt + __camera_base_tilt);

  float alpha = M_PI_2 - __slope;

  float e = __camera_height - __ball_radius - __ball_radius * cos(alpha);
  __distance = - (e * tan(alpha) + __ball_radius * sin(alpha));

  __ball_x = cos( __bearing ) * __distance + __camera_offset_x;
  __ball_y = sin( __bearing ) * __distance + __camera_offset_y;
}


bool
BallTrigoRelativePos::is_pos_valid() const
{
  return __distance > 0; //Distance is < 0 if the ROI is above the horizon
}

} // end namespace firevision
