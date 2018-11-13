
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
  image_width_        = image_width;
  image_width_2_      = image_width_ / 2;
  image_height_       = image_height;
  image_height_2_     = image_height_ / 2;
  ball_circumference_ = ball_circumference;
  camera_height_      = camera_height;
  camera_offset_x_    = camera_offset_x;
  camera_offset_y_    = camera_offset_y;
  camera_base_pan_    = camera_base_pan;
  camera_base_tilt_   = camera_base_tilt;
  horizontal_angle_   = deg2rad( horizontal_angle );
  vertical_angle_     = deg2rad( vertical_angle   );

  cirt_center_.x      = 0.0f;
  cirt_center_.y      = 0.0f;
  pan_                = 0.0f;
  tilt_               = 0.0f;

  pan_rad_per_pixel_  = horizontal_angle_ / (float)image_width_;
  tilt_rad_per_pixel_ = vertical_angle_   / (float)image_height_;
  ball_radius_        = ball_circumference_ / ( 2.0 * M_PI );

  ball_x_ = ball_y_ = bearing_ = slope_ = distance_ = 0.f;
}


float
BallTrigoRelativePos::get_distance() const
{
  return distance_;
}


float
BallTrigoRelativePos::get_bearing() const
{
  return bearing_;
}


float
BallTrigoRelativePos::get_slope() const
{
  return slope_;
}


float
BallTrigoRelativePos::get_y() const
{
  return ball_y_;
}


float
BallTrigoRelativePos::get_x() const
{
  return ball_x_;
}


void
BallTrigoRelativePos::set_center(float x, float y)
{
  cirt_center_.x = x;
  cirt_center_.y = y;
}


void
BallTrigoRelativePos::set_center(const center_in_roi_t& c)
{
  cirt_center_.x = c.x;
  cirt_center_.y = c.y;
}


void
BallTrigoRelativePos::set_radius(float r)
{
}


void
BallTrigoRelativePos::set_pan_tilt(float pan, float tilt)
{
  pan_ = pan;
  tilt_ = tilt;
}


void
BallTrigoRelativePos::get_pan_tilt(float *pan, float *tilt) const
{
  *pan  = pan_;
  *tilt = tilt_;
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
  bearing_ =   (((cirt_center_.x - image_width_2_) * pan_rad_per_pixel_
		  + pan_ + camera_base_pan_));
#else
  /* Bearing shall be counter-clockwise positive. */
  bearing_ = - (((cirt_center_.x - image_width_2_) * pan_rad_per_pixel_
		  + pan_ + camera_base_pan_));
#endif

  /* Slope shall be downward negative */
  slope_   = ((image_height_2_ - cirt_center_.y) * tilt_rad_per_pixel_
		 + tilt_ + camera_base_tilt_);

  float alpha = M_PI_2 - slope_;

  float e = camera_height_ - ball_radius_ - ball_radius_ * cos(alpha);
  distance_ = - (e * tan(alpha) + ball_radius_ * sin(alpha));

  ball_x_ = cos( bearing_ ) * distance_ + camera_offset_x_;
  ball_y_ = sin( bearing_ ) * distance_ + camera_offset_y_;
}


bool
BallTrigoRelativePos::is_pos_valid() const
{
  return distance_ > 0; //Distance is < 0 if the ROI is above the horizon
}

} // end namespace firevision
