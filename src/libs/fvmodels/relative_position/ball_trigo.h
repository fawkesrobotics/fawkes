
/****************************************************************************
 *  ball_trigo.h - Ball relpos for pan/tilt camera using basic trigonometry
 *
 *  Created: Mon Mar 23 09:39:26 2009
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

#ifndef __FIREVISION_MODELS_RELATIVE_POSITION_BALL_TRIGO_H_
#define __FIREVISION_MODELS_RELATIVE_POSITION_BALL_TRIGO_H_

#include <fvmodels/relative_position/relativepositionmodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BallTrigoRelativePos : public RelativePositionModel
{
 public:
  BallTrigoRelativePos(unsigned int image_width,
		       unsigned int image_height,
		       float camera_height,
		       float camera_offset_x,
		       float camera_offset_y,
		       float camera_base_pan,
		       float camera_base_tilt,
		       float horizontal_angle,
		       float vertical_angle,
		       float ball_circumference);

  virtual const char *	get_name() const;
  virtual void		set_radius(float r);
  virtual void		set_center(float x, float y);
  virtual void		set_center(const center_in_roi_t& c);

  virtual void		set_pan_tilt(float pan = 0.0f, float tilt = 0.0f);
  virtual void          get_pan_tilt(float *pan, float *tilt) const;

  virtual float		get_distance() const;
  virtual float		get_x() const;
  virtual float		get_y() const;
  virtual float		get_bearing() const;
  virtual float		get_slope() const;

  virtual void          calc();
  virtual void          calc_unfiltered() { calc(); }
  virtual void          reset();

  virtual bool          is_pos_valid() const;

private:
  center_in_roi_t       __cirt_center;
  float			__pan;
  float			__tilt;

  float                 __horizontal_angle;
  float                 __vertical_angle;
  float	                __pan_rad_per_pixel;
  float	                __tilt_rad_per_pixel;

  unsigned int          __image_width;
  unsigned int          __image_width_2;  // image_width / 2
  unsigned int          __image_height;
  unsigned int          __image_height_2; // image_height / 2

  float                 __camera_height;
  float                 __camera_offset_x;
  float                 __camera_offset_y;
  float                 __camera_base_pan;
  float                 __camera_base_tilt;

  float                 __ball_circumference;
  float                 __ball_radius;
  float                 __ball_x;
  float                 __ball_y;
  float                 __bearing;
  float                 __slope;
  float                 __distance;
};

} // end namespace firevision

#endif
