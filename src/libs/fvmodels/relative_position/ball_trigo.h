
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

#ifndef _FIREVISION_MODELS_RELATIVE_POSITION_BALL_TRIGO_H_
#define _FIREVISION_MODELS_RELATIVE_POSITION_BALL_TRIGO_H_

#include <fvmodels/relative_position/relativepositionmodel.h>

namespace firevision {

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
  center_in_roi_t       cirt_center_;
  float			pan_;
  float			tilt_;

  float                 horizontal_angle_;
  float                 vertical_angle_;
  float	                pan_rad_per_pixel_;
  float	                tilt_rad_per_pixel_;

  unsigned int          image_width_;
  unsigned int          image_width_2_;  // image_width / 2
  unsigned int          image_height_;
  unsigned int          image_height_2_; // image_height / 2

  float                 camera_height_;
  float                 camera_offset_x_;
  float                 camera_offset_y_;
  float                 camera_base_pan_;
  float                 camera_base_tilt_;

  float                 ball_circumference_;
  float                 ball_radius_;
  float                 ball_x_;
  float                 ball_y_;
  float                 bearing_;
  float                 slope_;
  float                 distance_;
};

} // end namespace firevision

#endif
