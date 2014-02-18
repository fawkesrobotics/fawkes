
/***************************************************************************
 *  box_relative.h - A simple implementation of a relative position model
 *                   for boxes
 *
 *  Created: Thu Jun 08 19:21:35 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_MODELS_RELPOS_BOX_H_
#define __FIREVISION_MODELS_RELPOS_BOX_H_

#include <fvmodels/relative_position/relativepositionmodel.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BoxRelative : public RelativePositionModel
{
 public:
  BoxRelative(unsigned int image_width, unsigned int image_height,
	      float camera_height,
	      float camera_offset_x, float camera_offset_y,
	      float camera_ori,
	      float horizontal_angle, float vertical_angle
	      );

  virtual const char *	get_name() const;
  virtual void		set_center(float x, float y);
  virtual void		set_center(const center_in_roi_t& c);
  virtual void          set_radius(float r);

  virtual void		set_pan_tilt(float pan = 0.0f, float tilt = 0.0f);
  virtual void          get_pan_tilt(float *pan, float *tilt) const;

  virtual void          set_horizontal_angle(float angle_deg);
  virtual void          set_vertical_angle(float angle_deg);

  virtual float		get_distance() const;

  virtual float		get_x() const;
  virtual float		get_y() const;

  virtual float		get_bearing() const;
  virtual float		get_slope() const;

  virtual void          calc();
  virtual void          calc_unfiltered();
  virtual void          reset();

  virtual bool          is_pos_valid() const;

private:
  float                 DEFAULT_X_VARIANCE;
  float                 DEFAULT_Y_VARIANCE;

  float	                pan_rad_per_pixel;
  float	                tilt_rad_per_pixel;

  center_in_roi_t       center;
  float			pan;
  float			tilt;

  float                 horizontal_angle;
  float                 vertical_angle;

  unsigned int          image_width;
  unsigned int          image_height;

  float                 camera_height;
  float                 camera_offset_x;
  float                 camera_offset_y;
  float                 camera_orientation;

  bool                  last_available;
  float                 box_x;
  float                 box_y;
  float                 bearing;
  float                 slope;
  float                 distance_box_motor;
  float                 distance_box_cam;

  /*
  float                 var_proc_x;
  float                 var_proc_y;
  float                 var_meas_x;
  float                 var_meas_y;
  kalmanFilter2Dim     *kalman_filter;

  void                  applyKalmanFilter();
  */
};

} // end namespace firevision

#endif // __FIREVISION_MODELS_RELPOS_BOX_H_

