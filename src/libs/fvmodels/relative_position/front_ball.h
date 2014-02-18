
/***************************************************************************
 *  front_ball.h - A simple implementation of the relative position model
 *                 for the ball in the front vision
 *
 *  Created: Fri Jun 03 22:56:22 2005
 *  Copyright  2005  Hu Yuxiao      <Yuxiao.Hu@rwth-aachen.de>
 *                   Tim Niemueller [www.niemueller.de]
 *                   Martin Heracles <Martin.Heracles@rwth-aachen.de>
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

#ifndef __FIREVISION_MODELS_RELATIVE_FRONT_BALL_H_
#define __FIREVISION_MODELS_RELATIVE_FRONT_BALL_H_

#include <fvmodels/relative_position/relativepositionmodel.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FrontBallRelativePos : public RelativePositionModel
{
 public:
  FrontBallRelativePos(unsigned int image_width, unsigned int image_height,
		       float camera_height,
		       float camera_offset_x, float camera_offset_y,
		       float camera_ori,
		       float horizontal_angle, float vertical_angle,
		       float ball_circumference
		       );

  virtual const char *	get_name() const;
  virtual void		set_radius(float r);
  virtual void		set_center(float x, float y);
  virtual void		set_center(const center_in_roi_t& c);

  virtual void		set_pan_tilt(float pan = 0.0f, float tilt = 0.0f);
  virtual void          get_pan_tilt(float *pan, float *tilt) const;

  virtual void          set_horizontal_angle(float angle_deg);
  virtual void          set_vertical_angle(float angle_deg);

  virtual float		get_distance() const;
  virtual float		get_x() const;
  virtual float		get_y() const;
  virtual float		get_bearing() const;
  virtual float		get_slope() const;
  virtual float         get_radius() const;

  virtual void          calc();
  virtual void          calc_unfiltered();
  virtual void          reset();

  virtual bool          is_pos_valid() const;

private:
  float                 DEFAULT_X_VARIANCE;
  float                 DEFAULT_Y_VARIANCE;

  float	                m_fPanRadPerPixel;
  float	                m_fTiltRadPerPixel;
  float	                m_fBallRadius;        // in meter

  float			m_fRadius;
  center_in_roi_t       m_cirtCenter;
  float			m_fPan;
  float			m_fTilt;

  float                 horizontal_angle;
  float                 vertical_angle;

  unsigned int          image_width;
  unsigned int          image_height;

  float                 camera_height;
  float                 camera_offset_x;
  float                 camera_offset_y;
  float                 camera_orientation;

  float                 ball_circumference;

  bool                  last_available;
  float                 ball_x;
  float                 ball_y;
  float                 bearing;
  float                 slope;
  float                 distance_ball_motor;
  float                 distance_ball_cam;

  float                 avg_x;
  float                 avg_y;
  float                 avg_x_sum;
  float                 avg_y_sum;
  unsigned int          avg_x_num;
  unsigned int          avg_y_num;

  float                 var_proc_x;
  float                 var_proc_y;
  float                 var_meas_x;
  float                 var_meas_y;
  // kalmanFilter2Dim     *kalman_filter;

  // void                  applyKalmanFilter();
};

} // end namespace firevision

#endif // __FIREVISION_MODELS_RELPOS_BALL_H_

