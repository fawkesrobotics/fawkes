
/***************************************************************************
 *  omni_ball_relative.h - A simple implementation of a relative omni
 *                         relative position model using a MirrorModel
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

#ifndef __FIREVISION_MODELS_RELATIVE_POSITION_OMNI_RELATIVE_H_
#define __FIREVISION_MODELS_RELATIVE_POSITION_OMNI_RELATIVE_H_

#include <fvmodels/relative_position/relativepositionmodel.h>
#include <fvmodels/mirror/mirrormodel.h>

// include <utils/kalman_filter/ckalman_filter_2dim.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OmniRelative : public RelativePositionModel
{
 public:
  // constructor
  OmniRelative(MirrorModel *mirror_model);

  virtual const char *  get_name() const;
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
  virtual float         get_radius() const;

  virtual void          calc();
  virtual void          calc_unfiltered();
  virtual void          reset();

  virtual bool          is_pos_valid() const;

private:
  float                 DEFAULT_X_VARIANCE;
  float                 DEFAULT_Y_VARIANCE;

  MirrorModel          *mirror_model;

  unsigned int          image_x;
  unsigned int          image_y;

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

  //kalmanFilter2Dim     *kalman_filter;

  //void                  applyKalmanFilter();
};

} // end namespace firevision

#endif
