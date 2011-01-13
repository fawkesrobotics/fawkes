/***************************************************************************
 *  back_projection.h - Projective camera back projection model
 *
 *  Created:  Mon Apr 20 21:59:00 2009
 *  Copyright 2009 Christof Rath <christof.rath@gmail.com>
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

#ifndef __FIREVISION_MODELS_RELATIVE_POSITION_BACK_PROJECTION_H_
#define __FIREVISION_MODELS_RELATIVE_POSITION_BACK_PROJECTION_H_

#include <fvmodels/relative_position/relativepositionmodel.h>

#include <fvmodels/camera/projective_cam.h>
#include <utils/math/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class BackProjectionPositionModel : public RelativePositionModel
{
 public:
  BackProjectionPositionModel(ProjectiveCam &projective_cam,
                              float ball_circumference = 0.f);

  virtual const char *  get_name() const { return "BackProjectionPositionModel"; }
  virtual void          set_radius(float r);
  virtual void          set_center(float x, float y);
  virtual void          set_center(const center_in_roi_t& c) { set_center(c.x, c.y); }

  virtual void          set_pan_tilt(float pan = 0.0f, float tilt = 0.0f) { set_cam_rotation(pan, tilt, 0.f); }
  virtual void          get_pan_tilt(float *pan, float *tilt) const;

  virtual void          set_cam_rotation(float pan = 0.f, float tilt = 0.f, float roll = 0.f);
  virtual void          get_cam_rotation(float &pan, float &tilt, float &roll) const;

  virtual void          set_cam_translation(float height, float rel_x = 0.f, float rel_y = 0.f);
  virtual void          get_cam_translation(float &height, float &rel_x, float &rel_y) const;

  virtual float         get_distance() const { return __distance; }
  virtual float         get_x() const        { return __world_x; }
  virtual float         get_y() const        { return __world_y; }
  virtual float         get_bearing() const  { return __bearing; }
  virtual float         get_slope() const    { return __slope; }

  virtual void          calc();
  virtual void          calc_unfiltered()    { calc(); }
  virtual void          reset();

  virtual bool          is_pos_valid() const { return __pos_valid; }

 private:
  ProjectiveCam&  __projective_cam;

  center_in_roi_t       __cirt_center;

  fawkes::point_6D_t    __cam_position;

  float                 __ball_circumference;
  float                 __ball_radius;
  float                 __world_x;
  float                 __world_y;
  float                 __bearing;
  float                 __slope;
  float                 __distance;
  bool                  __pos_valid;
};

} // end namespace firevision

#endif /* BACK_PROJECTION_H_ */
