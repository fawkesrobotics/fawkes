/***************************************************************************
 *  back_projection.cpp - Projective camera back projection model
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

#include <fvmodels/relative_position/back_projection.h>

#include <cmath>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BackProjectionPositionModel <fvmodels/relative_position/back_projection.h>
 * This model uses a ProjectiveCam to back project points in the image to
 * the world by the ground plane assumption.
 *
 * @author Christof Rath
 */

/** Constructor
 * @param projective_cam the projective camera object
 * @param ball_circumference the circumference in case it's a ball
 */
BackProjectionPositionModel::BackProjectionPositionModel(ProjectiveCam &projective_cam,
                                                         float ball_circumference):
  __projective_cam(projective_cam),
  __ball_circumference(ball_circumference)
{
  __ball_radius   = __ball_circumference / ( 2.0 * M_PI );

  __world_x        = __world_y = __bearing = __slope = __distance = 0.f;
  __cirt_center.x = __cirt_center.y = 0.0f;
  __pos_valid     = false;
}


void
BackProjectionPositionModel::set_radius(float r)
{
  __ball_radius = r;
}


void
BackProjectionPositionModel::set_center(float x, float y)
{
  __cirt_center.x = x;
  __cirt_center.y = y;
}


void
BackProjectionPositionModel::set_cam_rotation(float pan, float tilt, float roll)
{
  __cam_position.roll  = roll;
  __cam_position.pitch = -tilt; //tilt down-wards negative, cam pitch down-wards positive
  __cam_position.yaw   = pan;
}

void BackProjectionPositionModel::get_pan_tilt(float *pan, float *tilt) const
{
  float roll;
  get_cam_rotation(*pan, *tilt, roll);
}

void BackProjectionPositionModel::get_cam_rotation(float &pan, float &tilt, float &roll) const
{
  pan  = __cam_position.yaw;
  tilt = -__cam_position.pitch; //tilt down-wards negative, cam pitch down-wards positive
  roll = __cam_position.roll;
}

void BackProjectionPositionModel::set_cam_translation(float height, float rel_x, float rel_y)
{
  __cam_position.x = rel_x;
  __cam_position.y = rel_y;
  __cam_position.z = height;
}

void BackProjectionPositionModel::get_cam_translation(float & height, float & rel_x, float & rel_y) const
{
  height = __cam_position.z;
  rel_x  = __cam_position.x;
  rel_y  = __cam_position.y;
}



void BackProjectionPositionModel::calc()
{
  __projective_cam.set_location(
      __cam_position.roll,
      __cam_position.pitch,
      __cam_position.yaw,
      __cam_position.z,
      __cam_position.x,
      __cam_position.y);

  try {
    fawkes::cart_coord_2d_t wld_pt = __projective_cam.get_GPA_world_coord(__cirt_center);

    __world_x   = wld_pt.x;
    __world_y   = wld_pt.y;
    __pos_valid = true;
  }
  catch (AboveHorizonException &e) {
    __world_x   = 0.f;
    __world_y   = 0.f;
    __pos_valid = false;
  }

#ifdef OLD_COORD_SYS
  /* Bearing shall be clockwise positive. */
  __bearing = -atan2f(_ball_y, __world_x);
#else
  __bearing = atan2f(__world_y, __world_x);
#endif

  __distance = sqrt(__world_x * __world_x + __world_y * __world_y);
  __slope    = -atan2f(__cam_position.z, __distance); //slope is down-wards negative
}

void BackProjectionPositionModel::reset()
{
  __pos_valid = false;
}

} // end namespace firevision
