
/***************************************************************************
 *  omni_relative.cpp - Implementation of the relative ball model
 *                      for the omni cam
 *
 *  Created: Thu Mar 23 22:00:15 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <fvmodels/global_position/omni_global.h>
#include <fvmodels/mirror/mirrormodel.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OmniGlobal <fvmodels/global_position/omni_global.h>
 * Omni vision global position model.
 */

/** Constructor.
 * @param mirror_model mirror model
 */
OmniGlobal::OmniGlobal(MirrorModel *mirror_model)
{
  this->mirror_model = mirror_model;

  ball_x = ball_y = 0.f;
}


void
OmniGlobal::set_position_in_image(unsigned int x, unsigned int y)
{
  image_x = x;
  image_y = y;
}


void
OmniGlobal::set_robot_position(float x, float y, float ori)
{
  pose_x   = x;
  pose_y   = y;
  pose_ori = ori;
}


float
OmniGlobal::get_y(void) const
{
  return ball_y;
}


float
OmniGlobal::get_x(void) const
{
  return ball_x;
}


void
OmniGlobal::calc()
{
  if ( mirror_model->isValidPoint( image_x, image_y ) ) {

    fawkes::cart_coord_2d_t glob_pos = mirror_model->getWorldPointGlobal( image_x,
									  image_y,
									  pose_x,
									  pose_y,
									  pose_ori);

    ball_x = glob_pos.x;
    ball_y = glob_pos.y;
  }
}


bool
OmniGlobal::is_pos_valid() const
{
  return mirror_model->isValidPoint( image_x, image_y );
}

} // end namespace firevision
