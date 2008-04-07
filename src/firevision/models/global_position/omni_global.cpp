
/***************************************************************************
 *  omni_relative.cpp - Implementation of the relative ball model
 *                      for the omni cam
 *
 *  Generated: Thu Mar 23 22:00:15 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <models/global_position/omni_global.h>
#include <models/mirror/mirrormodel.h>

/** @class OmniGlobal <models/global_position/omni_global.h>
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
OmniGlobal::setPositionInImage(unsigned int x, unsigned int y)
{
  image_x = x;
  image_y = y;
}


void
OmniGlobal::setRobotPosition(float x, float y, float ori)
{
  pose_x   = x;
  pose_y   = y;
  pose_ori = ori;
}


float
OmniGlobal::getY(void) const
{
  return ball_y;
}


float
OmniGlobal::getX(void) const
{
  return ball_x;
}


void
OmniGlobal::calc()
{
  if ( mirror_model->isValidPoint( image_x, image_y ) ) {

    f_point_t glob_pos = mirror_model->getWorldPointGlobal( image_x, image_y,
							    pose_x, pose_y, pose_ori);

    ball_x = glob_pos.x;
    ball_y = glob_pos.y;
  }
}


bool
OmniGlobal::isPosValid() const
{
  return mirror_model->isValidPoint( image_x, image_y );
}
