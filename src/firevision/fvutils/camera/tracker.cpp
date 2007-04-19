
/***************************************************************************
 *  camera_tracker.cpp - Implementation of the camera tracker
 *
 *  Generated: Thu Jul 14 22:18:14 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: camera_tracker.cpp,v 2.3 2006/08/21 13:12:45 tim Exp $
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <fvutils/camera/tracker.h>
#include <utils/system/console_colors.h>
#include <utils/math/angle.h>

#include <models/relative_position/relativepositionmodel.h>

#include <iostream>
#include <cmath>

using namespace std;

const unsigned int CameraTracker::MODE_MODEL = 0;
const unsigned int CameraTracker::MODE_WORLD = 1;


CameraTracker::CameraTracker(RelativePositionModel *relative_position_model,
			     float camera_height,
                             float camera_ori_deg
                             )
{
  rpm = relative_position_model;
  mode = MODE_MODEL;
  this->camera_height = camera_height;
  this->camera_orientation = deg2rad( camera_ori_deg );
}


CameraTracker::~CameraTracker()
{
}


void
CameraTracker::calc()
{
  if (mode == MODE_MODEL) {
    new_pan  = rpm->getBearing() - camera_orientation;
    new_tilt = rpm->getSlope();
  } else if (mode == MODE_WORLD) {

    float w_r_x = world_x - robot_x;
    float w_r_y = world_y - robot_y;

    float distance = sqrt( w_r_x * w_r_x + w_r_y * w_r_y );

    //cout << msg_prefix << "  world_x=" << world_x << "  world_y=" << world_y
    //     << "  robot_x=" << robot_x << "  robot_y=" << robot_y << endl;
    //cout << msg_prefix << "  w_r_x=" << w_r_x << "  w_r_y=" << w_r_y
    //     << "  dist=" << distance << endl;

    /* atan2f magic
     * tan alpha = opposite leg / adjacent leg
     * => alpha = atan( opposite leg / adjacent leg )
     *
     * atan2f now takes y = length(opposite leg) and x = length(adjacent leg)
     * and calculates the angle alpha. It's exactle the same as the above
     *
     * So here we want to calculate the bearing to the world point
     * So we have a right triangle, where w_r_y is the length of the adjacent
     * leg and w_r_x is the length of the opposite leg. So to calculate the
     * bearing / new pan we call atan2f(w_r_x, w_r_y).
     * For the new tilt we need the distance. This gives us a right triangle
     * with distance being the opposite leg and the height of the camera on
     * the robot being the adjacent leg. So slope / new tilt is
     * atan2f(distance, camera_height).
     */

    // Calculate bearing to point
    new_pan  = atan2f( w_r_y, w_r_x );
    new_pan = normalize_mirror_rad( new_pan - robot_ori - camera_orientation);
    new_tilt = atan2f( camera_height, distance );
  }
}


float
CameraTracker::getNewPan()
{
  return new_pan;
}


float
CameraTracker::getNewTilt()
{
  return new_tilt;
}


void
CameraTracker::setPanTilt(float pan, float tilt)
{
  this->pan  = pan;
  this->tilt = tilt;
}


void
CameraTracker::setMode(unsigned int mode)
{
  if ( (mode == MODE_WORLD) || (mode == MODE_MODEL)) {
    this->mode = mode;
  } else {
    cout << "CameraTracker: Invalid mode, not setting mode" << endl;
  }
}


void
CameraTracker::setRelativePositionModel(RelativePositionModel *rpm)
{
  this->rpm = rpm;
}


void
CameraTracker::setRobotPosition(float x, float y, float ori)
{
  robot_x   = x;
  robot_y   = y;
  robot_ori = ori;
}


void
CameraTracker::setWorldPoint(float x, float y)
{
  world_x = x;
  world_y = y;
}

