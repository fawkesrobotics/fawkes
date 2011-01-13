
/***************************************************************************
 *  camera_tracker.cpp - Implementation of the camera tracker
 *
 *  Created: Thu Jul 14 22:18:14 2005
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>

#include <fvutils/camera/tracker.h>
#include <utils/system/console_colors.h>
#include <utils/math/angle.h>

#include <fvmodels/relative_position/relativepositionmodel.h>

#include <cmath>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CameraTracker <fvutils/camera/tracker.h>
 * Camera Tracker.
 * Utility class that allows for tracking and object or a world point
 * by using a camera pan/tilt unit. It is NOT meant to track an object
 * in a scene!
 *
 * The camera tracker will try to keep the desired object or point in the middle
 * of the image. Given a relative position model or a world point and robot pose
 * information and initial information the camera tracker returns pan/tilt angles
 * that are required to have the object in the center of the image. The using
 * application can then fulfill this desired angles if this lies within the
 * physical constraints of the pan/tilt unit.
 *
 * @author Tim Niemueller
 */

/** Model mode, track by a relative world model. */
const unsigned int CameraTracker::MODE_MODEL = 0;
/** World point mode, track a world point */
const unsigned int CameraTracker::MODE_WORLD = 1;


/** Constructor.
 * @param relative_position_model Relative position model to use if in model tracking
 * mode.
 * @param camera_height height above ground of the camera, objects are assumed to lie
 * on the ground plane.
 * @param camera_ori_deg The angle between the forward position and the actual position
 * of the camera on the robot in degrees, clock-wise positive.
 */
CameraTracker::CameraTracker(RelativePositionModel *relative_position_model,
			     float camera_height,
                             float camera_ori_deg
                             )
{
  rpm = relative_position_model;
  mode = MODE_MODEL;
  this->camera_height = camera_height;
  this->camera_orientation = fawkes::deg2rad( camera_ori_deg );
}


/** Destructor. */
CameraTracker::~CameraTracker()
{
}


/** Calculate values.
 * Based on the set data like robot position, world point and relative position model
 * this calculates the new desired values for pan and tilt.
 */
void
CameraTracker::calc()
{
  if (mode == MODE_MODEL) {
    new_pan  = rpm->get_bearing() - camera_orientation;
    new_tilt = rpm->get_slope();
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
    new_pan = fawkes::normalize_mirror_rad( new_pan - robot_ori - camera_orientation);
    new_tilt = atan2f( camera_height, distance );
  }
}


/** Get the new pan value.
 * @return new optimal pan value
 */
float
CameraTracker::get_new_pan()
{
  return new_pan;
}


/** Get the new tilt value.
 * @return new optimal tilt value
 */
float
CameraTracker::get_new_tilt()
{
  return new_tilt;
}


/** Set tracking mode.
 * @param mode new tracking mode
 * @exception Exception thrown, if mode is neither MODE_WORLD nor MODE_MODEL
 */
void
CameraTracker::set_mode(unsigned int mode)
{
  if ( (mode == MODE_WORLD) || (mode == MODE_MODEL)) {
    this->mode = mode;
  } else {
    throw fawkes::Exception("CameraTracker: Invalid mode, not setting mode");
  }
}


/** Set relative position model.
 * Switch the relative position model.
 * @param rpm new relative position model
 */
void
CameraTracker::set_relative_position_model(RelativePositionModel *rpm)
{
  this->rpm = rpm;
}


/** Set robot position.
 * Set the current robot position.
 * @param x new x coordinate in robot system
 * @param y new y coordinate in robot system
 * @param ori new orientation
 */
void
CameraTracker::set_robot_position(float x, float y, float ori)
{
  robot_x   = x;
  robot_y   = y;
  robot_ori = ori;
}


/** Set world point.
 * World point to track for the robot. The world point is given in a robot-relative
 * coordinate system on the ground plane. X-axis is pointing forward, Y-axis to
 * the right (right-handed coordinate system).
 * @param x x coordinate to track
 * @param y y coordinate to track
 */
void
CameraTracker::set_world_point(float x, float y)
{
  world_x = x;
  world_y = y;
}


} // end namespace firevision
