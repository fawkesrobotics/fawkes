
/***************************************************************************
 *  camera_tracker.h - This header defines a camera tracker
 *
 *  Generated: Thu Jul 14 22:29:25 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_CAMERA_TRACKER_H_
#define __FIREVISION_FVUTILS_CAMERA_TRACKER_H_

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RelativePositionModel;

class CameraTracker {

 public:
  CameraTracker(RelativePositionModel *relative_position_model,
		float camera_height,
                float camera_ori_deg );

  ~CameraTracker();

  void  calc();

  float get_new_pan();
  float get_new_tilt();

  void  set_mode(unsigned int mode);
  void  set_relative_position_model(RelativePositionModel *rpm);
  void  set_robot_position(float x, float y, float ori);
  void  set_world_point(float x, float y);

  static const unsigned int MODE_MODEL;
  static const unsigned int MODE_WORLD;

 private:

  RelativePositionModel *rpm;

  float  camera_height;
  float  camera_orientation;

  float  new_pan;
  float  new_tilt;

  float  robot_x;
  float  robot_y;
  float  robot_ori;

  float  world_x;
  float  world_y;

  unsigned int mode;
};

} // end namespace firevision

#endif
