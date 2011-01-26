
/***************************************************************************
 *  pantilt.h - Abstract class defining a pan/tilt camera controller
 *
 *  Created: Tue Apr 21 22:39:20 2009
 *  Copyright  2009      Tobias Kellner
 *             2005-2009 Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_CONTROL_PANTILT_H_
#define __FIREVISION_CAMS_CONTROL_PANTILT_H_

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraControlPanTilt : virtual public CameraControl
{
 public:
  virtual ~CameraControlPanTilt();

  virtual void process_pantilt()                                        = 0;

  virtual bool supports_pan()                                           = 0;
  virtual bool supports_tilt()                                          = 0;
  virtual void set_pan(int pan)                                         = 0;
  virtual void set_tilt(int tilt)                                       = 0;
  virtual void set_pan_tilt(int pan, int tilt)                          = 0;
  virtual void set_pan_tilt_rad(float pan, float tilt)                  = 0;
  virtual int  pan()                                                    = 0;
  virtual int  tilt()                                                   = 0;
  virtual void start_get_pan_tilt()                                     = 0;
  virtual void pan_tilt(int &pan, int &tilt)                            = 0;
  virtual void pan_tilt_rad(float &pan, float &tilt)                    = 0;
  virtual int  min_pan()                                                = 0;
  virtual int  max_pan()                                                = 0;
  virtual int  min_tilt()                                               = 0;
  virtual int  max_tilt()                                               = 0;
  virtual void reset_pan_tilt()                                         = 0;
  virtual void set_pan_tilt_limit(int pan_left, int pan_right,
                                  int tilt_up, int tilt_down)           = 0;
  virtual void reset_pan_tilt_limit()                                   = 0;
};

} // end namespace firevision

#endif // __FIREVISION_CAMS_CONTROL_PANTILT_H_
