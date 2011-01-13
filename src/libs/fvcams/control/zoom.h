
/***************************************************************************
 *  zoom.h - Abstract class defining a camera zoom controller
 *
 *  Created: Wed Apr 22 10:50:53 2009
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

#ifndef __FIREVISION_CAMS_CONTROL_ZOOM_H_
#define __FIREVISION_CAMS_CONTROL_ZOOM_H_

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraControlZoom : virtual public CameraControl
{
 public:
  virtual ~CameraControlZoom();

  virtual void         reset_zoom()                                     = 0;
  virtual void         set_zoom(unsigned int zoom)                      = 0;
  virtual unsigned int zoom()                                           = 0;
  virtual unsigned int zoom_max()                                       = 0;
  virtual unsigned int zoom_min()                                       = 0;
  virtual void         set_zoom_speed_tele(unsigned int speed);
  virtual void         set_zoom_speed_wide(unsigned int speed);
  virtual void         set_zoom_digital_enabled(bool enabled);
};

} // end namespace firevision

#endif // __FIREVISION_CAMS_CONTROL_ZOOM_H_
