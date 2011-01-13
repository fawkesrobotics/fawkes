
/***************************************************************************
 *  image.h - Abstract class defining a camera image controller
 *
 *  Created: Wed Apr 22 11:32:56 CEST 2009
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

#ifndef __FIREVISION_CAMS_CONTROL_IMAGE_H_
#define __FIREVISION_CAMS_CONTROL_IMAGE_H_

#include <fvcams/control/control.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraControlImage : virtual public CameraControl
{
 public:
  virtual ~CameraControlImage();

  virtual const char * format();
  virtual void         set_format(const char *format);
  virtual unsigned int width()                                         = 0;
  virtual unsigned int height()                                        = 0;
  virtual void         size(unsigned int &width, unsigned int &height);
  virtual void         set_size(unsigned int width,
                                unsigned int height)                   = 0;
  virtual bool         horiz_mirror();
  virtual bool         vert_mirror();
  virtual void         mirror(bool &horiz, bool &vert);
  virtual void         set_horiz_mirror(bool enabled);
  virtual void         set_vert_mirror(bool enabled);
  virtual void         set_mirror(bool horiz, bool vert);

  virtual unsigned int fps();
  virtual void         set_fps(unsigned int fps);

  virtual unsigned int lens_x_corr();
  virtual unsigned int lens_y_corr();
  virtual void         lens_corr(unsigned int &x_corr, unsigned int &y_corr);
  virtual void         set_lens_x_corr(unsigned int x_corr);
  virtual void         set_lens_y_corr(unsigned int y_corr);
  virtual void         set_lens_corr(unsigned int x_corr, unsigned int y_corr);
};

} // end namespace firevision

#endif // __FIREVISION_CAMS_CONTROL_IMAGE_H_
