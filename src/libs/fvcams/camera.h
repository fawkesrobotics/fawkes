
/***************************************************************************
 *  camera.h - Abstract class defining a camera
 *
 *  Created: Tue Feb 22 10:36:39 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMERA_H_
#define __FIREVISION_CAMERA_H_

#include <fvutils/color/colorspaces.h>
#include <utils/time/time.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Camera
{

 public:
  virtual ~Camera();

  virtual void             open()                                  = 0;
  virtual void             start()                                 = 0;
  virtual void             stop()                                  = 0;
  virtual void             close()                                 = 0;
  virtual void             capture()                               = 0;
  virtual void             flush()                                 = 0;

  virtual bool             ready()                                 = 0;

  virtual void             print_info()                            = 0;

  virtual unsigned char *  buffer()                                = 0;
  virtual unsigned int     buffer_size()                           = 0;
  virtual void             dispose_buffer()                        = 0;

  virtual unsigned int     pixel_width()                           = 0;
  virtual unsigned int     pixel_height()                          = 0;
  virtual colorspace_t     colorspace()                            = 0;
  virtual fawkes::Time *   capture_time();

  // virtual unsigned int     number_of_images()                      = 0;
  virtual void             set_image_number(unsigned int n)        = 0;
};

} // end namespace firevision

#endif
