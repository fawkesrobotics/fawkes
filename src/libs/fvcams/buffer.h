
/***************************************************************************
 *  buffer.h - Camera model for a simple buffer
 *
 *  Created: Tue Mar 08 22:42:12 2016
 *  Copyright  2005-2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_BUFFER_H_
#define __FIREVISION_CAMS_BUFFER_H_

#include <fvcams/camera.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class BufferCamera : public Camera
{
 public:
  BufferCamera(colorspace_t cspace, unsigned int width, unsigned int height);
  ~BufferCamera();

  virtual void             open();
  virtual void             start();
  virtual void             stop();
  virtual void             close();
  virtual void             capture();
  virtual void             flush();

  virtual bool             ready();

  virtual void             print_info();

  virtual unsigned char *  buffer();
  virtual unsigned int     buffer_size();
  virtual void             dispose_buffer();

  virtual unsigned int     pixel_width();
  virtual unsigned int     pixel_height();
  virtual colorspace_t     colorspace();

  virtual void             set_image_number(unsigned int n);

 private:
  unsigned char* buffer_;
  unsigned int buffer_size_;
  unsigned int width_;
  unsigned int height_;
  colorspace_t cspace_;
};

} // end namespace firevision

#endif
