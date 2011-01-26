
/***************************************************************************
 *  v4l.h - General Video4Linux access
 *
 *  Generated: Sat Jul  5 16:16:16 2008
 *  Copyright  2008 Tobias Kellner
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

#ifndef __FIREVISION_CAMS_V4L_H_
#define __FIREVISION_CAMS_V4L_H_

#include <fvcams/camera.h>
//#include <sys/types.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class V4LCamera : public Camera
{
 public:
  V4LCamera(const char *device_name = "/dev/video0");
  V4LCamera(const CameraArgumentParser *cap);
  virtual ~V4LCamera();

  virtual void open();
  virtual void start();
  virtual void stop();
  virtual void close();
  virtual void flush();
  virtual void capture();
  virtual void print_info();
  virtual bool ready();

  virtual unsigned char* buffer();
  virtual unsigned int   buffer_size();
  virtual void           dispose_buffer();

  virtual unsigned int    pixel_width();
  virtual unsigned int    pixel_height();
  virtual colorspace_t    colorspace();

  virtual void           set_image_number(unsigned int n);

 private:
  Camera *_v4l_cam;
  char *_device_name;
};

} // end namespace firevision

#endif //__FIREVISION_CAMS_V4L_H_
