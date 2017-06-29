
/***************************************************************************
 *  v4l1.h - This header defines a Video4Linux cam
 *
 *  Generated: Fri Mar 11 17:46:31 2005
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

#ifndef __FIREVISION_CAMS_V4L1_H_
#define __FIREVISION_CAMS_V4L1_H_

#include <fvcams/camera.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;
class V4L1CameraData;
class V4LCamera;

class V4L1Camera : public Camera
{
 friend V4LCamera;

 public:
  V4L1Camera(const char *device_name = "/dev/video0");
  V4L1Camera(const CameraArgumentParser *cap);
  virtual ~V4L1Camera();

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

 protected:
  V4L1Camera(const char *device_name, int dev);

 private:
  virtual void post_open();

 private:

  static const int MMAP = 1;
  static const int READ = 2;

  V4L1CameraData *__data;

  bool opened;
  bool started;

  int capture_method;

  int dev;
  unsigned char *frame_buffer;


};

} // end namespace firevision

#endif
