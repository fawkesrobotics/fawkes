
/***************************************************************************
 *  leutron.h - This header defines a Leutron cam
 *
 *  Generated: Thu Mar 24 22:33:33 2005
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

#ifndef __FIREVISION_CAMS_LEUTRON_H_
#define __FIREVISION_CAMS_LEUTRON_H_

#include <fvcams/camera.h>

class LvCameraNode;
class LvGrabberNode;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class LeutronCamera : public Camera
{

 public:

  LeutronCamera();
  virtual ~LeutronCamera();

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

  virtual unsigned int   pixel_width();
  virtual unsigned int   pixel_height();
  virtual colorspace_t   colorspace();

  virtual void           set_image_number(unsigned int n);

 private:
  bool opened;
  bool started;
  bool autodetect;

  const char *camera_name;

  // should be HGRABBER, set to unsigned short to get rid of all the warnings
  // by the broken-like-shit dsylib.
  unsigned short int camera_handle;
  LvCameraNode   *camera;
  LvGrabberNode  *grabber;

  unsigned int    width;
  unsigned int    height;
  unsigned int    scaled_width;
  unsigned int    scaled_height;

  colorspace_t cspace;

  unsigned char  *scaled_buffer;

};

} // end namespace firevision

#endif
