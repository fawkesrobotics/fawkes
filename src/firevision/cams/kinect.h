
/***************************************************************************
 *  kinect.h - Microsoft Kinect 3D Camera using the freenect driver
 *
 *  Created: Fri Nov 26 10:46:09 2010
 *  Copyright  2010  Daniel Beck
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

#ifndef __FIREVISION_CAMS_KINECT_H_
#define __FIREVISION_CAMS_KINECT_H_

#include "camera.h"

#include <libfreenect.hpp>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class FvFreenectDevice : public Freenect::FreenectDevice
{
 public:
  FvFreenectDevice( freenect_context* ctx, int index );
  ~FvFreenectDevice();

  void RGBCallback( freenect_pixel* rgb, uint32_t timestamp );
  void DepthCallback( void* depth, uint32_t timestamp );

  unsigned char* rgb_buffer();
  uint16_t*      depth_buffer();

 private:
  unsigned char* m_rgb_buffer;
  uint16_t*      m_depth_buffer;

  uint32_t m_rgb_timestamp;
  uint32_t m_depth_timestamp;
};

class KinectCamera : public Camera
{
 public:
  KinectCamera( const CameraArgumentParser* cap = NULL );
  ~KinectCamera();

  virtual void open();
  virtual void start();
  virtual void stop();
  virtual void close();
  virtual void capture();
  virtual void flush();

  virtual bool ready();

  virtual void print_info();

  virtual unsigned char* buffer();
  virtual unsigned int   buffer_size();
  virtual void           dispose_buffer();

  virtual unsigned int pixel_width();
  virtual unsigned int pixel_height();
  virtual colorspace_t colorspace();

  virtual void set_image_number( unsigned int n );

 public:
  static const unsigned int RGB_IMAGE;
  static const unsigned int FALSE_COLOR_DEPTH_IMAGE;

 private:
  Freenect::Freenect< FvFreenectDevice >* m_freenect_ctx;
  FvFreenectDevice*                       m_freenect_dev;

  bool m_opened;
  bool m_started;

  unsigned int m_image_num;

  unsigned char* m_buffer;
  unsigned char* m_false_color_depth_buffer;

  uint16_t m_gamma[2048];
};

} // end namespace firevision

#endif /* __FIREVISION_CAMS_KINECT_H_ */
