
/***************************************************************************
 *  v4l1.h - This header defines a Video4Linux cam
 *
 *  Generated: Fri Mar 11 17:46:31 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <cams/camera.h>

#include <sys/types.h>
#include <linux/videodev.h>

class CameraArgumentParser;

class V4L1Camera : public Camera
{
  friend class V4LCamera;

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
  V4L1Camera(const char *device_name, int dev,
             struct video_capability capabilities);

 private:
  virtual void post_open();

 private:

  static const int MMAP = 1;
  static const int READ = 2;


  bool opened;
  bool started;
  char *device_name;

  int capture_method;

  int dev;
  unsigned char *frame_buffer;

  /* V4L1 stuff */
  struct video_capability  capabilities;          // Device Capabilities: Can overlay, Number of channels, etc
  struct video_buffer      vbuffer;               // information about buffer
  struct video_window      window;                // Window Information: Size, Depth, etc
  struct video_channel    *channel;               // Channels information: Channel[0] holds information for channel 0 and so on...
  struct video_picture     picture;               // Picture information: Palette, contrast, hue, etc
  struct video_tuner      *tuner;                 // Tuner Information: if the card has tuners...
  struct video_audio       audio;                 // If the card has audio
  struct video_mbuf        captured_frame_buffer; // Information for the frame to be captured: norm, palette, etc
  struct video_mmap       *buf_v4l;               // mmap() buffer VIDIOCMCAPTURE


};

#endif
