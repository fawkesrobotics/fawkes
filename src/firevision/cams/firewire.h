
/***************************************************************************
 *  firewire.h - This header defines a Firewire 1394 cam
 *
 *  Generated: Tue Feb 22 10:36:39 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __FIREVISION_FIREWIRE_H_
#define __FIREVISION_FIREWIRE_H_

#include "cams/camera.h"
#include "cams/cameracontrol.h"

namespace dc1394 {
#include <libraw1394/raw1394.h>
#include <dc1394/control.h>
}

class CameraArgumentParser;

class FirewireCamera : public Camera, public CameraControl
{

 public:

  FirewireCamera(dc1394::dc1394framerate_t framerate = dc1394::DC1394_FRAMERATE_30,
		 dc1394::dc1394video_mode_t mode     = dc1394::DC1394_VIDEO_MODE_640x480_YUV422,
		 dc1394::dc1394speed_t speed         = dc1394::DC1394_ISO_SPEED_400,
		 int num_buffers=8);
  FirewireCamera(CameraArgumentParser *cap);

  virtual ~FirewireCamera();

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

  bool    iso_mode_enabled();

  virtual bool           supports_focus();
  virtual bool           auto_focus();
  virtual void           set_auto_focus(bool enabled);
  virtual unsigned int   focus();
  virtual void           set_focus(unsigned int focus);
  virtual unsigned int   focus_min();
  virtual unsigned int   focus_max();

 private:
  int  drop_frames;
  int  num_buffers;
  bool opened;
  bool started;
  bool _auto_focus;
  bool valid_frame_received;

  dc1394::dc1394video_mode_t     mode;
  dc1394::dc1394speed_t          speed;
  dc1394::dc1394framerate_t      framerate;
  dc1394::dc1394camera_t        *camera;
  dc1394::dc1394camera_t       **cameras;
  dc1394::dc1394video_frame_t   *frame;
  dc1394::dc1394color_coding_t   format7_coding;
  int                            format7_width;
  int                            format7_height;
  int                            format7_startx;
  int                            format7_starty;
};

#endif
