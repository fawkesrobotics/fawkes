
/***************************************************************************
 *  v4l.cpp - General Video4Linux access
 *
 *  Generated: Sat Jul  5 16:16:16 2008
 *  Copyright  2008  Tobias Kellner
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

#include <fvcams/v4l.h>

#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef HAVE_V4L1_CAM
#include <linux/videodev.h>
#include <fvcams/v4l1.h>
#endif

#ifdef HAVE_V4L2_CAM
#include <linux/videodev2.h>
#include <fvcams/v4l2.h>
#endif

#include <fvutils/system/camargp.h>
#include <core/exception.h>
#include <core/exceptions/software.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class V4LCamera <fvcams/v4l.h>
 * General Video4Linux camera implementation.
 * Maintains backwards compatibility.
 * Chooses on the fly whether v4l1 or v4l2 is needed for a given device.
 * @author Tobias Kellner
 */

/** Constructor.
 * @param device_name device file name (e.g. /dev/video0)
 */
V4LCamera::V4LCamera(const char *device_name)
{
  _v4l_cam = NULL;
  _device_name = strdup(device_name);
}


/** Constructor.
 * Initialize camera with parameters from camera argument parser.
 * Supported arguments:
 * - device=DEV, device file, for example /dev/video0
 * @param cap camera argument parser
 */
V4LCamera::V4LCamera(const CameraArgumentParser *cap)
{
  _v4l_cam = NULL;
  if (cap->has("device")) _device_name = strdup(cap->get("device").c_str());
  else throw fawkes::MissingParameterException("Missing device for V4lCamera");
}

/** Destructor. */
V4LCamera::~V4LCamera()
{
  free(_device_name);
  if (_v4l_cam) delete _v4l_cam;
}

void
V4LCamera::open()
{
  if (_v4l_cam) delete _v4l_cam;

  int dev = ::open(_device_name, O_RDWR);
  if (dev < 0) throw fawkes::Exception("V4LCam: Could not open device");

#ifdef HAVE_V4L1_CAM
  struct video_capability caps1;
#endif
#ifdef HAVE_V4L2_CAM
  struct v4l2_capability caps2;
#endif

#ifdef HAVE_V4L2_CAM
  if (ioctl(dev, VIDIOC_QUERYCAP, &caps2))
  {
#endif
#ifdef HAVE_V4L1_CAM
    if (ioctl(dev, VIDIOCGCAP, &caps1))
    {
#endif
      throw fawkes::Exception("V4LCam: Device doesn't appear to be a v4l device");
#ifdef HAVE_V4L1_CAM
    }
    _v4l_cam = new V4L1Camera(_device_name, dev);
#endif
#ifdef HAVE_V4L2_CAM
  }
  else
  {
    _v4l_cam = new V4L2Camera(_device_name, dev);
  }
#endif
}


void
V4LCamera::start()
{
  if (!_v4l_cam) throw fawkes::Exception("V4LCam: Trying to start closed cam!");

  _v4l_cam->start();
}

void
V4LCamera::stop()
{
  if (!_v4l_cam) throw fawkes::Exception("V4LCam: Trying to stop closed cam!");

  _v4l_cam->stop();
}

void
V4LCamera::close()
{
  if (_v4l_cam) _v4l_cam->close();
}

void
V4LCamera::flush()
{
  if (_v4l_cam) _v4l_cam->flush();
}

void
V4LCamera::capture()
{
  if (_v4l_cam) _v4l_cam->capture();
}

void
V4LCamera::print_info()
{
  if (_v4l_cam) _v4l_cam->print_info();
}

bool
V4LCamera::ready()
{
  return (_v4l_cam ? _v4l_cam->ready() : false);
}

unsigned char*
V4LCamera::buffer()
{
  return (_v4l_cam ? _v4l_cam->buffer() : NULL);
}

unsigned int
V4LCamera::buffer_size()
{
  return (_v4l_cam ? _v4l_cam->buffer_size() : 0);
}

void
V4LCamera::dispose_buffer()
{
  if (_v4l_cam) _v4l_cam->dispose_buffer();
}

unsigned int
V4LCamera::pixel_width()
{
  if (!_v4l_cam) throw fawkes::Exception("V4LCam::pixel_width(): Camera not opened");

  return _v4l_cam->pixel_width();
}

unsigned int
V4LCamera::pixel_height()
{
  if (!_v4l_cam) throw fawkes::Exception("V4LCam::pixel_height(): Camera not opened");

  return _v4l_cam->pixel_height();
}

colorspace_t
V4LCamera::colorspace()
{
  return (_v4l_cam ? _v4l_cam->colorspace() : CS_UNKNOWN);
}

void
V4LCamera::set_image_number(unsigned int n)
{
  if (_v4l_cam) _v4l_cam->set_image_number(n);
}

} // end namespace firevision
