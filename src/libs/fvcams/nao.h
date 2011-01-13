
/***************************************************************************
 *  nao.h - V4L2 camera with Nao specific extensions
 *
 *  Created: Sun Feb 01 13:56:23 2009
 *  Copyright  2008 Tobias Kellner
 *             2009 Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CAMS_NAO_H_
#define __FIREVISION_CAMS_NAO_H_

#include <cams/v4l2.h>

#include <cams/control/source.h>

#include <core/exceptions/software.h>
#include <unistd.h>

#define DSPIC_I2C_ADDR 0x8
#define DSPIC_SWITCH_REG 220
#define I2C_SLAVE 0x0703

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class NaoCamera :
  public V4L2Camera,
  public CameraControlSource
{
 public:
  NaoCamera(const CameraArgumentParser *cap);
  virtual ~NaoCamera();

  virtual unsigned char source();
  virtual void          set_source(unsigned char source);
  virtual bool          auto_exposure();
  virtual void          set_auto_exposure(bool enabled);

 private:
  static int open_dev(const char *i2c);
  static void close_dev(int dev, const char *error = 0);
  static int get_open_cam_id(int dev);
  static void switch_to_cam_id(int dev, int cam_id);
  static void init_cam(const char *cam);

 private:
  char *__i2c_device_name; ///< I2C device file descriptor
  int   __cam_id;          ///< ID of the Camera to be used
  bool  __can_switch_cam;  ///< Needs to be Nao V3 for camera switching
};

} // end namespace firevision

#endif //__FIREVISION_CAMS_V4L2_H_

