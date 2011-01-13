
/***************************************************************************
 *  nao.h - V4L2 camera with Nao specific extensions
 *
 *  Created: Sun Feb 01 13:57:43 2009
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

#include <cams/nao.h>
#include <fvutils/system/camargp.h>
#include <utils/logging/liblogger.h>
#include <core/exceptions/software.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>

#include <cstring>
#include <cstdlib>
#include <vector>

using namespace fawkes;

#define V4L2_CID_AUTOEXPOSURE           (V4L2_CID_BASE+32)
#define V4L2_CID_CAM_INIT               (V4L2_CID_BASE+33)
#define V4L2_CID_EXPOSURE_CORRECTION    (V4L2_CID_BASE+34)
#define V4L2_CID_AEC_ALGORITHM          (V4L2_CID_BASE+35)

#ifndef I2C_FUNC_SMBUS_READ_BYTE_DATA
#include <linux/i2c.h>
/// @cond I2C_INTERNALS

// this is the bare minimum of I2C code required to build this thing without
// the extended i2c.h header from i2c-tools, copied straight from version 3.0.1.

static inline __s32
i2c_smbus_access(int file, char read_write, __u8 command,
                 int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = read_write;
  args.command = command;
  args.size = size;
  args.data = data;
  return ioctl(file,I2C_SMBUS,&args);
}

static inline __s32
i2c_smbus_read_byte_data(int file, __u8 command)
{
  union i2c_smbus_data data;
  if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
                       I2C_SMBUS_BYTE_DATA,&data))
    return -1;
  else
    return 0x0FF & data.byte;
}

static inline __s32
i2c_smbus_write_block_data(int file, __u8 command,
                           __u8 length, __u8 *values)
{
  union i2c_smbus_data data;
  int i;
  if (length > 32)
    length = 32;
  for (i = 1; i <= length; i++)
    data.block[i] = values[i-1];
  data.block[0] = length;
  return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
                          I2C_SMBUS_BLOCK_DATA, &data);
}

/// @endcond
#endif

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class NaoCamera <cams/nao.h>
 * Video4Linux 2 camera with Nao-specific extensions.
 *
 * @author Tobias Kellner
 * @author Tim Niemueller
 */


/** Constructor.
 * Initialize camera with parameters from camera argument parser.
 * Supported arguments (additionally to V4L2Camera arguments):
 * *Required:
 * - i2c_device=DEV, i2c device file, for example /dev/i2c-0 (required)
 * *Optional:
 * - cam=brow/mouth string to identify camera, default is mouth
 * @param cap camera argument parser
 */
NaoCamera::NaoCamera(const CameraArgumentParser *cap)
  : V4L2Camera(cap)
{
  if (cap->has("i2c_device")) __i2c_device_name = strdup(cap->get("i2c_device").c_str());
  else throw MissingParameterException("NaoCamera: Missing I2C device");

  __can_switch_cam = false;
  __cam_id = 2;

  if (cap->has("cam"))
  {
    if (strcasecmp(cap->get("cam").c_str(), "brow") == 0) __cam_id = 1;
  }

  int dev = open_dev(__i2c_device_name);

  // Get dsPIC version (in order to know Nao version)
  int val = i2c_smbus_read_byte_data(dev, 170);
  if (val == -1) close_dev(dev, "NaoCamera: Error reading dsPic version from I2C");
  if (val < 2)
  {
    LibLogger::log_info("NaoCamera", "Nao V2 found - No camera switching possible");
    close_dev(dev);
    return;
  }
  __can_switch_cam = true;
  LibLogger::log_debug("NaoCamera", "Nao V3 found - Trying to switch to camera %d", __cam_id);

  val = get_open_cam_id(dev);

  if (val == __cam_id)
  {
    LibLogger::log_debug("NaoCamera", "Correct camera already chosen");
  }
  else
  {
    // Switch to other camera
    switch_to_cam_id(dev, __cam_id);
  }
  close_dev(dev);

  // Connect to the chosen camera and initialize it
  // FIXME: Maybe not needed? Try it!
  init_cam(_device_name);
}

NaoCamera::~NaoCamera()
{
  free(__i2c_device_name);
}

/**
 * Helper function to open the I2C device
 * @param i2c I2C device name
 * @return device handle
 */
int NaoCamera::open_dev(const char *i2c)
{
  // Connect to dsPIC through I2C
  int dev = ::open(i2c, O_RDWR);
  if (dev < 0) throw Exception("NaoCamera: Error opening I2C for connection to dsPIC");
  if (ioctl(dev, I2C_SLAVE, DSPIC_I2C_ADDR) < 0) close_dev(dev, "NaoCamera: Can't connect I2C to dsPIC");
  return dev;
}

/**
 * Helper function called when something fails during camera switching.
 * Closes the opened device and reports an error (if any) by throwing
 * an exception.
 *
 * @param dev the device to close
 * @param error null if no error, an error string otherwise
 */
void NaoCamera::close_dev(int dev, const char *error)
{
  if (::close(dev) < 0) throw fawkes::Exception("NaoCamera: Error closing device");
  if (error) throw fawkes::Exception(error);
}

/**
 * Helper function to get the ID of the currently opened camera
 * @param dev I2C device handle
 */
int NaoCamera::get_open_cam_id(int dev)
{
  // Ask dsPIC which camera is active
  int cid = i2c_smbus_read_byte_data(dev, 220);
  if (cid == -1) close_dev(dev, "Error reading active cam from I2C");
  return cid;
}

/**
 * Helper function to switch to another camera
 * @param dev I2C device handle
 * @param cam_id ID of the camera to open
 */
void NaoCamera::switch_to_cam_id(int dev, int cam_id)
{
  unsigned char cmd[2];
  cmd[0] = cam_id;
  cmd[1] = 0;
  int size = i2c_smbus_write_block_data(dev, 220, 1, cmd);
  if (size == -1) close_dev(dev, "NaoCamera: Error switching to other camera");
}

/**
 * Helper function to initialize a camera
 * @param cam Camera device name
 */
void NaoCamera::init_cam(const char *cam)
{
  int dev = ::open(cam, O_RDWR);
  if (dev < 0)  throw Exception("NaoCamera: Error opening Camera");

  struct v4l2_control control;
  memset(&control, 0, sizeof(control));

  control.id    = V4L2_CID_CAM_INIT;
  control.value = 0;

  if (ioctl(dev, VIDIOC_S_CTRL, &control)) close_dev(dev, "Error setting other camera to default parameters");

  close_dev(dev);
}

/**
 * Return which cam is currently being used.
 * 1: brow-cam
 * 2: mouth-cam
 * @return ID of camera currently in use
 */
unsigned char NaoCamera::source()
{
  int dev = open_dev(__i2c_device_name);
  __cam_id = get_open_cam_id(dev);
  close_dev(dev);

  return static_cast<unsigned char>(__cam_id);
}

/**
 * Switch currently used camera.
 * Valid arguments:
 * 1: brow-cam
 * 2: mouth-cam
 * @param source ID of the camera to use
 */
void NaoCamera::set_source(unsigned char source)
{
  if (source == __cam_id)
  {
    LibLogger::log_debug("NaoCamera", "Correct camera already chosen");
    return;
  }

  int dev = open_dev(__i2c_device_name);
  switch_to_cam_id(dev, source);
  close_dev(dev);
  init_cam(_device_name);
}

/**
 * Return whether auto exposure is enabled.
 * @return true if auto exposure is enabled
 */
bool NaoCamera::auto_exposure()
{
  return get_one_control("AEC", V4L2_CID_AUTOEXPOSURE);
}

/**
 * Enable/disable auto exposure.
 * @param enabled whether auto exposure should be enabled
 */
void NaoCamera::set_auto_exposure(bool enabled)
{
  LibLogger::log_debug("NaoCamera", (enabled ? "enabling AEC" : "disabling AEC"));

  set_one_control("AEC", V4L2_CID_AUTOEXPOSURE, (enabled ? 1 : 0));
}

} // end namespace firevision

