
/***************************************************************************
 *  v4l2.cpp - Video4Linux 2 camera access
 *
 *  Created: Sat Jul  5 20:40:20 2008
 *  Copyright  2008  Tobias Kellner
 *             2010  Tim Niemueller
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

#include <fvcams/v4l2.h>

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <logging/liblogger.h>
#include <fvutils/system/camargp.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <linux/version.h>

using std::cout;
using std::endl;
using std::string;
using fawkes::Exception;
using fawkes::MissingParameterException;
using fawkes::NotImplementedException;
using fawkes::LibLogger;

#ifdef HAVE_LIBV4L2
#  include <libv4l2.h>
#else
#  include <unistd.h>
#  define v4l2_fd_open(fd, flags) (fd)
#  define v4l2_close ::close
#  define v4l2_dup dup
#  define v4l2_ioctl ioctl
#  define v4l2_read read
#  define v4l2_mmap mmap
#  define v4l2_munmap munmap
#endif 

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS
class V4L2CameraData
{
 public:
  v4l2_capability caps;        //< Device capabilites
};

/// @endcond


/** @class V4L2Camera <fvcams/v4l2.h>
 * Video4Linux 2 camera access implementation.
 *
 * @todo UPTR method
 * @todo Standards queries (VIDIOC_ENUMSTD)
 * @todo v4l2_pix_format.field
 * @author Tobias Kellner
 */


/** Constructor.
 * @param device_name device file name (e.g. /dev/video0)
 */
V4L2Camera::V4L2Camera(const char *device_name)
{
  _opened = _started = false;
  _nao_hacks = _switch_u_v = false;
  _width = _height = _bytes_per_line = _fps = _buffers_length = 0;
  _current_buffer = -1;
  _brightness.set = _contrast.set = _saturation.set = _hue.set =
    _red_balance.set = _blue_balance.set = _exposure.set = _gain.set =
    _lens_x.set = _lens_y.set = false;
  _aec = _awb = _agc = _h_flip = _v_flip = NOT_SET;
  _read_method = MMAP;
  memset(_format, 0, 5);
  _frame_buffers = NULL;
  _capture_time = NULL;
  _device_name = strdup(device_name);
  _data = new V4L2CameraData();
}


/** Constructor.
 * Initialize camera with parameters from camera argument parser.
 * Supported arguments:
 * *Required:
 * - device=DEV, device file, for example /dev/video0 (required)
 * *Optional:
 * - read_method=METHOD, preferred read method
 *    READ: read()
 *    MMAP: memory mapping
 *    UPTR: user pointer
 * - format=FOURCC, preferred format
 * - size=WIDTHxHEIGHT, preferred image size
 * - switch_u_v=true/false, switch U and V channels
 * - fps=FPS, frames per second
 * - aec=true/false, Auto Exposition enabled [warning: only valid on nao]
 * - awb=true/false, Auto White Balance enabled
 * - agc=true/false, Auto Gain enabled
 * - h_flip=true/false, Horizontal mirror
 * - v_flip=true/false, Vertical mirror
 * - brightness=BRIGHT, Brightness [0-255] (def. 128)
 * - contrast=CONTR, Contrast [0-127] (def. 64)
 * - saturation=SAT, Saturation [0-256] (def. 128)
 * - hue=HUE, Hue [-180-180] (def. 0)
 * - red_balance=RB, Red Balance [0-255] (def. 128)
 * - blue_balance=BB, Blue Balance [0-255] (def. 128)
 * - exposure=EXP, Exposure [0-65535] (def. 60)
 * - gain=GAIN, Gain [0-255] (def. 0)
 * - lens_x=CORR, Lens Correction X [0-255] (def. 0)
 * - lens_y=CORR, Lens Correction Y [0-255] (def. 0)
 * @param cap camera argument parser
 */
V4L2Camera::V4L2Camera(const CameraArgumentParser *cap)
{
  _opened = _started = false;
  _nao_hacks = false;
  _width = _height = _bytes_per_line = _buffers_length = 0;
  _current_buffer = -1;
  _frame_buffers = NULL;
  _capture_time = NULL;
  _data = new V4L2CameraData();

  if (cap->has("device")) _device_name = strdup(cap->get("device").c_str());
  else throw MissingParameterException("V4L2Cam: Missing device");


  if (cap->has("read_method"))
  {
    string rm = cap->get("read_method");
    if (rm.compare("READ") == 0) _read_method = READ;
    else if (rm.compare("MMAP") == 0) _read_method = MMAP;
    else if (rm.compare("UPTR") == 0) _read_method = UPTR;
    else throw Exception("V4L2Cam: Invalid read method");
  }
  else
  {
    _read_method = MMAP;
  }


  if (cap->has("format"))
  {
    string fmt = cap->get("format");
    if (fmt.length() != 4) throw Exception("V4L2Cam: Invalid format fourcc");
    strncpy(_format, fmt.c_str(), 4);
    _format[4] = '\0';
  }
  else
  {
    memset(_format, 0, 5);
  }


  if (cap->has("size"))
  {
    string size = cap->get("size");
    string::size_type pos;
    if ((pos = size.find('x')) == string::npos) throw Exception("V4L2Cam: invalid image size string");
    if ((pos == (size.length() - 1))) throw Exception("V4L2Cam: invalid image size string");

    unsigned int mult = 1;
    for (string::size_type i = pos - 1; i != string::npos; --i)
    {
      _width += (size.at(i) - '0') * mult;
      mult *= 10;
    }

    mult = 1;
    for (string::size_type i = size.length() - 1; i > pos; --i)
    {
      _height += (size.at(i) - '0') * mult;
      mult *= 10;
    }
  }


  if (cap->has("switch_u_v"))
  {
    _switch_u_v = (cap->get("switch_u_v").compare("true") == 0);
  }
  else
  {
    _switch_u_v = false;
  }


  if (cap->has("fps"))
  {
    if ((_fps = atoi(cap->get("fps").c_str())) == 0) throw Exception("V4L2Cam: invalid fps string");
  }
  else
  {
    _fps = 0;
  }


  if (cap->has("aec"))
  {
    _aec = (cap->get("aec").compare("true") == 0 ? TRUE : FALSE);
  }
  else
  {
    _aec = NOT_SET;
  }


  if (cap->has("awb"))
  {
    _awb = (cap->get("awb").compare("true") == 0 ? TRUE : FALSE);
  }
  else
  {
    _awb = NOT_SET;
  }


  if (cap->has("agc"))
  {
    _agc = (cap->get("agc").compare("true") == 0 ? TRUE : FALSE);
  }
  else
  {
    _agc = NOT_SET;
  }


  if (cap->has("h_flip"))
  {
    _h_flip = (cap->get("h_flip").compare("true") == 0 ? TRUE : FALSE);
  }
  else
  {
    _h_flip = NOT_SET;
  }


  if (cap->has("v_flip"))
  {
    _v_flip = (cap->get("v_flip").compare("true") == 0 ? TRUE : FALSE);
  }
  else
  {
    _v_flip = NOT_SET;
  }


  if (cap->has("brightness"))
  {
    _brightness.set = true;
    _brightness.value = atoi(cap->get("brightness").c_str());
  }
  else
  {
    _brightness.set = false;
  }


  if (cap->has("contrast"))
  {
    _contrast.set = true;
    _contrast.value = atoi(cap->get("contrast").c_str());
  }
  else
  {
    _contrast.set = false;
  }


  if (cap->has("saturation"))
  {
    _saturation.set = true;
    _saturation.value = atoi(cap->get("saturation").c_str());
  }
  else
  {
    _saturation.set = false;
  }


  if (cap->has("hue"))
  {
    _hue.set = true;
    _hue.value = atoi(cap->get("hue").c_str());
  }
  else
  {
    _hue.set = false;
  }


  if (cap->has("red_balance"))
  {
    _red_balance.set = true;
    _red_balance.value = atoi(cap->get("red_balance").c_str());
  }
  else
  {
    _red_balance.set = false;
  }


  if (cap->has("blue_balance"))
  {
    _blue_balance.set = true;
    _blue_balance.value = atoi(cap->get("blue_balance").c_str());
  }
  else
  {
    _blue_balance.set = false;
  }


  if (cap->has("exposure"))
  {
    _exposure.set = true;
    _exposure.value = atoi(cap->get("exposure").c_str());
  }
  else
  {
    _exposure.set = false;
  }


  if (cap->has("gain"))
  {
    _gain.set = true;
    _gain.value = atoi(cap->get("gain").c_str());
  }
  else
  {
    _gain.set = false;
  }


  if (cap->has("lens_x"))
  {
    _lens_x.set = true;
    _lens_x.value = atoi(cap->get("lens_x").c_str());
  }
  else
  {
    _lens_x.set = false;
  }


  if (cap->has("lens_y"))
  {
    _lens_y.set = true;
    _lens_y.value = atoi(cap->get("lens_y").c_str());
  }
  else
  {
    _lens_y.set = false;
  }
}


/** Protected Constructor.
 * Gets called from V4LCamera, when the device has already been opened
 * and determined to be a V4L2 device.
 * @param device_name device file name (e.g. /dev/video0)
 * @param dev file descriptor of the opened device
 */
V4L2Camera::V4L2Camera(const char *device_name, int dev)
{
  _opened = true;
  _started = false;
  _nao_hacks = _switch_u_v = false;
  _width = _height = _bytes_per_line = _buffers_length = _fps = 0;
  _current_buffer = -1;
  _brightness.set = _contrast.set = _saturation.set = _hue.set =
    _red_balance.set = _blue_balance.set = _exposure.set = _gain.set =
    _lens_x.set = _lens_y.set = false;
  _aec = _awb = _agc = _h_flip = _v_flip = NOT_SET;
  _read_method = UPTR;
  memset(_format, 0, 5);
  _frame_buffers = NULL;
  _capture_time = NULL;
  _device_name = strdup(device_name);
  _data = new V4L2CameraData();

  _dev = dev;

  // getting capabilities
  if (v4l2_ioctl(_dev, VIDIOC_QUERYCAP, &_data->caps))
  {
    close();
    throw Exception("V4L2Cam: Could not get capabilities - probably not a v4l2 device");
  }

  post_open();
}

/** Destructor. */
V4L2Camera::~V4L2Camera()
{
  if (_started) stop();
  if (_opened) close();

  free(_device_name);
  delete _data;
}

void
V4L2Camera::open()
{
  if (_started) stop();
  if(_opened) close();

  _dev = ::open(_device_name, O_RDWR);
  int libv4l2_fd = v4l2_fd_open(_dev, 0);
  if (libv4l2_fd != -1)  _dev = libv4l2_fd;
  /* Note the v4l2_xxx functions are designed so that if they get passed an
     unknown fd, the will behave exactly as their regular xxx counterparts, so
     if v4l2_fd_open fails, we continue as normal (missing the libv4l2 custom
     cam format to normal formats conversion). Chances are big we will still
     fail then though, as normally v4l2_fd_open only fails if the device is not
     a v4l2 device. */ 
  if (_dev < 0) throw Exception("V4L2Cam: Could not open device");

  _opened = true;

  // getting capabilities
  if (v4l2_ioctl(_dev, VIDIOC_QUERYCAP, &_data->caps))
  {
    close();
    throw Exception("V4L2Cam: Could not get capabilities - probably not a v4l2 device");
  }

  post_open();
}

/**
 * Post-open() operations.
 * Precondition: _dev (file desc) and _data->caps (capabilities) are set.
 * @param dev file descriptor of the opened device
 */
void
V4L2Camera::post_open()
{
  //LibLogger::log_debug("V4L2Cam", "select_read_method()");
  select_read_method();

  //LibLogger::log_debug("V4L2Cam", "select_format()");
  select_format();

  if (_fps)
  {
    //LibLogger::log_debug("V4L2Cam", "set_fps()");
    set_fps();
  }

  //LibLogger::log_debug("V4L2Cam", "set_controls()");
  set_controls();

  //LibLogger::log_debug("V4L2Cam", "create_buffer()");
  create_buffer();

  //LibLogger::log_debug("V4L2Cam", "reset_cropping()");
  reset_cropping();

}

/**
 * Find suitable reading method.
 * The one set in _read_method is preferred.
 * Postconditions:
 *  - _read_method and _buffers_length are set
 */
void
V4L2Camera::select_read_method()
{
  /* try preferred method */
  if (!(_data->caps.capabilities &
        (_read_method == READ ? V4L2_CAP_READWRITE : V4L2_CAP_STREAMING)))
  {
    /* preferred read method not supported - try next */
    _read_method = (_read_method == READ ? MMAP : READ);
    if (!(_data->caps.capabilities &
          (_read_method == READ ? V4L2_CAP_READWRITE : V4L2_CAP_STREAMING)))
    {
      close();
      throw Exception("V4L2Cam: Neither read() nor streaming IO supported");
    }
  }

  if (_read_method != READ)
  {
    v4l2_requestbuffers buf;

    /* Streaming IO - Try 1st method, and if that fails 2nd */
    for (int i = 0; i < 2; ++i)
    {
      if (_read_method == MMAP)
      {
        _buffers_length = MMAP_NUM_BUFFERS;
        buf.count = _buffers_length;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
      }
      else /* UPTR */
      {
        _buffers_length = 0;
        buf.count = 0;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
      }

      if (v4l2_ioctl(_dev, VIDIOC_REQBUFS, &buf))
      {
        if (errno != EINVAL)
        {
          close();
          throw Exception("V4L2Cam: REQBUFS query failed");
        }

        /* Not supported */
        if (i == 1)
        {
          close();
          throw Exception("V4L2Cam: Neither memory mapped nor user pointer IO supported");
        }

        /* try other method */
        _read_method = (_read_method == MMAP ? UPTR : MMAP);
        continue;
      }

      /* Method supported */
      if ((_read_method == MMAP) && (buf.count < _buffers_length))
      {
        close();
        throw Exception("V4L2Cam: Not enough memory for the buffers");
      }

      break;
    }
  }
  else /* Read IO */
  {
    _buffers_length = 1;
  }

  switch (_read_method)
  {
    case READ:
      LibLogger::log_debug("V4L2Cam", "Using read() method");
      break;

    case MMAP:
      LibLogger::log_debug("V4L2Cam", "Using memory mapping method");
      break;

    case UPTR:
      LibLogger::log_debug("V4L2Cam", "Using user pointer method");
      //TODO
      throw Exception("V4L2Cam: user pointer method not supported yet");
      break;
  }
}

/**
 * Find suitable image format.
 * The one set in _format (if any) is preferred.
 * Postconditions:
 *  - _format is set (and selected)
 *  - _colorspace is set accordingly
 *  - _width, _height, and _bytes_per_line are set
 */
void
V4L2Camera::select_format()
{
  bool preferred_found = false;
  v4l2_fmtdesc format_desc;

  char fourcc[5] = "    ";

  if (strcmp(_format, ""))
  {
    /* Try to select preferred format */
    memset(&format_desc, 0, sizeof(format_desc));
    format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (format_desc.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUM_FMT, &format_desc) == 0; format_desc.index++)
    {
      fourcc[0] = static_cast<char>(format_desc.pixelformat & 0xFF);
      fourcc[1] = static_cast<char>((format_desc.pixelformat >> 8) & 0xFF);
      fourcc[2] = static_cast<char>((format_desc.pixelformat >> 16) & 0xFF);
      fourcc[3] = static_cast<char>((format_desc.pixelformat >> 24) & 0xFF);

      if (strcmp(_format, fourcc) == 0)
      {
        preferred_found = true;
        break;
      }
    }
  }

  if (!preferred_found)
  {
    /* Preferred format not found (or none selected)
       -> just take first available format */
    memset(&format_desc, 0, sizeof(format_desc));
    format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format_desc.index = 0;
    if (v4l2_ioctl(_dev, VIDIOC_ENUM_FMT, &format_desc))
    {
      close();
      throw Exception("V4L2Cam: No image format found");
    }

    fourcc[0] = static_cast<char>(format_desc.pixelformat & 0xFF);
    fourcc[1] = static_cast<char>((format_desc.pixelformat >> 8) & 0xFF);
    fourcc[2] = static_cast<char>((format_desc.pixelformat >> 16) & 0xFF);
    fourcc[3] = static_cast<char>((format_desc.pixelformat >> 24) & 0xFF);
  }

  /* Now set this format */
  v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (v4l2_ioctl(_dev, VIDIOC_G_FMT, &format))
  {
    close();
    throw Exception("V4L2Cam: Format query failed");
  }

  //LibLogger::log_debug("V4L2Cam", "setting %dx%d (%s) - type %d", _width, _height, fourcc, format.type);

  format.fmt.pix.pixelformat = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
  format.fmt.pix.field       = V4L2_FIELD_ANY;
  if (_width)
    format.fmt.pix.width = _width;
  if (_height)
    format.fmt.pix.height = _height;

  int s_fmt_rv = v4l2_ioctl(_dev, VIDIOC_S_FMT, &format);
  if (s_fmt_rv != 0 && errno != EBUSY)
  {
    //throw Exception(errno, "Failed to set video format");
    //}

    // Nao workaround (Hack alert)
    LibLogger::log_warn("V4L2Cam", "Format setting failed (driver sucks) - %d: %s", errno, strerror(errno));
    LibLogger::log_info("V4L2Cam", "Trying workaround");
    _nao_hacks = true;

    v4l2_std_id std;
    if (v4l2_ioctl(_dev, VIDIOC_G_STD, &std))
    {
      close();
      throw Exception("V4L2Cam: Standard query (workaround) failed");
    }

    if ((_width == 320) && (_height == 240))
    {
      std = 0x04000000UL; // QVGA
    }
    else
    {
      std = 0x08000000UL; // VGA
      _width = 640;
      _height = 480;
    }
    if (v4l2_ioctl(_dev, VIDIOC_S_STD, &std))
    {
      close();
      throw Exception("V4L2Cam: Standard setting (workaround) failed");
    }

    format.fmt.pix.width       = _width;
    format.fmt.pix.height      = _height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format.fmt.pix.field       = V4L2_FIELD_ANY;

    if (v4l2_ioctl(_dev, VIDIOC_S_FMT, &format))
    {
      close();
      throw Exception("V4L2Cam: Format setting (workaround) failed");
    }

    if (_switch_u_v) _colorspace = YVY2;
  }

  /* ...and store final values */
  _format[0] = static_cast<char>(format.fmt.pix.pixelformat & 0xFF);
  _format[1] = static_cast<char>((format.fmt.pix.pixelformat >> 8) & 0xFF);
  _format[2] = static_cast<char>((format.fmt.pix.pixelformat >> 16) & 0xFF);
  _format[3] = static_cast<char>((format.fmt.pix.pixelformat >> 24) & 0xFF);

  if (!_nao_hacks || !_switch_u_v)
  {
    if (strcmp(_format, "RGB3") == 0) _colorspace = RGB;
    else if (strcmp(_format, "Y41P") == 0) _colorspace = YUV411_PACKED; //different byte ordering
    else if (strcmp(_format, "411P") == 0) _colorspace = YUV411_PLANAR;
    else if (strcmp(_format, "YUYV") == 0) _colorspace = YUY2;
    else if (strcmp(_format, "BGR3") == 0) _colorspace = BGR;
    else if (strcmp(_format, "UYVY") == 0) _colorspace = YUV422_PACKED;
    else if (strcmp(_format, "422P") == 0) _colorspace = YUV422_PLANAR;
    else if (strcmp(_format, "GREY") == 0) _colorspace = GRAY8;
    else if (strcmp(_format, "RGB4") == 0) _colorspace = RGB_WITH_ALPHA;
    else if (strcmp(_format, "BGR4") == 0) _colorspace = BGR_WITH_ALPHA;
    else if (strcmp(_format, "BA81") == 0) _colorspace = BAYER_MOSAIC_BGGR;
    else if (strcmp(_format, "Y16 ") == 0) _colorspace = MONO16;
    else _colorspace = CS_UNKNOWN;
  }

  if (!_nao_hacks)
  {
    _width = format.fmt.pix.width;
    _height = format.fmt.pix.height;
  }

  _bytes_per_line = format.fmt.pix.bytesperline;

  /* Hack for bad drivers */
  if (_bytes_per_line == 0)
  {
    LibLogger::log_warn("V4L2Cam", "bytesperline is 0 (driver sucks)");
    _bytes_per_line = colorspace_buffer_size(_colorspace, _width, _height) / _height;
  }

  LibLogger::log_debug("V4L2Cam", "w%d h%d bpl%d cs%d fmt%s", _width, _height, _bytes_per_line, _colorspace, _format);
}

/**
 * Set desired FPS count.
 */
void
V4L2Camera::set_fps()
{
  v4l2_streamparm param;
  param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (v4l2_ioctl(_dev, VIDIOC_G_PARM, &param))
  {
    close();
    throw Exception("V4L2Cam: Streaming parameter query failed");
  }

  if (!(param.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
  {
    LibLogger::log_warn("V4L2Cam", "FPS change not supported");
    return;
  }

  param.parm.capture.timeperframe.numerator = 1;
  param.parm.capture.timeperframe.denominator = _fps;
  if (v4l2_ioctl(_dev, VIDIOC_S_PARM, &param))
  {
    close();
    throw Exception("V4L2Cam: Streaming parameter setting failed");
  }
  else
  {
    LibLogger::log_debug("V4L2Cam", "FPS set - %d/%d",
                         param.parm.capture.timeperframe.numerator,
                         param.parm.capture.timeperframe.denominator);
  }
}

/**
 * Set Camera controls.
 */
void
V4L2Camera::set_controls()
{
  if (_aec != NOT_SET) set_auto_exposure(_aec == TRUE);
  if (_awb != NOT_SET) set_auto_white_balance(_awb == TRUE);
  if (_agc != NOT_SET) set_auto_gain(_agc == TRUE);

  if (_h_flip != NOT_SET) set_horiz_mirror(_h_flip == TRUE);
  if (_v_flip != NOT_SET) set_vert_mirror(_v_flip == TRUE);

  if (_brightness.set)   set_brightness(_brightness.value);
  if (_contrast.set)     set_contrast(_contrast.value);
  if (_saturation.set)   set_saturation(_saturation.value);
  if (_hue.set)          set_hue(_hue.value);
  if (_red_balance.set)  set_red_balance(_red_balance.value);
  if (_blue_balance.set) set_blue_balance(_blue_balance.value);
  if (_exposure.set)     set_exposure(_exposure.value);
  if (_gain.set)         set_gain(_gain.value);
  if (_lens_x.set)       set_lens_x_corr(_lens_x.value);
  if (_lens_y.set)       set_lens_y_corr(_lens_y.value);
}

/**
 * Set one Camera control value.
 * @param ctrl name of the value
 * @param id ID of the value
 * @param value value to set
 */
void
V4L2Camera::set_one_control(const char *ctrl, unsigned int id, int value)
{
  v4l2_queryctrl queryctrl;
  v4l2_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = id;

  if (v4l2_ioctl(_dev, VIDIOC_QUERYCTRL, &queryctrl))
  {
    if (errno == EINVAL)
    {
      LibLogger::log_error("V4L2Cam", "Control %s not supported", ctrl);
      return;
    }

    close();
    throw Exception("V4L2Cam: %s Control query failed", ctrl);
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    LibLogger::log_error("V4L2Cam", "Control %s disabled", ctrl);
    return;
  }

  memset(&control, 0, sizeof(control));
  control.id = id;
  control.value = value;

  if (v4l2_ioctl(_dev, VIDIOC_S_CTRL, &control))
  {
    close();
    throw Exception("V4L2Cam: %s Control setting failed", ctrl);
  }
}

/**
 * Get one Camera control value.
 * @param ctrl name of the value
 * @param id ID of the value
 * @return current value
 */
int
V4L2Camera::get_one_control(const char *ctrl, unsigned int id)
{
  v4l2_queryctrl queryctrl;
  v4l2_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = id;

  if (v4l2_ioctl(_dev, VIDIOC_QUERYCTRL, &queryctrl))
  {
    if (errno == EINVAL)
    {
      LibLogger::log_error("V4L2Cam", "Control %s not supported", ctrl);
      return 0;
    }

    close();
    throw Exception("V4L2Cam: %s Control query failed", ctrl);
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    LibLogger::log_error("V4L2Cam", "Control %s disabled", ctrl);
    return 0;
  }

  memset(&control, 0, sizeof(control));
  control.id = id;

  if (v4l2_ioctl(_dev, VIDIOC_G_CTRL, &control))
  {
    close();
    throw Exception("V4L2Cam: %s Control value reading failed", ctrl);
  }

  return control.value;
}

/**
 * Create a buffer for image transfer.
 * Preconditions:
 *  - _read_method is set
 *  - _height and _bytes_per_line are set
 *  - _buffers_length is set
 * Postconditions:
 *  - _frame_buffers is set up
 */
void
V4L2Camera::create_buffer()
{
  _frame_buffers = new FrameBuffer[_buffers_length];

  switch (_read_method)
  {
    case READ:
    {
      _frame_buffers[0].size = _bytes_per_line * _height;
      _frame_buffers[0].buffer = static_cast<unsigned char *>(malloc(_frame_buffers[0].size));
      if (_frame_buffers[0].buffer == NULL)
      {
        close();
        throw Exception("V4L2Cam: Out of memory");
      }
      break;
    }

    case MMAP:
    {
      for (unsigned int i = 0; i < _buffers_length; ++i)
      {
        /* Query status of buffer */
        v4l2_buffer buffer;

        memset(&buffer, 0, sizeof (buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (v4l2_ioctl(_dev, VIDIOC_QUERYBUF, &buffer))
        {
          close();
          throw Exception("V4L2Cam: Buffer query failed");
        }

        _frame_buffers[i].size = buffer.length;
        _frame_buffers[i].buffer = static_cast<unsigned char *>(
          v4l2_mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, _dev, buffer.m.offset)
        );
        if (_frame_buffers[i].buffer == MAP_FAILED)
        {
          close();
          throw Exception("V4L2Cam: Memory mapping failed");
        }
      }

      break;
    }

    case UPTR:
      /* not supported yet */
      break;
  }
}

/**
 * Reset cropping parameters.
 */
void
V4L2Camera::reset_cropping()
{
  v4l2_cropcap cropcap;
  v4l2_crop crop;

  memset(&cropcap, 0, sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (v4l2_ioctl(_dev, VIDIOC_CROPCAP, &cropcap))
  {
    LibLogger::log_warn("V4L2Cam", "cropcap query failed (driver sucks) - %d: %s", errno, strerror(errno));
  }

  memset(&crop, 0, sizeof(crop));
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  crop.c = cropcap.defrect;

  /* Ignore if cropping is not supported (EINVAL). */
  if (v4l2_ioctl(_dev, VIDIOC_S_CROP, &crop) && errno != EINVAL)
  {
    LibLogger::log_warn("V4L2Cam", "cropping query failed (driver sucks) - %d: %s", errno, strerror(errno));
  }
}

void
V4L2Camera::close()
{
  //LibLogger::log_debug("V4L2Cam", "close()");

  if (_started) stop();

  if (_frame_buffers)
  {
    switch (_read_method)
    {
      case READ:
      {
        free(_frame_buffers[0].buffer);
        break;
      }

      case MMAP:
      {
        for (unsigned int i = 0; i < _buffers_length; ++i)
        {
          v4l2_munmap(_frame_buffers[i].buffer, _frame_buffers[i].size);
        }
        break;
      }

      case UPTR:
        /* not supported yet */
        break;
    }
    delete[] _frame_buffers;
    _frame_buffers = NULL;
    _current_buffer = -1;
  }

  if (_opened)
  {
    v4l2_close(_dev);
    _opened = false;
    _dev = 0;
  }

  if (_capture_time)
  {
    delete _capture_time;
    _capture_time = 0;
  }
}

void
V4L2Camera::start()
{
  //LibLogger::log_info("V4L2Cam", "start()");

  if (!_opened) throw Exception("VL42Cam: Camera not opened");

  if (_started) stop();

  switch (_read_method)
  {
    case READ:
      /* nothing to do here */
      break;

    case MMAP:
    {
      /* enqueue buffers */
      for (unsigned int i = 0; i < _buffers_length; ++i)
      {
        v4l2_buffer buffer;
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (v4l2_ioctl(_dev, VIDIOC_QBUF, &buffer))
        {
          close();
          throw Exception("V4L2Cam: Enqueuing buffer failed");
        }
      }

      /* start streaming */
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (v4l2_ioctl(_dev, VIDIOC_STREAMON, &type))
      {
        close();
        throw Exception("V4L2Cam: Starting stream failed");
      }
      break;
    }

    case UPTR:
      /* not supported yet */
      break;
  }

  //LibLogger::log_debug("V4L2Cam", "start() complete");
  _started = true;
}

void
V4L2Camera::stop()
{
  //LibLogger::log_debug("V4L2Cam", "stop()");

  if (!_started) return;

  switch (_read_method)
  {
    case READ:
      /* nothing to do here */
      break;

    case MMAP: /* fall through */
    case UPTR:
    {
      /* stop streaming */
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (v4l2_ioctl(_dev, VIDIOC_STREAMOFF, &type))
      {
        close();
        throw Exception("V4L2Cam: Stopping stream failed");
      }
      break;
    }
  }

  _current_buffer = -1;
  _started = false;
}

bool
V4L2Camera::ready()
{
  //LibLogger::log_debug("V4L2Cam", "ready()");

  return _started;
}

void
V4L2Camera::flush()
{
  //LibLogger::log_debug("V4L2Cam", "flush()");
  /* not needed */
}

void
V4L2Camera::capture()
{
  //LibLogger::log_debug("V4L2Cam", "capture()");

  if (!_started) return;

  switch (_read_method)
  {
    case READ:
    {
      _current_buffer = 0;
      //LibLogger::log_debug("V4L2Cam", "calling read()");
      if (v4l2_read(_dev, _frame_buffers[_current_buffer].buffer, _frame_buffers[_current_buffer].size) == -1)
      {
        //TODO: errno handling
        LibLogger::log_warn("V4L2Cam", "read() failed with code %d: %s", errno, strerror(errno));
      }
      //LibLogger::log_debug("V4L2Cam", "returned from read()");

      //No timestamping support here - just take current system time
      if (_capture_time)
      {
        _capture_time->stamp();
      }
      else
      {
        _capture_time = new fawkes::Time();
      }

      break;
    }

    case MMAP:
    {
      /* dequeue buffer */
      v4l2_buffer buffer;
      memset(&buffer, 0, sizeof(buffer));
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;

      if (v4l2_ioctl(_dev, VIDIOC_DQBUF, &buffer))
      {
        //TODO: errno handling -> EAGAIN, ...?
        close();
        throw Exception("V4L2Cam: Dequeuing buffer failed");
      }

      _current_buffer = buffer.index;

      if (_capture_time)
      {
        _capture_time->set_time(&buffer.timestamp);
      }
      else
      {
        _capture_time = new fawkes::Time(&buffer.timestamp);
      }
      break;
    }

    case UPTR:
      /* not supported yet */
      break;
  }
}

unsigned char *
V4L2Camera::buffer()
{
  //LibLogger::log_debug("V4L2Cam", "buffer()");

  return (_current_buffer == -1 ? NULL : _frame_buffers[_current_buffer].buffer);
}

unsigned int
V4L2Camera::buffer_size()
{
  //LibLogger::log_debug("V4L2Cam", "buffer_size()");

  return (_opened && (_current_buffer != -1) ? _frame_buffers[_current_buffer].size : 0);
}

void
V4L2Camera::dispose_buffer()
{
  //LibLogger::log_debug("V4L2Cam", "dispose_buffer()");

  if (!_opened) return;

  switch (_read_method)
  {
    case READ:
      /* nothing to do here */
      break;

    case MMAP:
    {
      /* enqueue next buffer */
      v4l2_buffer buffer;
      memset(&buffer, 0, sizeof(buffer));
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = _current_buffer;

      //TODO: Test if the next buffer is also the latest buffer (VIDIOC_QUERYBUF)
      if (v4l2_ioctl(_dev, VIDIOC_QBUF, &buffer))
      {
        int errno_save = errno;
        close();
        throw Exception(errno_save, "V4L2Cam: Enqueuing buffer failed");
      }
      break;
    }

    case UPTR:
      /* not supported yet */
      break;
  }

  _current_buffer = -1;
}

unsigned int
V4L2Camera::pixel_width()
{
  //LibLogger::log_debug("V4L2Cam", "pixel_width()");

  return _width;
}

unsigned int
V4L2Camera::pixel_height()
{
  //LibLogger::log_debug("V4L2Cam", "pixel_height()");

  return _height;
}

colorspace_t
V4L2Camera::colorspace()
{
  //LibLogger::log_debug("V4L2Cam", "colorspace()");

  if (!_opened)
    return CS_UNKNOWN;
  else
    return _colorspace;
}

fawkes::Time *
V4L2Camera::capture_time()
{
  return _capture_time;
}

void
V4L2Camera::set_image_number(unsigned int n)
{
  //LibLogger::log_debug("V4L2Cam", "set_image_number(%d)", n);

  /* not needed */
}


/* --- CameraControls --- */

bool
V4L2Camera::auto_gain()
{
  return get_one_control("AGC", V4L2_CID_AUTOGAIN);
}

void
V4L2Camera::set_auto_gain(bool enabled)
{
  LibLogger::log_debug("V4L2Cam", (enabled ? "enabling AGC" : "disabling AGC"));
  set_one_control("AGC", V4L2_CID_AUTOGAIN, (enabled ? 1 : 0));
}

bool
V4L2Camera::auto_white_balance()
{
  return get_one_control("AWB", V4L2_CID_AUTO_WHITE_BALANCE);
}

void
V4L2Camera::set_auto_white_balance(bool enabled)
{
  LibLogger::log_debug("V4L2Cam", (enabled ? "enabling AWB" : "disabling AWB"));
  set_one_control("AWB", V4L2_CID_AUTO_WHITE_BALANCE, (enabled ? 1 : 0));
}

bool
V4L2Camera::auto_exposure()
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

void
V4L2Camera::set_auto_exposure(bool enabled)
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

int
V4L2Camera::red_balance()
{
  return get_one_control("red balance", V4L2_CID_RED_BALANCE);
}

void
V4L2Camera::set_red_balance(int red_balance)
{
  LibLogger::log_debug("V4L2Cam", "Setting red balance to %d", red_balance);
  set_one_control("red balance", V4L2_CID_RED_BALANCE, red_balance);
}

int
V4L2Camera::blue_balance()
{
  return get_one_control("blue balance", V4L2_CID_BLUE_BALANCE);
}

void
V4L2Camera::set_blue_balance(int blue_balance)
{
  LibLogger::log_debug("V4L2Cam", "Setting blue balance to %d", blue_balance);
  set_one_control("blue balance", V4L2_CID_BLUE_BALANCE, blue_balance);
}

int
V4L2Camera::u_balance()
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

void
V4L2Camera::set_u_balance(int u_balance)
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

int
V4L2Camera::v_balance()
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

void
V4L2Camera::set_v_balance(int v_balance)
{
  throw NotImplementedException("No such method in the V4L2 standard");
}

unsigned int
V4L2Camera::brightness()
{
  return get_one_control("brightness", V4L2_CID_BRIGHTNESS);
}

void
V4L2Camera::set_brightness(unsigned int brightness)
{
  LibLogger::log_debug("V4L2Cam", "Setting brighness to %d", brightness);
  set_one_control("brightness", V4L2_CID_BRIGHTNESS, brightness);
}

unsigned int
V4L2Camera::contrast()
{
  return get_one_control("contrast", V4L2_CID_CONTRAST);
}

void
V4L2Camera::set_contrast(unsigned int contrast)
{
  LibLogger::log_debug("V4L2Cam", "Setting contrast to %d", contrast);
  set_one_control("contrast", V4L2_CID_CONTRAST, contrast);
}

unsigned int
V4L2Camera::saturation()
{
  return get_one_control("saturation", V4L2_CID_SATURATION);
}

void
V4L2Camera::set_saturation(unsigned int saturation)
{
  LibLogger::log_debug("V4L2Cam", "Setting saturation to %d", saturation);
  set_one_control("saturation", V4L2_CID_SATURATION, saturation);
}

int
V4L2Camera::hue()
{
  return get_one_control("hue", V4L2_CID_HUE);
}

void
V4L2Camera::set_hue(int hue)
{
  LibLogger::log_debug("V4L2Cam", "Setting hue to %d", hue);
  set_one_control("hue", V4L2_CID_HUE, hue);
}

unsigned int
V4L2Camera::exposure()
{
  return get_one_control("exposure", V4L2_CID_EXPOSURE);
}

void
V4L2Camera::set_exposure(unsigned int exposure)
{
  LibLogger::log_debug("V4L2Cam", "Setting exposure to %d", exposure);
  set_one_control("exposure", V4L2_CID_EXPOSURE, exposure);
}

unsigned int
V4L2Camera::gain()
{
  return get_one_control("gain", V4L2_CID_GAIN);
}

void
V4L2Camera::set_gain(unsigned int gain)
{
  LibLogger::log_debug("V4L2Cam", "Setting gain to %u", gain);
  set_one_control("gain", V4L2_CID_GAIN, gain);
}


const char *
V4L2Camera::format()
{
  return _format;
}

void
V4L2Camera::set_format(const char *format)
{
  strncpy(_format, format, 4);
  _format[4] = '\0';
  select_format();
}

unsigned int
V4L2Camera::width()
{
  return pixel_width();
}

unsigned int
V4L2Camera::height()
{
  return pixel_height();
}

void
V4L2Camera::set_size(unsigned int width,
                     unsigned int height)
{
  _width = width;
  _height = height;
  select_format();
}

bool
V4L2Camera::horiz_mirror()
{
  return (get_one_control("hflip", V4L2_CID_HFLIP) != 0);
}

bool
V4L2Camera::vert_mirror()
{
  return (get_one_control("vflip", V4L2_CID_VFLIP) != 0);
}

void
V4L2Camera::set_horiz_mirror(bool enabled)
{
  LibLogger::log_debug("V4L2Cam", (enabled ? "enabling horizontal flip" : "disabling horizontal flip"));
  set_one_control("hflip", V4L2_CID_HFLIP, (enabled ? 1 : 0));
}

void
V4L2Camera::set_vert_mirror(bool enabled)
{
  LibLogger::log_debug("V4L2Cam", (enabled ? "enabling vertical flip" : "disabling vertical flip"));
  set_one_control("vflip", V4L2_CID_VFLIP, (enabled ? 1 : 0));
}

/** Get the number of frames per second that have been requested from the camera.
 * A return value of 0 means that fps haven't been set yet through the camera.
 * @return the currently requested fps or 0 if not set yet
 */
unsigned int
V4L2Camera::fps()
{
  return _fps;
}

void
V4L2Camera::set_fps(unsigned int fps)
{
  _fps = fps;
  set_fps();
}

unsigned int
V4L2Camera::lens_x_corr()
{
  return get_one_control("lens x", V4L2_CID_HCENTER/*_DEPRECATED*/);
}

unsigned int
V4L2Camera::lens_y_corr()
{
  return get_one_control("lens y", V4L2_CID_VCENTER/*_DEPRECATED*/);
}

void
V4L2Camera::set_lens_x_corr(unsigned int x_corr)
{
  LibLogger::log_debug("V4L2Cam", "Setting horizontal lens correction to %d", x_corr);
  set_one_control("lens x", V4L2_CID_HCENTER/*_DEPRECATED*/, x_corr);
}

void
V4L2Camera::set_lens_y_corr(unsigned int y_corr)
{
  LibLogger::log_debug("V4L2Cam", "Setting vertical lens correction to %d", y_corr);
  set_one_control("lens x", V4L2_CID_VCENTER/*_DEPRECATED*/, y_corr);
}


void
V4L2Camera::print_info()
{
 /* General capabilities */
  cout <<
    "=========================================================================="
    << endl << _device_name << " (" << _data->caps.card << ") - " << _data->caps.bus_info
    << endl << "Driver: " << _data->caps.driver << " (ver " <<
    ((_data->caps.version >> 16) & 0xFF) << "." <<
    ((_data->caps.version >> 8) & 0xFF) << "." <<
    (_data->caps.version & 0xFF) << ")" << endl <<
    "--------------------------------------------------------------------------"
    << endl;

  /* General capabilities */
  cout << "Capabilities:" << endl;
  if (_data->caps.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    cout << " + Video capture interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_VIDEO_OUTPUT)
    cout << " + Video output interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_VIDEO_OVERLAY)
    cout << " + Video overlay interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_VBI_CAPTURE)
    cout << " + Raw VBI capture interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_VBI_OUTPUT)
    cout << " + Raw VBI output interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE)
    cout << " + Sliced VBI capture interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT)
    cout << " + Sliced VBI output interface supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_RDS_CAPTURE)
    cout << " + RDS_CAPTURE set" << endl;
  /* Not included in Nao's version
  if (caps.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY)
    cout << " + Video output overlay interface supported" << endl; */
  if (_data->caps.capabilities & V4L2_CAP_TUNER)
    cout << " + Has some sort of tuner" << endl;
  if (_data->caps.capabilities & V4L2_CAP_AUDIO)
    cout << " + Has audio inputs or outputs" << endl;
  if (_data->caps.capabilities & V4L2_CAP_RADIO)
    cout << " + Has a radio receiver" << endl;
  if (_data->caps.capabilities & V4L2_CAP_READWRITE)
    cout << " + read() and write() IO supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_ASYNCIO)
    cout << " + asynchronous IO supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_STREAMING)
    cout << " + streaming IO supported" << endl;
  if (_data->caps.capabilities & V4L2_CAP_TIMEPERFRAME)
    cout << " + timeperframe field is supported" << endl;
  cout << endl;

  /* Inputs */
  cout << "Inputs:" << endl;
  v4l2_input input;
  memset(&input, 0, sizeof(input));

  for (input.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUMINPUT, &input) == 0; input.index++)
  {
    cout << "Input " << input.index << ": " << input.name << endl;

    cout << " |- Type: ";
    switch (input.type)
    {
      case V4L2_INPUT_TYPE_TUNER:
        cout << "Tuner";
        break;

      case V4L2_INPUT_TYPE_CAMERA:
        cout << "Camera";
        break;

      default:
        cout << "Unknown";
    }
    cout << endl;

    cout << " |- Supported standards:";
    if (input.std == 0)
    {
      cout << " Unknown" << endl;
    }
    else
    {
      cout << endl;

      v4l2_standard standard;
      memset (&standard, 0, sizeof(standard));
      standard.index = 0;

      for (standard.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUMSTD, &standard) == 0; standard.index++)
      {
        if (standard.id & input.std) cout << "  + " << standard.name << endl;
      }
    }
  }
  if (input.index == 0) cout << "None" << endl;
  cout << endl;

  /* Outputs */
  cout << "Outputs:" << endl;
  v4l2_output output;
  memset (&output, 0, sizeof(output));

  for (output.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUMOUTPUT, &output) == 0; output.index++)
  {
    cout << " + Output " << output.index << ": " << output.name << endl;

    cout << " |- Type: ";
    switch (output.type)
    {
      case V4L2_OUTPUT_TYPE_MODULATOR:
        cout << "TV Modulator";
        break;

      case V4L2_OUTPUT_TYPE_ANALOG:
        cout << "Analog output";
        break;

      default:
        cout << "Unknown";
    }
    cout << endl;

    cout << " |- Supported standards:";
    if (output.std == 0)
    {
      cout << " Unknown" << endl;
    }
    else
    {
      cout << endl;

      v4l2_standard standard;
      memset (&standard, 0, sizeof (standard));
      standard.index = 0;

      for (standard.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUMSTD, &standard) == 0; standard.index++)
      {
        if (standard.id & output.std) cout << "  + " << standard.name << endl;
      }
    }
  }
  if (output.index == 0) cout << "None" << endl;
  cout << endl;

  /* Supported formats */
  cout << "Formats:" << endl;
  v4l2_fmtdesc format_desc;
  memset(&format_desc, 0, sizeof(format_desc));
  format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  char fourcc[5] = "    ";
  for (format_desc.index = 0; v4l2_ioctl(_dev, VIDIOC_ENUM_FMT, &format_desc) == 0; format_desc.index++)
  {
    fourcc[0] = static_cast<char>(format_desc.pixelformat & 0xFF);
    fourcc[1] = static_cast<char>((format_desc.pixelformat >> 8) & 0xFF);
    fourcc[2] = static_cast<char>((format_desc.pixelformat >> 16) & 0xFF);
    fourcc[3] = static_cast<char>((format_desc.pixelformat >> 24) & 0xFF);

    colorspace_t cs = CS_UNKNOWN;
    if (strcmp(fourcc, "RGB3") == 0) cs = RGB;
    else if (strcmp(fourcc, "Y41P") == 0) cs = YUV411_PACKED; //different byte ordering
    else if (strcmp(fourcc, "411P") == 0) cs = YUV411_PLANAR;
    else if (strcmp(fourcc, "YUYV") == 0) cs = YUY2;
    else if (strcmp(fourcc, "BGR3") == 0) cs = BGR;
    else if (strcmp(fourcc, "UYVY") == 0) cs = YUV422_PACKED;
    else if (strcmp(fourcc, "422P") == 0) cs = YUV422_PLANAR;
    else if (strcmp(fourcc, "GREY") == 0) cs = GRAY8;
    else if (strcmp(fourcc, "RGB4") == 0) cs = RGB_WITH_ALPHA;
    else if (strcmp(fourcc, "BGR4") == 0) cs = BGR_WITH_ALPHA;
    else if (strcmp(fourcc, "BA81") == 0) cs = BAYER_MOSAIC_BGGR;
    else if (strcmp(fourcc, "Y16 ") == 0) cs = MONO16;

    cout << " + Format " << format_desc.index << ": " << fourcc <<
      " (" << format_desc.description << ")";
    if (format_desc.flags & V4L2_FMT_FLAG_COMPRESSED) cout << " [Compressed]";
    cout << endl << " |- Colorspace: " << colorspace_to_string(cs) << endl;
  }
  cout << endl;

  /* Current Format */
  v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (v4l2_ioctl(_dev, VIDIOC_G_FMT, &format)) throw Exception("V4L2Cam: Format query failed");
  fourcc[0] = static_cast<char>(format.fmt.pix.pixelformat & 0xFF);
  fourcc[1] = static_cast<char>((format.fmt.pix.pixelformat >> 8) & 0xFF);
  fourcc[2] = static_cast<char>((format.fmt.pix.pixelformat >> 16) & 0xFF);
  fourcc[3] = static_cast<char>((format.fmt.pix.pixelformat >> 24) & 0xFF);

  cout << " Current Format:" << endl <<
    " " << format.fmt.pix.width << "x" << format.fmt.pix.height <<
    " (" << fourcc << ")" << endl <<
    " " << format.fmt.pix.bytesperline << " bytes per line" << endl <<
    " Total size: " << format.fmt.pix.sizeimage << endl;

  /* Supported Controls */
  cout << "Controls:" << endl;
  v4l2_queryctrl queryctrl;
  v4l2_querymenu querymenu;

  memset(&queryctrl, 0, sizeof(queryctrl));

  for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_LASTP1;
       queryctrl.id++)
  {
    if (v4l2_ioctl(_dev, VIDIOC_QUERYCTRL, &queryctrl))
    {
      if (errno == EINVAL) continue;

      cout << "Control query failed" << endl;
      return;
    }
    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) continue;

    cout << " + " <<  queryctrl.name << " [" <<
      (queryctrl.id - V4L2_CID_BASE) << "] (";
    switch (queryctrl.type)
    {
      case V4L2_CTRL_TYPE_INTEGER:
        cout << "int [" << queryctrl.minimum << "-" << queryctrl.maximum <<
          " /" << queryctrl.step << " def " << queryctrl.default_value <<
          "]";
        break;

      case V4L2_CTRL_TYPE_MENU:
        cout << "menu [def " << queryctrl.default_value << "]";
        break;

      case V4L2_CTRL_TYPE_BOOLEAN:
        cout << "bool [def " << queryctrl.default_value << "]";
        break;

      case V4L2_CTRL_TYPE_BUTTON:
        cout << "button";
        break;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
      case V4L2_CTRL_TYPE_INTEGER64:
        cout << "int64";
        break;

      case V4L2_CTRL_TYPE_CTRL_CLASS:
        cout << "ctrl_class";
        break;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
      case V4L2_CTRL_TYPE_STRING:
        cout << "string";
        break;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,41)
      case V4L2_CTRL_TYPE_BITMASK:
        cout << "bitmask";
        break;
#endif
    }
    cout << ")" << endl;

    if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
    {
      cout << " |- Menu items:" << endl;

      memset(&querymenu, 0, sizeof(querymenu));
      querymenu.id = queryctrl.id;

      for (querymenu.index = queryctrl.minimum;
           querymenu.index <= static_cast<unsigned long int>(queryctrl.maximum);
           querymenu.index++)
      {
        if (v4l2_ioctl(_dev, VIDIOC_QUERYMENU, &querymenu))
        {
          cout << "Getting menu items failed" << endl;
          return;
        }
        cout << " |   + " << querymenu.name << endl;
      }
    }
  }
  if (queryctrl.id == V4L2_CID_BASE) cout << "None" << endl;
  cout << endl;

  /* Supported Private Controls */
  cout << "Private Controls:" << endl;
  for (queryctrl.id = V4L2_CID_PRIVATE_BASE; ; queryctrl.id++)
  {
    if (v4l2_ioctl(_dev, VIDIOC_QUERYCTRL, &queryctrl))
    {
      if (errno == EINVAL) break;

      cout << "Private Control query failed" << endl;
      return;
    }

    if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) continue;

    cout << " + " <<  queryctrl.name << " [" <<
      (queryctrl.id - V4L2_CID_PRIVATE_BASE) << "] (";
    switch (queryctrl.type)
    {
      case V4L2_CTRL_TYPE_INTEGER:
        cout << "int [" << queryctrl.minimum << "-" << queryctrl.maximum <<
          " /" << queryctrl.step << " def " << queryctrl.default_value <<
          "]";
        break;

      case V4L2_CTRL_TYPE_MENU:
        cout << "menu [def " << queryctrl.default_value << "]";
        break;

      case V4L2_CTRL_TYPE_BOOLEAN:
        cout << "bool [def " << queryctrl.default_value << "]";
        break;

      case V4L2_CTRL_TYPE_BUTTON:
        cout << "button";
        break;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17)
      case V4L2_CTRL_TYPE_INTEGER64:
        cout << "int64";
        break;

      case V4L2_CTRL_TYPE_CTRL_CLASS:
        cout << "ctrl_class";
        break;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
      case V4L2_CTRL_TYPE_STRING:
        cout << "string";
        break;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0) || LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,41)
      case V4L2_CTRL_TYPE_BITMASK:
        cout << "bitmask";
        break;
#endif
    }
    cout << ")" << endl;

    if (queryctrl.type == V4L2_CTRL_TYPE_MENU)
    {
      cout << " |- Menu items:" << endl;

      memset(&querymenu, 0, sizeof(querymenu));
      querymenu.id = queryctrl.id;

      for (querymenu.index = queryctrl.minimum;
           querymenu.index <= static_cast<unsigned long int>(queryctrl.maximum);
           querymenu.index++)
      {
        if (v4l2_ioctl(_dev, VIDIOC_QUERYMENU, &querymenu))
        {
          cout << "Getting menu items failed" << endl;
          return;
        }
        cout << " |   + " << querymenu.name << endl;
      }
    }
  }
  if (queryctrl.id == V4L2_CID_PRIVATE_BASE) cout << "None" << endl;

  cout <<
    "=========================================================================="
    << endl;
}

} // end namespace firevision
