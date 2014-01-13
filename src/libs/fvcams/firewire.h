
/***************************************************************************
 *  firewire.h - This header defines a Firewire 1394 cam
 *
 *  Generated: Tue Feb 22 10:36:39 2005
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

#ifndef __FIREVISION_CAMS_FIREWIRE_H_
#define __FIREVISION_CAMS_FIREWIRE_H_

#include <fvcams/camera.h>
#include <fvcams/control/focus.h>

#include <dc1394/dc1394.h>

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif
#include <stdint.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CameraArgumentParser;

class FirewireCamera
: public Camera,
  public CameraControlFocus
{

 public:

  FirewireCamera(dc1394framerate_t framerate = DC1394_FRAMERATE_30,
		 dc1394video_mode_t mode     = DC1394_VIDEO_MODE_640x480_YUV422,
		 dc1394speed_t speed         = DC1394_ISO_SPEED_400,
		 int num_buffers=8);
  FirewireCamera(const CameraArgumentParser *cap);

  virtual ~FirewireCamera();

  virtual void open_device();
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

  virtual bool           auto_focus();
  virtual void           set_auto_focus(bool enabled);

  virtual unsigned int   focus();
  virtual void           set_focus(unsigned int focus);
  virtual unsigned int   focus_min();
  virtual unsigned int   focus_max();

  virtual bool           auto_shutter();
  virtual void           set_auto_shutter(bool enabled);
  
  virtual unsigned int   shutter();
  virtual void           set_shutter(unsigned int shutter);

  virtual bool           auto_white_balance();
  virtual void           set_auto_white_balance(bool enabled);

  virtual void           white_balance(unsigned int *ub, unsigned int *vr);
  virtual void           set_white_balance(unsigned int ub, unsigned int vr);

  virtual void           set_gain(unsigned int gain);

  virtual void           parse_set_focus(const char *focus);
  virtual void           parse_set_white_balance(const char *white_balance);
  virtual void           parse_set_shutter(const char *shutter);

  virtual uint64_t       guid() const;
  virtual const char *   model() const;

  static  void           print_available_fwcams();

  /** Get underlying libdc1394 C handle.
   * @return libdc1394 handle */
  dc1394_t * cobj() const
  { return _dc1394; }

  /** Get underlying libdc1394 camera C handle.
   * @return libdc1394 camera handle */
  dc1394camera_t * camera_cobj() const
  { return _camera; }

 protected:
  /** Number of DMA buffers. */
  int  _num_buffers;
  /** true if device has been opened, false otherwise */
  bool _device_opened;
  /** true if camera has been opened, false otherwise */
  bool _opened;
  /** true if camera has been started, false otherwise */
  bool _started;
  /** true if auto focus is enabled, false if disabled */
  bool _auto_focus;
  /** true if auto shutter is enabled, false if disabled */
  bool _auto_shutter;
  /** true if auto white balance is enabled, false if disabled */
  bool _auto_white_balance;
  /** true, if a valid frame has been received, false otherwise */
  bool _valid_frame_received;
  /** true if the shutter should actually be set, false otherwise */
  bool _do_set_shutter;
  /** true if the white balance should actually be set, false otherwise */
  bool _do_set_white_balance;
  /** true if the focus should actually be set, false otherwise */
  bool _do_set_focus;

  /** DC1394 main context */
  dc1394_t                      *_dc1394;
  /** DC1394 video mode */
  dc1394video_mode_t             _mode;
  /** Indicator of Format7 status. */
  bool                           _format7_mode_enabled;
  /** DC1394 speed */
  dc1394speed_t                  _speed;
  /** DC1394 framerate */
  dc1394framerate_t              _framerate;
  /** DC1394 camera handle */
  dc1394camera_t                *_camera;
  /** Last captured DC1394 video frame */
  dc1394video_frame_t           *_frame;
  /** Format7 color coding */
  dc1394color_coding_t           _format7_coding;
  /** Format7 bytes per packet */
  int                            _format7_bpp;
  /** Format7 width */
  int                            _format7_width;
  /** Format7 height */
  int                            _format7_height;
  /** Format7 ROI Start X coordinate */
  int                            _format7_startx;
  /** Format7 ROI Start Y coordinate */
  int                            _format7_starty;

  /** White balance U/B value */
  unsigned int                   _white_balance_ub;
  /** White balance V/R value */
  unsigned int                   _white_balance_vr;

  /** Shutter value */
  unsigned int                   _shutter;

  /** Focus value */
  unsigned int                   _focus;

  /** Gain value */
  unsigned int                   _gain;
  /** True, if gain is set automatically */
  bool                           _auto_gain;

  /** Camera model, used in open to identify the camera, if empty first found camera is used */
  char *_model;

};

} // end namespace firevision

#endif
